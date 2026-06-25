"""
exp_per_cell_bhtp_metrics.py

Computes three per-beam-slot metrics for BHTP scheduling analysis.

Grouping key: cell_idx (0–24), the phased-array beam direction index.
This answers "for beam slot j, how often does it have usable SNR?" —
the correct question for a BHTP scheduler that picks ONE beam per time step.

Grouping by geographic lat/lon (the previous approach) was incorrect: it
produced ~2000+ bins because the beam footprint sweeps many locations as the
satellite moves, yielding misleading ~0.6% service opportunity figures.

The denominator for Service Opportunity is the number of ticks where ANY beam
of ANY satellite is recorded in the dataset (i.e. the satellite is in range).
Ticks with no satellite data at all are counted as gap ticks.

Metrics:
  1. Service Opportunity  : fraction of in-range ticks where cell_idx j has
                            SNR >= threshold, across all visible satellites (%)
  2. Longest Gap Duration : longest consecutive run of all ticks (including
                            satellite-absent ticks) where cell_idx j has
                            SNR < threshold (seconds)
  3. SNR Margin           : median SNR during covered ticks minus threshold (dB)

Usage (4 priority scenarios):
  python exp_per_cell_bhtp_metrics.py `
      --data-dir  ns_result/0611/nbeams/iridium/deg25/Helsinki_out `
      --scene     iridium_bh_helsinki_deg25 `
      --snr-thresh 10.0 `
      --out-dir   data/processed

Output files (per run):
  data/processed/<scene>_per_cell_metrics.csv   — per-cell_idx rows (25 rows)
  data/processed/<scene>_per_cell_meta.json      — run config + summary stats
"""

import argparse
import csv
import json
import math
import os
import statistics
from collections import defaultdict

SNR_COL = "snr_dB"


# ── data loading ───────────────────────────────────────────────────────────────

def load_per_cell_idx_snr_timeseries(data_dir: str) -> tuple[dict, int, int]:
    """
    Read all sat_*_cells.csv files and build a per-beam-slot SNR time series.

    Grouping key: cell_idx (int, 0–24).
    At each tick, if multiple satellites report data for the same cell_idx, we
    keep the best (greedy-max) SNR — the scheduler can choose the best satellite.

    Returns:
        cell_series : {cell_idx: {t_int: best_snr_dB}}
        t_min       : earliest tick with any data (int seconds)
        t_max       : latest tick with any data (int seconds)
    """
    cell_series: dict[int, dict[int, float]] = defaultdict(dict)
    t_min, t_max = None, None

    for fname in sorted(os.listdir(data_dir)):
        if not fname.endswith("_cells.csv"):
            continue
        fpath = os.path.join(data_dir, fname)
        with open(fpath, newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f, skipinitialspace=True)
            for row in reader:
                try:
                    snr = float(row[SNR_COL])
                    if math.isnan(snr):
                        continue
                    t        = int(round(float(row["time_s"])))
                    cell_idx = int(row["cell_idx"])
                except (ValueError, KeyError):
                    continue

                # Greedy-max across satellites for the same beam slot at the same tick
                if t not in cell_series[cell_idx] or snr > cell_series[cell_idx][t]:
                    cell_series[cell_idx][t] = snr

                if t_min is None or t < t_min:
                    t_min = t
                if t_max is None or t > t_max:
                    t_max = t

    return dict(cell_series), t_min, t_max


# ── metric computation ─────────────────────────────────────────────────────────

def _longest_consecutive_run(flags: list[bool]) -> int:
    """Return the length of the longest consecutive run of True values in flags."""
    max_run = cur = 0
    for v in flags:
        if v:
            cur += 1
            if cur > max_run:
                max_run = cur
        else:
            cur = 0
    return max_run


def compute_per_cell_metrics(
    cell_series: dict,
    t_min: int,
    t_max: int,
    snr_thresh: float,
) -> list[dict]:
    """
    For each beam slot (cell_idx 0–24), compute the three BHTP scheduling metrics.

    Two denominators are reported:
      n_inrange_ticks : ticks where ANY beam has data (satellite is overhead)
      n_total_ticks   : full window [t_min, t_max] including satellite-absent ticks

    Service Opportunity uses n_inrange_ticks so that 100% means "served every
    tick the satellite was visible" — the relevant measure for BHTP scheduling.

    Longest Gap uses the full window so gap spans between passes are captured.

    Returns a list of 25 dicts (one per cell_idx), sorted by cell_idx.
    """
    all_ticks = list(range(t_min, t_max + 1))
    n_total = len(all_ticks)

    # Ticks where at least one beam has data (satellite is in range)
    inrange_ticks = set(t for ts in cell_series.values() for t in ts)
    n_inrange = len(inrange_ticks)

    results = []
    for cell_idx in sorted(cell_series.keys()):
        t_to_snr = cell_series[cell_idx]

        # Per-tick SNR: None if satellite not in range or no data for this beam
        snr_seq = [t_to_snr.get(t) for t in all_ticks]
        covered = [s is not None and s >= snr_thresh for s in snr_seq]

        # 1. Service Opportunity (relative to in-range ticks) ─────────────────
        n_covered = sum(covered)
        # Among in-range ticks only
        covered_inrange = sum(
            1 for t, c in zip(all_ticks, covered) if c and t in inrange_ticks
        )
        service_opp_pct = 100.0 * covered_inrange / n_inrange if n_inrange else 0.0

        # 2. Longest Gap (full window, including satellite-absent ticks) ───────
        gap_flags = [not c for c in covered]
        longest_gap_s = _longest_consecutive_run(gap_flags)

        # 3. SNR Margin ───────────────────────────────────────────────────────
        snr_covered = [s for s, c in zip(snr_seq, covered) if c]
        if snr_covered:
            snr_median = statistics.median(snr_covered)
            snr_mean   = statistics.mean(snr_covered)
            snr_margin = snr_median - snr_thresh
        else:
            snr_median = snr_mean = snr_margin = float("nan")

        results.append({
            "cell_idx":          cell_idx,
            "service_opp_pct":   round(service_opp_pct, 3),   # % of in-range ticks
            "longest_gap_s":     longest_gap_s,                # full-window gap
            "snr_median_dB":     round(snr_median, 3) if not math.isnan(snr_median) else "nan",
            "snr_mean_dB":       round(snr_mean,   3) if not math.isnan(snr_mean)   else "nan",
            "snr_margin_dB":     round(snr_margin,  3) if not math.isnan(snr_margin) else "nan",
            "n_covered_ticks":   covered_inrange,
            "n_inrange_ticks":   n_inrange,
            "n_total_ticks":     n_total,
        })

    return results


def summary_stats(results: list[dict]) -> dict:
    """Compute ROI-level summary statistics across all bins."""
    def _vals(key):
        return [r[key] for r in results if r[key] != "nan" and not
                (isinstance(r[key], float) and math.isnan(r[key]))]

    opp  = _vals("service_opp_pct")
    gap  = _vals("longest_gap_s")
    marg = _vals("snr_margin_dB")

    def _safe_stats(vals):
        if not vals:
            return {"mean": "nan", "min": "nan", "max": "nan", "median": "nan"}
        return {
            "mean":   round(statistics.mean(vals),   3),
            "min":    round(min(vals),                3),
            "max":    round(max(vals),                3),
            "median": round(statistics.median(vals),  3),
        }

    return {
        "n_bins":                 len(results),
        "service_opp_pct":        _safe_stats(opp),
        "longest_gap_s":          _safe_stats(gap),
        "snr_margin_dB":          _safe_stats(marg),
        "n_bins_never_served":    sum(r["service_opp_pct"] == 0.0 for r in results),
        "n_bins_always_served":   sum(r["service_opp_pct"] == 100.0 for r in results),
    }


# ── I/O ────────────────────────────────────────────────────────────────────────

def write_csv(results: list[dict], out_path: str) -> None:
    if not results:
        return
    fieldnames = list(results[0].keys())
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(results)


def write_meta(meta: dict, out_path: str) -> None:
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2)


# ── CLI ────────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="Compute per-geographic-bin BHTP scheduling metrics"
    )
    p.add_argument("--data-dir",   required=True,
                   help="Directory containing sat_*_cells.csv files")
    p.add_argument("--scene",      required=True,
                   help="Scene identifier used as filename prefix (e.g. iridium_bh_helsinki_deg25)")
    p.add_argument("--snr-thresh", type=float, default=10.0,
                   help="SNR threshold in dB for 'covered' determination (default: 10.0)")
    p.add_argument("--out-dir",    default="data/processed",
                   help="Output directory for CSV and JSON files")
    return p.parse_args()


def main():
    args = parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    print(f"[load]    Reading CSVs from: {args.data_dir}")
    cell_series, t_min, t_max = load_per_cell_idx_snr_timeseries(args.data_dir)

    if not cell_series:
        print("[error]   No valid data found. Check --data-dir path.")
        return

    n_cells = len(cell_series)
    n_ticks = t_max - t_min + 1
    inrange = set(t for ts in cell_series.values() for t in ts)
    print(f"[load]    {n_cells} beam slots (cell_idx), ticks {t_min}–{t_max} "
          f"({n_ticks} s window, {len(inrange)} s satellite in-range)")
    print(f"[compute] SNR threshold = {args.snr_thresh} dB")

    results = compute_per_cell_metrics(cell_series, t_min, t_max, args.snr_thresh)

    summary = summary_stats(results)
    print(f"[summary] Service Opportunity:  mean={summary['service_opp_pct']['mean']}%  "
          f"min={summary['service_opp_pct']['min']}%")
    print(f"[summary] Longest Gap:          mean={summary['longest_gap_s']['mean']}s  "
          f"max={summary['longest_gap_s']['max']}s")
    print(f"[summary] SNR Margin:           mean={summary['snr_margin_dB']['mean']}dB  "
          f"min={summary['snr_margin_dB']['min']}dB")
    print(f"[summary] Bins never served:    {summary['n_bins_never_served']}")

    # Write outputs
    csv_path  = os.path.join(args.out_dir, f"{args.scene}_per_cell_metrics.csv")
    meta_path = os.path.join(args.out_dir, f"{args.scene}_per_cell_meta.json")

    meta = {
        "scene":            args.scene,
        "data_dir":         args.data_dir,
        "snr_thresh":       args.snr_thresh,
        "grouping":         "cell_idx (beam slot 0-24)",
        "t_min":            t_min,
        "t_max":            t_max,
        "window_s":         n_ticks,
        "inrange_ticks":    len(inrange),
        "note": (
            "service_opp_pct is relative to inrange_ticks (satellite visible), "
            "longest_gap_s uses the full window including satellite-absent time."
        ),
        "summary":          summary,
    }

    write_csv(results,  csv_path)
    write_meta(meta,    meta_path)

    print(f"[write]   {csv_path}")
    print(f"[write]   {meta_path}")


if __name__ == "__main__":
    main()
