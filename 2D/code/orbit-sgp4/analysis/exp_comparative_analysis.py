"""
analysis/exp_comparative_analysis.py
--------------------------------------
Comparative statistics across ALL 0611 scenarios.

For each (constellation, beam mode, city, elevation mask) combination, computes:
  - Coverage_greedy %   : fraction of ticks where any location has SNR >= threshold
  - Coverage_mrc %      : fraction of ticks where any location's MRC SNR >= threshold
  - Mean SNR (dB)       : mean of per-tick greedy-max SNR (over ticks with ANY satellite)
  - P5 SNR (dB)         : 5th percentile of per-tick greedy-max SNR
  - Hard-cell ratio %   : fraction of geographic bins never served above threshold
  - Window (s)          : total simulation window length

Outputs:
  <out-dir>/comparative_stats.csv         raw CSV for downstream use
  <out-dir>/comparative_stats_meta.json   config record
  Prints Markdown table to stdout

Usage
-----
cd 2D/code/orbit-sgp4/analysis
python exp_comparative_analysis.py [--snr-thresh 3.0] [--out-dir <path>]
"""

import argparse
import csv
import json
import math
import os
import sys
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from link_budget import mrc_combine_snr_db

# ---------------------------------------------------------------------------
# Constants: data layout mirrors plot_all_combinations.py
# ---------------------------------------------------------------------------

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_0611  = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "ns_result", "0611"))

GEO_BIN = 0.05  # degrees — must match check_coverage.py

# All scenarios present in 0611:
#   25beams / nbeams × {iridium, starlink} × {deg5, deg25, deg37} × {city}
# Starlink deg5 was not simulated (below operational spec).
# Helsinki nbeams/starlink deg37 folder: check_coverage showed 0 ticks for this combo.

SCENARIOS = [
    # (label, rel_path_from_BASE_0611)
    # ── nbeams / iridium ────────────────────────────────────────────────────
    ("Iridium", "BH",        "Helsinki",  "≥5°",  "nbeams/iridium/deg5/Helsinki_out"),
    ("Iridium", "BH",        "Helsinki",  "≥25°", "nbeams/iridium/deg25/Helsinki_out"),
    ("Iridium", "BH",        "Helsinki",  "≥37°", "nbeams/iridium/deg37/Helsinki_out"),
    ("Iridium", "BH",        "Tokyo",     "≥5°",  "nbeams/iridium/deg5/tokyo_out"),
    ("Iridium", "BH",        "Tokyo",     "≥25°", "nbeams/iridium/deg25/tokyo_out"),
    ("Iridium", "BH",        "Tokyo",     "≥37°", "nbeams/iridium/deg37/tokyo_out"),
    ("Iridium", "BH",        "Singapore", "≥5°",  "nbeams/iridium/deg5/singapore"),
    ("Iridium", "BH",        "Singapore", "≥25°", "nbeams/iridium/deg25/singapore"),
    ("Iridium", "BH",        "Singapore", "≥37°", "nbeams/iridium/deg37/singapore"),
    # ── nbeams / starlink ───────────────────────────────────────────────────
    ("Starlink", "BH",       "Helsinki",  "≥25°", "nbeams/starlink/deg25/Helsinki_out"),
    ("Starlink", "BH",       "Helsinki",  "≥37°", "nbeams/starlink/deg37/Helsinki_out"),
    ("Starlink", "BH",       "Tokyo",     "≥25°", "nbeams/starlink/deg25/tokyo_out"),
    ("Starlink", "BH",       "Tokyo",     "≥37°", "nbeams/starlink/deg37/tokyo_out"),
    ("Starlink", "BH",       "Singapore", "≥25°", "nbeams/starlink/deg25/singapore_out"),
    ("Starlink", "BH",       "Singapore", "≥37°", "nbeams/starlink/deg37/singapore"),
    # ── 25beams / starlink ──────────────────────────────────────────────────
    ("Starlink", "25-beam",  "Helsinki",  "≥5°",  "25beams/deg5/Helsinki_out"),
    ("Starlink", "25-beam",  "Helsinki",  "≥25°", "25beams/deg25/Helsinki_out"),
    ("Starlink", "25-beam",  "Helsinki",  "≥37°", "25beams/deg37/Helsinki_out"),
    ("Starlink", "25-beam",  "Tokyo",     "≥5°",  "25beams/deg5/tokyo_out"),
    ("Starlink", "25-beam",  "Tokyo",     "≥25°", "25beams/deg25/tokyo_out"),
    ("Starlink", "25-beam",  "Tokyo",     "≥37°", "25beams/deg37/tokyo_out"),
    ("Starlink", "25-beam",  "Singapore", "≥5°",  "25beams/deg5/singapore"),
    ("Starlink", "25-beam",  "Singapore", "≥25°", "25beams/deg25/singapore"),
    ("Starlink", "25-beam",  "Singapore", "≥37°", "25beams/deg37/singapore"),
]


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    default_out = os.path.join(BASE_0611, "figures")
    p = argparse.ArgumentParser(
        description="Compute comparative statistics across all 0611 scenarios."
    )
    p.add_argument("--snr-thresh", type=float, default=3.0,
                   help="SNR service threshold in dB (default 3.0)")
    p.add_argument("--out-dir", default=default_out,
                   help=f"Output directory for CSV and meta JSON "
                        f"(default: {default_out})")
    return p.parse_args()


# ---------------------------------------------------------------------------
# Data loader  (subset of check_coverage.py logic)
# ---------------------------------------------------------------------------

def _bin(v: float) -> float:
    return round(round(v / GEO_BIN) * GEO_BIN, 5)


def load_snr_by_tick(data_dir: str):
    """
    Load all sat_XXXXX_cells.csv files in data_dir.

    Returns
    -------
    tick_best_snr  : dict[int, float]  — greedy-max SNR per tick
    tick_mrc_snr   : dict[int, float]  — MRC-combined best SNR per tick
                     (max over all geographic bins of their MRC-combined SNR)
    bin_ever_served: dict[(lat, lon), bool] — whether bin was ever SNR >= thresh
                     (for hard-cell ratio; threshold passed in via closure-style)
    window_s       : int — t_max - t_min + 1  (0 if no data)
    n_files        : int

    tick_best_snr and tick_mrc_snr are over ticks where at least one satellite
    was visible (i.e., full-blackout ticks are excluded from SNR statistics).
    """
    # data[tick][(lat_bin, lon_bin)] = list[snr_dB]
    data = defaultdict(lambda: defaultdict(list))
    n_files = 0

    for fname in sorted(os.listdir(data_dir)):
        if not fname.endswith("_cells.csv"):
            continue
        n_files += 1
        path = os.path.join(data_dir, fname)
        with open(path, newline="", encoding="utf-8", errors="replace") as f:
            reader = csv.DictReader(f, skipinitialspace=True)
            if reader.fieldnames:
                reader.fieldnames = [n.strip() for n in reader.fieldnames]
            fields = reader.fieldnames or []
            has_geo = "cell_lat_deg" in fields and "cell_lon_deg" in fields
            for row in reader:
                if not row:
                    continue
                row = {(k.strip() if isinstance(k, str) else k): v
                       for k, v in row.items()}
                snr_str = row.get("snr_dB", "")
                if not snr_str:
                    continue
                snr = float(snr_str)
                if math.isnan(snr):
                    continue
                t = round(float(row["time_s"]))
                if has_geo:
                    lat_str = row.get("cell_lat_deg", "")
                    lon_str = row.get("cell_lon_deg", "")
                    if not lat_str or not lon_str:
                        continue
                    lat, lon = float(lat_str), float(lon_str)
                    if math.isnan(lat) or math.isnan(lon):
                        continue
                    key = (_bin(lat), _bin(lon))
                else:
                    key = (int(row.get("cell_idx", 0)), 0)
                data[t][key].append(snr)

    if not data:
        return {}, {}, {}, 0, 0

    all_times = sorted(data.keys())
    t_min, t_max = all_times[0], all_times[-1]
    window_s = t_max - t_min + 1

    # Aggregate all bins across all ticks
    bin_snr_max = defaultdict(lambda: -999.0)   # max SNR ever seen at this bin

    tick_best_snr = {}
    tick_mrc_snr  = {}

    for t in all_times:
        bins_at_t = data[t]
        # Greedy-max: best single-satellite SNR across ALL bins at this tick
        best = -999.0
        # MRC per bin then take max
        best_mrc = -999.0
        for k, snr_list in bins_at_t.items():
            s_best = max(snr_list)
            s_mrc  = mrc_combine_snr_db(snr_list)
            if s_best > best:
                best = s_best
            if s_mrc > best_mrc:
                best_mrc = s_mrc
            if s_best > bin_snr_max[k]:
                bin_snr_max[k] = s_best
        tick_best_snr[t] = best
        tick_mrc_snr[t]  = best_mrc

    return tick_best_snr, tick_mrc_snr, bin_snr_max, window_s, n_files


# ---------------------------------------------------------------------------
# Per-scenario statistics
# ---------------------------------------------------------------------------

def compute_stats(tick_best_snr: dict, tick_mrc_snr: dict,
                  bin_snr_max: dict, window_s: int,
                  thresh: float) -> dict:
    """Reduce loaded data to the six reported metrics."""
    if not tick_best_snr:
        return {
            "coverage_greedy_pct": float("nan"),
            "coverage_mrc_pct":    float("nan"),
            "mean_snr_db":         float("nan"),
            "p5_snr_db":           float("nan"),
            "hard_cell_pct":       float("nan"),
            "window_s":            window_s,
            "n_ticks_with_sat":    0,
        }

    ticks_with_sat = sorted(tick_best_snr.keys())
    n_ticks = len(ticks_with_sat)

    # Coverage = fraction of ticks with satellite visible that achieve threshold
    # (full-blackout ticks are not counted in numerator or denominator — they
    # represent gaps where no satellite is overhead at all, which is a coverage
    # gap independent of beam mode; BH controller cannot improve them)
    cov_greedy = sum(1 for t in ticks_with_sat if tick_best_snr[t] >= thresh) / n_ticks * 100
    cov_mrc    = sum(1 for t in ticks_with_sat if tick_mrc_snr[t]  >= thresh) / n_ticks * 100

    best_snrs = [tick_best_snr[t] for t in ticks_with_sat]
    best_snrs.sort()
    mean_snr = sum(best_snrs) / len(best_snrs)
    p5_idx   = max(0, int(len(best_snrs) * 0.05) - 1)
    p5_snr   = best_snrs[p5_idx]

    # Hard-cell: bins that were never served above threshold in the entire window
    n_bins  = len(bin_snr_max)
    n_hard  = sum(1 for v in bin_snr_max.values() if v < thresh)
    hard_pct = n_hard / n_bins * 100 if n_bins > 0 else float("nan")

    return {
        "coverage_greedy_pct": round(cov_greedy, 1),
        "coverage_mrc_pct":    round(cov_mrc, 1),
        "mean_snr_db":         round(mean_snr, 1),
        "p5_snr_db":           round(p5_snr, 1),
        "hard_cell_pct":       round(hard_pct, 1),
        "window_s":            window_s,
        "n_ticks_with_sat":    n_ticks,
    }


# ---------------------------------------------------------------------------
# Output helpers
# ---------------------------------------------------------------------------

def _fmt(v, unit=""):
    """Format a float or nan for table display."""
    if isinstance(v, float) and math.isnan(v):
        return "—"
    if isinstance(v, float):
        return f"{v:.1f}{unit}"
    return str(v)


def print_markdown_table(rows: list[dict]):
    header = (
        "| Constellation | Mode | City | Elev Mask | "
        "Coverage Greedy (%) | Coverage MRC (%) | "
        "Avg SNR (dB) | P5 SNR (dB) | Hard-cell (%) | Window (s) |"
    )
    sep = "|---|---|---|---|---|---|---|---|---|---|"
    print(header)
    print(sep)
    for r in rows:
        s = r["stats"]
        print(
            f"| {r['constellation']} | {r['mode']} | {r['city']} | {r['elev']} | "
            f"{_fmt(s['coverage_greedy_pct'], '%')} | "
            f"{_fmt(s['coverage_mrc_pct'], '%')} | "
            f"{_fmt(s['mean_snr_db'], ' dB')} | "
            f"{_fmt(s['p5_snr_db'], ' dB')} | "
            f"{_fmt(s['hard_cell_pct'], '%')} | "
            f"{s['window_s']} |"
        )


def write_csv(rows: list[dict], out_path: str):
    fieldnames = [
        "constellation", "mode", "city", "elev",
        "coverage_greedy_pct", "coverage_mrc_pct",
        "mean_snr_db", "p5_snr_db", "hard_cell_pct",
        "window_s", "n_ticks_with_sat",
    ]
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in rows:
            row_out = {
                "constellation":       r["constellation"],
                "mode":                r["mode"],
                "city":                r["city"],
                "elev":                r["elev"],
            }
            row_out.update(r["stats"])
            writer.writerow(row_out)
    print(f"\n  CSV  → {out_path}")


def write_meta(args, out_path: str):
    meta = {
        "script":     "exp_comparative_analysis.py",
        "date":       _today(),
        "source":     os.path.abspath(BASE_0611),
        "parameters": {"snr_thresh_db": args.snr_thresh},
    }
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2)
    print(f"  Meta → {out_path}")


def _today() -> str:
    import datetime
    return datetime.date.today().isoformat()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    args = parse_args()
    thresh = args.snr_thresh
    os.makedirs(args.out_dir, exist_ok=True)

    print(f"=== Comparative Statistics — 0611 dataset ===")
    print(f"SNR threshold : {thresh} dB")
    print(f"Base dir      : {BASE_0611}")
    print(f"Output dir    : {args.out_dir}\n")

    results = []

    for (constellation, mode, city, elev, rel_path) in SCENARIOS:
        data_dir = os.path.join(BASE_0611, rel_path.replace("/", os.sep))

        if not os.path.isdir(data_dir):
            # Try alternate city suffix (some folders use _out, some don't)
            alt = data_dir + "_out" if not data_dir.endswith("_out") else data_dir[:-4]
            if os.path.isdir(alt):
                data_dir = alt
            else:
                print(f"  [SKIP] {constellation} {mode} {city} {elev} — folder not found: {data_dir}")
                results.append({
                    "constellation": constellation,
                    "mode":          mode,
                    "city":          city,
                    "elev":          elev,
                    "stats": {
                        "coverage_greedy_pct": float("nan"),
                        "coverage_mrc_pct":    float("nan"),
                        "mean_snr_db":         float("nan"),
                        "p5_snr_db":           float("nan"),
                        "hard_cell_pct":       float("nan"),
                        "window_s":            0,
                        "n_ticks_with_sat":    0,
                    },
                })
                continue

        tick_best, tick_mrc, bin_max, window_s, n_files = load_snr_by_tick(data_dir)
        stats = compute_stats(tick_best, tick_mrc, bin_max, window_s, thresh)

        label = f"{constellation:8s} {mode:8s} {city:12s} {elev}"
        cov_g = _fmt(stats["coverage_greedy_pct"], "%")
        cov_m = _fmt(stats["coverage_mrc_pct"], "%")
        print(f"  {label}  files={n_files:3d}  "
              f"cov_greedy={cov_g:>7s}  cov_mrc={cov_m:>7s}  "
              f"mean={_fmt(stats['mean_snr_db'], 'dB'):>8s}  "
              f"p5={_fmt(stats['p5_snr_db'], 'dB'):>8s}  "
              f"hard={_fmt(stats['hard_cell_pct'], '%'):>7s}")

        results.append({
            "constellation": constellation,
            "mode":          mode,
            "city":          city,
            "elev":          elev,
            "stats":         stats,
        })

    # Outputs
    csv_path  = os.path.join(args.out_dir, "comparative_stats.csv")
    meta_path = os.path.join(args.out_dir, "comparative_stats_meta.json")
    write_csv(results, csv_path)
    write_meta(args, meta_path)

    print("\n=== Markdown Table (paste into result.md) ===\n")
    print_markdown_table(results)
    print()


if __name__ == "__main__":
    main()
