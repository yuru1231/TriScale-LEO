"""
exp_gap1_snr_alignment.py  (Gap 1)

Time-aligned SNR comparison between BH (1-beam focused) and 25-beam simultaneous modes.

Both datasets must use the same constellation (Starlink) and the same elevation mask.
The alignment key is (sat_id, time_s, cell_idx) so that satellite geometry is exactly
controlled — only beam activation strategy differs between the two matched values.

Theoretical prediction:
  ΔSNR = SNR_BH − SNR_25beam
       = G_peak_BH − G_peak_25beam
       = 60.5 dBi − 46.52 dBi (= 60.5 − 10·log10(25))
       = 13.98 dB  (constant, geometry-independent)

If the raw (unaligned) comparison shows 16.5 dB, it means the two datasets covered
different satellite passes with different elevation distributions.  After alignment,
the mean ΔSNR should converge to 13.98 ± ε dB.

Usage:
  python analysis/exp_gap1_snr_alignment.py \
      --dir-bh      ns_result/0611/nbeams/starlink/deg25/Helsinki_out \
      --dir-25beam  ns_result/0611/25beams/deg25/Helsinki_out \
      --scene       starlink_helsinki_deg25 \
      --out-dir     data/processed

Output:
  data/processed/<scene>_gap1_aligned.csv   — per-pair ΔSNR table
  data/processed/<scene>_gap1_meta.json     — summary statistics
  ns_result/0611/figures/paper/fig_gap1_delta_snr_<scene>.png / .svg
"""

import argparse
import csv
import json
import math
import os
import statistics
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


SNR_COL = "snr_dB"
THEORETICAL_DELTA = 10 * math.log10(25)   # 13.979 dB


# ── data loading ───────────────────────────────────────────────────────────────

def load_snr_table(data_dir: str) -> dict:
    """
    Load all sat_*_cells.csv files and return:
        {(sat_id, t_int, cell_idx): snr_dB}

    If multiple rows share the same key (shouldn't happen within one mode),
    keep the best SNR.
    """
    table: dict[tuple, float] = {}

    for fname in sorted(os.listdir(data_dir)):
        if not fname.endswith("_cells.csv"):
            continue
        sat_id = fname.replace("_cells.csv", "")
        fpath  = os.path.join(data_dir, fname)
        with open(fpath, newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f, skipinitialspace=True)
            for row in reader:
                try:
                    snr      = float(row[SNR_COL])
                    if math.isnan(snr):
                        continue
                    t        = int(round(float(row["time_s"])))
                    cell_idx = int(row["cell_idx"])
                except (ValueError, KeyError):
                    continue
                key = (sat_id, t, cell_idx)
                if key not in table or snr > table[key]:
                    table[key] = snr

    return table


# ── alignment & ΔSNR ──────────────────────────────────────────────────────────

def compute_delta_snr(
    table_bh: dict,
    table_25: dict,
) -> list[dict]:
    """
    Match keys present in BOTH tables and compute ΔSNR = SNR_BH − SNR_25beam.
    Returns list of dicts with fields: sat_id, t, cell_idx, snr_bh, snr_25, delta_snr.
    """
    common_keys = set(table_bh.keys()) & set(table_25.keys())
    rows = []
    for sat_id, t, cell_idx in sorted(common_keys):
        snr_bh  = table_bh[(sat_id, t, cell_idx)]
        snr_25  = table_25[(sat_id, t, cell_idx)]
        delta   = snr_bh - snr_25
        rows.append({
            "sat_id":    sat_id,
            "t_s":       t,
            "cell_idx":  cell_idx,
            "snr_bh":    round(snr_bh,  3),
            "snr_25":    round(snr_25,  3),
            "delta_snr": round(delta,   4),
        })
    return rows


# ── statistics ─────────────────────────────────────────────────────────────────

def compute_stats(rows: list[dict]) -> dict:
    deltas = [r["delta_snr"] for r in rows]
    snr_bh = [r["snr_bh"]   for r in rows]
    snr_25 = [r["snr_25"]   for r in rows]

    def _s(vals):
        if not vals:
            return {"mean": None, "median": None, "std": None, "min": None, "max": None}
        return {
            "mean":   round(statistics.mean(vals),   4),
            "median": round(statistics.median(vals),  4),
            "std":    round(statistics.stdev(vals),   4) if len(vals) > 1 else 0.0,
            "min":    round(min(vals),                4),
            "max":    round(max(vals),                4),
        }

    return {
        "n_matched_pairs":    len(rows),
        "n_unique_sats":      len({r["sat_id"] for r in rows}),
        "n_unique_ticks":     len({r["t_s"]    for r in rows}),
        "delta_snr":          _s(deltas),
        "snr_bh":             _s(snr_bh),
        "snr_25":             _s(snr_25),
        "theoretical_delta":  round(THEORETICAL_DELTA, 4),
        "delta_vs_theory":    round(
            (statistics.mean(deltas) - THEORETICAL_DELTA) if deltas else float("nan"), 4
        ),
    }


# ── figure ─────────────────────────────────────────────────────────────────────

def plot_delta_histogram(rows: list[dict], stats: dict, scene: str, out_dir: str) -> None:
    """
    Histogram of ΔSNR = SNR_BH − SNR_25beam.
    Vertical lines: theoretical value (13.98 dB) and empirical mean.
    """
    deltas = [r["delta_snr"] for r in rows]
    if not deltas:
        print("[warn] No matched pairs — skipping histogram")
        return

    fig, ax = plt.subplots(figsize=(9, 5))

    n_bins = min(60, max(10, len(deltas) // 20))
    ax.hist(deltas, bins=n_bins, color="#2196F3", edgecolor="black",
            linewidth=0.4, alpha=0.85, label=f"ΔSNR samples (n={len(deltas)})")

    ax.axvline(THEORETICAL_DELTA,
               color="#C62828", linestyle="--", linewidth=1.8,
               label=f"Theoretical: {THEORETICAL_DELTA:.2f} dB (= 10·log₁₀25)")

    mean_d = stats["delta_snr"]["mean"]
    ax.axvline(mean_d,
               color="#2E7D32", linestyle="-", linewidth=1.8,
               label=f"Empirical mean: {mean_d:.3f} dB")

    bias = stats["delta_vs_theory"]
    ax.set_xlabel("ΔSNR = SNR_BH − SNR_25beam (dB)", fontsize=12)
    ax.set_ylabel("Count", fontsize=12)
    ax.set_title(
        f"Gap 1: Time-Aligned SNR Gain — {scene}\n"
        f"n = {len(deltas)} matched pairs | "
        f"bias vs theory = {bias:+.3f} dB",
        fontsize=12,
    )
    ax.legend(fontsize=11)
    ax.grid(alpha=0.3)
    fig.tight_layout()

    fname_base = f"fig_gap1_delta_snr_{scene}"
    for ext in ("png", "svg"):
        out_path = os.path.join(out_dir, f"{fname_base}.{ext}")
        dpi = 300 if ext == "png" else None
        fig.savefig(out_path, dpi=dpi, bbox_inches="tight")
        print(f"[write] {out_path}")

    plt.close(fig)


# ── I/O ────────────────────────────────────────────────────────────────────────

def write_csv(rows: list[dict], out_path: str) -> None:
    if not rows:
        return
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_json(obj: dict, out_path: str) -> None:
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(obj, f, indent=2)


# ── CLI ────────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="Gap 1: time-aligned ΔSNR analysis (BH vs 25-beam, same Starlink constellation)"
    )
    p.add_argument("--dir-bh",     required=True,
                   help="Directory of sat_*_cells.csv for BH mode (Starlink)")
    p.add_argument("--dir-25beam", required=True,
                   help="Directory of sat_*_cells.csv for 25-beam mode (Starlink)")
    p.add_argument("--scene",      default="starlink_helsinki_deg25",
                   help="Scene label used in filenames and titles")
    p.add_argument("--out-dir",    default="data/processed",
                   help="Output directory for CSV / JSON")
    p.add_argument("--fig-dir",    default="ns_result/0611/figures/paper",
                   help="Output directory for PNG / SVG figures")
    return p.parse_args()


def main():
    args = parse_args()
    os.makedirs(args.out_dir,  exist_ok=True)
    os.makedirs(args.fig_dir,  exist_ok=True)

    print(f"[load] BH data from:     {args.dir_bh}")
    table_bh = load_snr_table(args.dir_bh)
    print(f"[load] {len(table_bh)} (sat, tick, cell) entries in BH dataset")

    print(f"[load] 25-beam data from: {args.dir_25beam}")
    table_25 = load_snr_table(args.dir_25beam)
    print(f"[load] {len(table_25)} (sat, tick, cell) entries in 25-beam dataset")

    print("[align] Computing ΔSNR on matched (sat, tick, cell) pairs …")
    rows  = compute_delta_snr(table_bh, table_25)
    stats = compute_stats(rows)

    print(f"[result] Matched pairs:    {stats['n_matched_pairs']}")
    print(f"[result] Unique sats:      {stats['n_unique_sats']}")
    print(f"[result] Unique ticks:     {stats['n_unique_ticks']}")
    print(f"[result] ΔSNR mean:        {stats['delta_snr']['mean']} dB")
    print(f"[result] ΔSNR std:         {stats['delta_snr']['std']} dB")
    print(f"[result] Theoretical:      {stats['theoretical_delta']} dB")
    print(f"[result] Bias vs theory:   {stats['delta_vs_theory']:+.4f} dB")

    if stats["n_matched_pairs"] == 0:
        print("[warn] No matched pairs found. Check that both directories use "
              "the same Starlink TLE epoch and elevation mask.")
        return

    csv_path  = os.path.join(args.out_dir, f"{args.scene}_gap1_aligned.csv")
    meta_path = os.path.join(args.out_dir, f"{args.scene}_gap1_meta.json")

    meta = {
        "scene":      args.scene,
        "dir_bh":     args.dir_bh,
        "dir_25beam": args.dir_25beam,
        "theoretical_delta_dB": stats["theoretical_delta"],
        "stats": stats,
        "note": (
            "ΔSNR = SNR_BH - SNR_25beam, matched by (sat_id, time_s, cell_idx). "
            "bias_vs_theory close to 0 confirms that the raw 16.5 dB gap was due "
            "to unaligned time windows, not a modelling error."
        ),
    }

    write_csv(rows,  csv_path)
    write_json(meta, meta_path)
    print(f"[write] {csv_path}")
    print(f"[write] {meta_path}")

    plot_delta_histogram(rows, stats, args.scene, args.fig_dir)
    print("[done]")


if __name__ == "__main__":
    main()
