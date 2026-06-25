"""
exp_gap4_hardcell_map.py  (Gap 4)

Geographic distribution map of hard cells — grid points that NEVER achieve
SNR >= threshold across the entire simulation window.

Used to visualize WHY 25-beam simultaneous mode at low elevation mask has high
hard-cell ratios (e.g., Starlink Helsinki ≥5°: 86.5% hard cells).

Each 0.05° geographic bin is coloured by its maximum SNR across all ticks and
all visible satellites (Greedy-Max).  Red = hard cell (max SNR < threshold),
green = served at least once.

Usage:
  python analysis/exp_gap4_hardcell_map.py ^
      --data-dir  ns_result/0611/25beams/deg5/Helsinki_out ^
      --scene     starlink_25beam_helsinki_deg5 ^
      --snr-thresh 3.0 ^
      --out-dir   ns_result/0611/figures/paper

Output:
  ns_result/0611/figures/paper/fig4b_hardcell_map_<scene>.png / .svg
  data/processed/<scene>_hardcell_meta.json
"""

import argparse
import csv
import json
import math
import os
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import numpy as np


GEO_BIN_DEG = 0.05   # geographic bin size, consistent with exp_comparative_analysis.py
SNR_COL     = "snr_dB"


# ── data loading ───────────────────────────────────────────────────────────────

def load_max_snr_per_bin(data_dir: str) -> dict[tuple, float]:
    """
    Read all sat_*_cells.csv and return the Greedy-Max SNR for each geographic bin.

    Returns:
        {(lat_bin, lon_bin): max_snr_dB}

    Only finite SNR values are counted.  A bin not present in this dict was never
    observed in any satellite pass during the simulation window.
    """
    def _bin(v: float) -> float:
        return round(round(v / GEO_BIN_DEG) * GEO_BIN_DEG, 6)

    bin_max: dict[tuple, float] = defaultdict(lambda: -math.inf)

    for fname in sorted(os.listdir(data_dir)):
        if not fname.endswith("_cells.csv"):
            continue
        fpath = os.path.join(data_dir, fname)
        with open(fpath, newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f, skipinitialspace=True)
            for row in reader:
                try:
                    snr = float(row[SNR_COL])
                    if math.isnan(snr) or math.isinf(snr):
                        continue
                    lb  = _bin(float(row["cell_lat_deg"]))
                    lob = _bin(float(row["cell_lon_deg"]))
                except (ValueError, KeyError):
                    continue
                key = (lb, lob)
                if snr > bin_max[key]:
                    bin_max[key] = snr

    return dict(bin_max)


# ── statistics ─────────────────────────────────────────────────────────────────

def compute_hardcell_stats(bin_max: dict, snr_thresh: float) -> dict:
    total     = len(bin_max)
    hard      = sum(1 for v in bin_max.values() if v < snr_thresh)
    served    = total - hard

    snr_vals  = list(bin_max.values())
    hard_snrs = [v for v in snr_vals if v < snr_thresh]
    good_snrs = [v for v in snr_vals if v >= snr_thresh]

    def _s(vals):
        if not vals:
            return {"mean": None, "min": None, "max": None}
        return {
            "mean": round(sum(vals) / len(vals), 3),
            "min":  round(min(vals), 3),
            "max":  round(max(vals), 3),
        }

    return {
        "n_bins_total":      total,
        "n_bins_hard":       hard,
        "n_bins_served":     served,
        "hard_cell_ratio":   round(hard / total * 100, 2) if total else 0.0,
        "snr_thresh_dB":     snr_thresh,
        "snr_hard_cells":    _s(hard_snrs),
        "snr_served_cells":  _s(good_snrs),
    }


# ── figure ─────────────────────────────────────────────────────────────────────

def plot_hardcell_map(
    bin_max:    dict,
    stats:      dict,
    snr_thresh: float,
    scene:      str,
    out_dir:    str,
) -> None:
    """
    Scatter map: each geographic bin is coloured by its peak SNR.
      Red   = hard cell (peak SNR < threshold) — never served
      Green = served (peak SNR >= threshold)

    Two-panel layout:
      Left  : binary hard/served map
      Right : continuous SNR heatmap (full range)
    """
    lats = np.array([k[0] for k in bin_max])
    lons = np.array([k[1] for k in bin_max])
    snrs = np.array([bin_max[k] for k in bin_max])

    # ── build binary colour array for left panel ────────────────────────────
    colors_bin = ["#C62828" if s < snr_thresh else "#2E7D32" for s in snrs]

    fig, (ax_l, ax_r) = plt.subplots(1, 2, figsize=(14, 6),
                                      constrained_layout=True)

    hard_ratio = stats["hard_cell_ratio"]
    n_total    = stats["n_bins_total"]
    n_hard     = stats["n_bins_hard"]

    fig.suptitle(
        f"Fig.4b: Hard-Cell Geographic Distribution — {scene}\n"
        f"Hard cells (peak SNR < {snr_thresh:.0f} dB): {n_hard}/{n_total} = {hard_ratio:.1f}%",
        fontsize=13, fontweight="bold",
    )

    # ── left panel: binary ──────────────────────────────────────────────────
    ax_l.scatter(lons, lats, c=colors_bin, s=8, linewidths=0, alpha=0.8)

    # Proxy artists for legend
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], marker="o", color="w", markerfacecolor="#C62828",
               markersize=8, label=f"Hard cell (SNR < {snr_thresh:.0f} dB): {n_hard}"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="#2E7D32",
               markersize=8, label=f"Served (SNR ≥ {snr_thresh:.0f} dB): {n_total - n_hard}"),
    ]
    ax_l.legend(handles=legend_elements, fontsize=10, loc="lower right", framealpha=0.9)
    ax_l.set_xlabel("Longitude (°)", fontsize=11)
    ax_l.set_ylabel("Latitude (°)", fontsize=11)
    ax_l.set_title("Binary: Hard vs Served", fontsize=11)
    ax_l.grid(alpha=0.25)

    # ── right panel: continuous SNR ─────────────────────────────────────────
    vmin = max(snrs.min(), -20)
    vmax = snrs.max()
    norm = mcolors.Normalize(vmin=vmin, vmax=vmax)
    cmap = plt.cm.RdYlGn

    sc = ax_r.scatter(lons, lats, c=snrs, cmap=cmap, norm=norm,
                      s=8, linewidths=0, alpha=0.8)
    cbar = fig.colorbar(sc, ax=ax_r, shrink=0.85, pad=0.02)
    cbar.set_label("Peak SNR (dB)", fontsize=10)
    cbar.ax.axhline(snr_thresh, color="black", linestyle="--", linewidth=1.5,
                    label=f"Threshold {snr_thresh:.0f} dB")
    cbar.ax.text(1.05, snr_thresh, f" {snr_thresh:.0f} dB", va="center",
                 fontsize=8, transform=cbar.ax.get_yaxis_transform())

    ax_r.set_xlabel("Longitude (°)", fontsize=11)
    ax_r.set_ylabel("Latitude (°)", fontsize=11)
    ax_r.set_title("Peak SNR Heatmap", fontsize=11)
    ax_r.grid(alpha=0.25)

    fname_base = f"fig4b_hardcell_map_{scene}"
    for ext in ("png", "svg"):
        out_path = os.path.join(out_dir, f"{fname_base}.{ext}")
        dpi = 300 if ext == "png" else None
        fig.savefig(out_path, dpi=dpi, bbox_inches="tight")
        print(f"[write] {out_path}")

    plt.close(fig)


# ── I/O ────────────────────────────────────────────────────────────────────────

def write_meta(meta: dict, out_path: str) -> None:
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2)


# ── CLI ────────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="Gap 4: Hard-cell geographic map from 25-beam simultaneous mode data"
    )
    p.add_argument("--data-dir",   required=True,
                   help="Directory containing sat_*_cells.csv (25-beam mode)")
    p.add_argument("--scene",      required=True,
                   help="Scene identifier used in output filenames")
    p.add_argument("--snr-thresh", type=float, default=3.0,
                   help="SNR threshold in dB for hard-cell classification (default: 3.0)")
    p.add_argument("--out-dir",    default="ns_result/0611/figures/paper",
                   help="Output directory for PNG / SVG figures")
    p.add_argument("--meta-dir",   default="data/processed",
                   help="Output directory for meta JSON")
    return p.parse_args()


def main():
    args = parse_args()
    os.makedirs(args.out_dir,  exist_ok=True)
    os.makedirs(args.meta_dir, exist_ok=True)

    print(f"[load] Reading CSVs from: {args.data_dir}")
    bin_max = load_max_snr_per_bin(args.data_dir)
    print(f"[load] {len(bin_max)} geographic bins found")

    if not bin_max:
        print("[error] No data found. Check --data-dir path and CSV column names.")
        return

    stats = compute_hardcell_stats(bin_max, args.snr_thresh)

    print(f"[result] Total bins:      {stats['n_bins_total']}")
    print(f"[result] Hard cells:      {stats['n_bins_hard']}  ({stats['hard_cell_ratio']:.1f}%)")
    print(f"[result] Served cells:    {stats['n_bins_served']}")

    meta = {
        "scene":     args.scene,
        "data_dir":  args.data_dir,
        "snr_thresh_dB": args.snr_thresh,
        "stats":     stats,
        "note": (
            "hard_cell = geographic bin where peak Greedy-Max SNR < snr_thresh "
            "across the entire simulation window and all visible satellites."
        ),
    }
    meta_path = os.path.join(args.meta_dir, f"{args.scene}_hardcell_meta.json")
    write_meta(meta, meta_path)
    print(f"[write] {meta_path}")

    plot_hardcell_map(bin_max, stats, args.snr_thresh, args.scene, args.out_dir)
    print("[done]")


if __name__ == "__main__":
    main()
