"""
analysis/fig_coverage.py
------------------------
Generates coverage and link-budget figures from constellation scan output.

Input  : ../out/  (or --out-dir)
         sat_XXXXX_cells.csv  — per-satellite per-cell SNR time series
Output : figures/fig_gap_timeline.png   — gap segments: Greedy vs MRC
         figures/fig_link_budget.png    — SNR vs elevation, threshold annotation
         figures/fig_mrc_vs_greedy.png  — bar comparison of gap duration

Usage
-----
cd 2D/code/orbit-sgp4/analysis
python fig_coverage.py [--out-dir ../out] [--snr-thresh 0.0]
"""

import argparse
import csv
import math
import os
import sys
from collections import defaultdict

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

# Allow importing sibling module regardless of working directory
sys.path.insert(0, os.path.dirname(__file__))
from link_budget import (
    snr_at_elevation,
    mrc_combine_snr_db,
    link_budget_table,
    critical_elevation,
)

# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(
        description="Generate coverage and link-budget figures."
    )
    p.add_argument("--out-dir",    default=".",
                   help="Directory containing sat_XXXXX_cells.csv files "
                        "(default: current working directory)")
    p.add_argument("--snr-thresh", type=float, default=0.0,
                   help="SNR service threshold in dB (default 0.0)")
    p.add_argument("--min-elev",   type=float, default=5.0,
                   help="Scanner geometric cutoff elevation in deg (default 5.0)")
    return p.parse_args()

# ---------------------------------------------------------------------------
# Data loading (identical logic to check_coverage.py)
# ---------------------------------------------------------------------------

def load_data(out_dir: str):
    """
    Returns data[time_s][cell_idx] = list[snr_dB] across all satellites.
    """
    data = defaultdict(lambda: defaultdict(list))
    for fname in os.listdir(out_dir):
        if not fname.endswith("_cells.csv"):
            continue
        path = os.path.join(out_dir, fname)
        with open(path, newline="") as f:
            for row in csv.DictReader(f):
                t   = round(float(row["time_s"]))
                ci  = int(row["cell_idx"])
                snr = float(row["snr_dB"])
                data[t][ci].append(snr)
    if not data:
        sys.exit(f"ERROR: no *_cells.csv files found in {out_dir}")
    return data

# ---------------------------------------------------------------------------
# Gap computation helpers
# ---------------------------------------------------------------------------

def compute_greedy_gaps(data, all_times, all_cells, snr_thresh):
    """Times where ALL cells have max per-satellite SNR < snr_thresh."""
    low_snr = []
    for t in sorted(all_times):
        any_good = any(
            ci in data[t] and max(data[t][ci]) >= snr_thresh
            for ci in all_cells
        )
        if not any_good:
            low_snr.append(t)
    return low_snr


def compute_mrc_gaps(data, all_times, all_cells, snr_thresh):
    """Times where ALL cells have MRC-combined SNR < snr_thresh."""
    low_snr = []
    for t in sorted(all_times):
        all_below = all(
            ci not in data[t] or mrc_combine_snr_db(data[t][ci]) < snr_thresh
            for ci in all_cells
        )
        if all_below:
            low_snr.append(t)
    return low_snr


def ticks_to_segments(ticks):
    """Group consecutive ticks into (start, end, duration) triples."""
    if not ticks:
        return []
    segs = []
    s = p = ticks[0]
    for t in ticks[1:]:
        if t == p + 1:
            p = t
        else:
            segs.append((s, p, p - s + 1))
            s = p = t
    segs.append((s, p, p - s + 1))
    return segs

# ---------------------------------------------------------------------------
# Figure 1: Gap timeline — Greedy vs MRC
# ---------------------------------------------------------------------------

def fig_gap_timeline(greedy_gaps, mrc_gaps, t_min, t_max, snr_thresh, fig_dir):
    """
    Horizontal timeline showing service coverage vs gap periods for
    Greedy (greedy-max selection) and MRC combining.

    Green = service, Red = gap segment
    """
    greedy_segs = ticks_to_segments(greedy_gaps)
    mrc_segs    = ticks_to_segments(mrc_gaps)
    window = t_max - t_min + 1

    fig, axes = plt.subplots(2, 1, figsize=(14, 3.5), sharex=True)
    labels = ["Greedy (best sat)", "MRC combined"]
    all_segs = [greedy_segs, mrc_segs]

    for ax, label, segs in zip(axes, labels, all_segs):
        # Draw full window as green (service)
        ax.barh(0, window, left=t_min, height=0.6,
                color="#4caf50", linewidth=0)

        # Overlay gap segments in red
        for gs, ge, dur in segs:
            ax.barh(0, dur, left=gs, height=0.6,
                    color="#f44336", linewidth=0)
            # Annotate duration for segments >= 10 s
            if dur >= 10:
                ax.text(gs + dur / 2, 0,
                        f"{dur} s", ha="center", va="center",
                        fontsize=8, color="white", fontweight="bold")

        total_gap = sum(d for _, _, d in segs)
        pct       = total_gap / window * 100
        ax.set_ylabel(label, fontsize=9)
        ax.set_yticks([])
        ax.set_xlim(t_min, t_max)
        ax.text(0.99, 0.88,
                f"Gap: {total_gap} s ({pct:.1f}%)",
                transform=ax.transAxes, ha="right", va="top",
                fontsize=9, color="#f44336" if total_gap > 0 else "#4caf50")

    axes[1].set_xlabel("Time (s)", fontsize=10)
    fig.suptitle(
        f"Service Coverage Timeline — Tokyo ROI (3600 s window)\n"
        f"Threshold = {snr_thresh} dB  |  "
        f"Green = service,  Red = SNR < {snr_thresh} dB gap",
        fontsize=10,
    )

    # Legend
    green_patch = mpatches.Patch(color="#4caf50", label="Service (SNR ≥ threshold)")
    red_patch   = mpatches.Patch(color="#f44336", label="Gap (SNR < threshold)")
    fig.legend(handles=[green_patch, red_patch],
               loc="upper right", fontsize=8, framealpha=0.9)

    plt.tight_layout(rect=[0, 0, 0.88, 1])
    path = os.path.join(fig_dir, "fig_gap_timeline.png")
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"[fig] {path}")

# ---------------------------------------------------------------------------
# Figure 2: Link budget — SNR vs elevation
# ---------------------------------------------------------------------------

def fig_link_budget(snr_thresh, min_elev_deg, fig_dir):
    """
    Analytical SNR vs elevation angle curve with:
      - Service threshold line (horizontal dashed)
      - Geometric cutoff (minElevDeg, vertical dotted)
      - Critical service elevation (vertical dashed)
      - Soft outage zone shaded
    """
    crit = critical_elevation(snr_thresh_db=snr_thresh)

    # Dense curve
    elevs  = np.linspace(1.0, 90.0, 360)
    snrs   = [snr_at_elevation(float(e)) for e in elevs]

    fig, ax = plt.subplots(figsize=(9, 5))

    # Main SNR curve
    ax.plot(elevs, snrs, color="#1565c0", linewidth=2.0, label="Analytical SNR")

    # Service threshold (horizontal)
    ax.axhline(snr_thresh, color="#888", linestyle="--", linewidth=1.0,
               label=f"Threshold = {snr_thresh} dB")

    # Geometric cutoff (vertical dotted)
    ax.axvline(min_elev_deg, color="#e65100", linestyle=":",
               linewidth=1.5, label=f"minElevDeg = {min_elev_deg}°")

    # Critical service elevation (vertical dashed)
    ax.axvline(crit, color="#c62828", linestyle="--",
               linewidth=1.5, label=f"Critical elev = {crit}°")

    # Soft outage zone: min_elev_deg ~ crit
    if crit > min_elev_deg:
        ax.axvspan(min_elev_deg, crit, alpha=0.12, color="#f44336",
                   label=f"Soft outage zone ({min_elev_deg}°–{crit}°)\n"
                         f"Visible but SNR < threshold")

    # SNR annotation at key angles
    for e_deg in [5.0, 7.3, 10.0, 20.0, 45.0, 90.0]:
        snr_val = snr_at_elevation(e_deg)
        ax.annotate(
            f"{snr_val:.1f} dB",
            xy=(e_deg, snr_val),
            xytext=(e_deg + 2, snr_val + 0.5),
            fontsize=7.5, color="#1565c0",
            arrowprops=dict(arrowstyle="-", color="#1565c0", lw=0.7),
        )

    ax.set_xlabel("Satellite Elevation Angle (°)", fontsize=11)
    ax.set_ylabel("SNR (dB)", fontsize=11)
    ax.set_xlim(0, 90)
    ax.set_title(
        "Link Budget: SNR vs Elevation — Iridium-NEXT LEO (600 km, Ka-band)\n"
        "SimConfig defaults: G_ant = 60.5 dBi, P_tx = 63 W, BW = 25 MHz",
        fontsize=10,
    )
    ax.legend(fontsize=8.5, loc="lower right")
    ax.grid(True, alpha=0.3)
    plt.tight_layout()

    path = os.path.join(fig_dir, "fig_link_budget.png")
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"[fig] {path}")

# ---------------------------------------------------------------------------
# Figure 3: MRC vs Greedy — gap duration bar comparison
# ---------------------------------------------------------------------------

def fig_mrc_vs_greedy(greedy_gaps, mrc_gaps, window_s, snr_thresh, fig_dir):
    """
    Side-by-side bar chart: total gap duration for Greedy vs MRC.
    Annotated with absolute seconds and percentage.
    """
    greedy_total = len(greedy_gaps)
    mrc_total    = len(mrc_gaps)
    greedy_pct   = greedy_total / window_s * 100
    mrc_pct      = mrc_total    / window_s * 100

    labels  = ["Greedy\n(best-sat selection)", "MRC\n(multi-sat combining)"]
    values  = [greedy_total, mrc_total]
    colors  = ["#ef9a9a", "#a5d6a7"]   # light red, light green

    fig, ax = plt.subplots(figsize=(7, 5))
    bars = ax.bar(labels, values, color=colors, edgecolor="white",
                  linewidth=1.5, width=0.45)

    # Annotate each bar
    for bar, val, pct in zip(bars, values, [greedy_pct, mrc_pct]):
        y_pos = bar.get_height() + 3
        label = f"{val} s\n({pct:.1f}%)"
        ax.text(bar.get_x() + bar.get_width() / 2, y_pos,
                label, ha="center", va="bottom", fontsize=12, fontweight="bold",
                color="#333")

    # Improvement arrow annotation
    if greedy_total > 0 and mrc_total == 0:
        ax.annotate(
            "100% reduction\n(0 gaps remain)",
            xy=(1, 5), xytext=(0.75, greedy_total * 0.55),
            fontsize=10, color="#1b5e20", fontweight="bold",
            arrowprops=dict(arrowstyle="->", color="#1b5e20", lw=1.5),
        )
    elif greedy_total > mrc_total:
        improvement = (greedy_total - mrc_total) / greedy_total * 100
        ax.annotate(
            f"−{greedy_total - mrc_total} s\n({improvement:.0f}% reduction)",
            xy=(1, mrc_total + 5), xytext=(0.7, (greedy_total + mrc_total) / 2),
            fontsize=10, color="#1b5e20", fontweight="bold",
            arrowprops=dict(arrowstyle="->", color="#1b5e20", lw=1.5),
        )

    ax.set_ylabel("Total Gap Duration (s)", fontsize=11)
    ax.set_ylim(0, greedy_total * 1.3 + 10)
    ax.set_title(
        f"Service Gap: Greedy-Max vs MRC Combining\n"
        f"Threshold = {snr_thresh} dB  |  Window = {window_s} s",
        fontsize=11,
    )
    ax.yaxis.grid(True, alpha=0.3)
    ax.set_axisbelow(True)
    plt.tight_layout()

    path = os.path.join(fig_dir, "fig_mrc_vs_greedy.png")
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"[fig] {path}")

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    args    = parse_args()
    out_dir = os.path.abspath(args.out_dir)
    # Figures saved alongside the data (out_dir/figures/) for easy navigation
    fig_dir = os.path.join(out_dir, "figures")
    os.makedirs(fig_dir, exist_ok=True)

    print(f"[info] loading CSVs from {out_dir} ...")
    data      = load_data(out_dir)
    all_times = sorted(data.keys())
    all_cells = sorted({ci for td in data.values() for ci in td})
    t_min, t_max = all_times[0], all_times[-1]
    window_s  = t_max - t_min + 1

    print(f"[info] time range: {t_min}–{t_max} s  |  "
          f"cells: {len(all_cells)}  |  ticks: {len(all_times)}")

    # Compute gap tick lists
    print("[info] computing greedy gaps ...")
    greedy_gaps = compute_greedy_gaps(data, all_times, all_cells, args.snr_thresh)
    print("[info] computing MRC gaps ...")
    mrc_gaps    = compute_mrc_gaps(data, all_times, all_cells, args.snr_thresh)

    print(f"[info] greedy gap: {len(greedy_gaps)} s  "
          f"| MRC gap: {len(mrc_gaps)} s")

    # Generate figures
    print("\n[info] generating figures ...")
    fig_gap_timeline(greedy_gaps, mrc_gaps, t_min, t_max,
                     args.snr_thresh, fig_dir)
    fig_link_budget(args.snr_thresh, args.min_elev, fig_dir)
    fig_mrc_vs_greedy(greedy_gaps, mrc_gaps, window_s,
                      args.snr_thresh, fig_dir)

    print("\nDone. Figures saved to:", fig_dir)


if __name__ == "__main__":
    main()
