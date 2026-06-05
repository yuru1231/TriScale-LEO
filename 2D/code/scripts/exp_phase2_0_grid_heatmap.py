"""
exp_phase2_0_grid_heatmap.py  —  Phase 2.0 Figure D
=====================================================

Figure D: 5×5 per-cell mean SNR heatmap for the single-satellite grid mode.

Shows time-averaged mean SNR at each cell centre across the full visible
pass of sat[i].  Each cell is annotated with its numeric value.
A second subplot shows the min/max SNR range (error bar style) per cell
index to indicate SNR variability during the pass.

Input:
  --summary   grid cell_summary.csv  (from Phase 2.0 C++ simulation)
  --out-dir   output directory for PNG + SVG

Usage:
  python exp_phase2_0_grid_heatmap.py \\
      --summary 2D/phase2/result/grid/cell_summary.csv \\
      --out-dir 2D/phase2/figures
"""

import argparse
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Phase 2.0: grid mode SNR heatmap")
    p.add_argument(
        "--summary",
        default="2D/phase2/result/grid/cell_summary.csv",
        help="Path to grid cell_summary.csv",
    )
    p.add_argument(
        "--out-dir",
        default="2D/phase2/figures",
        help="Output directory for PNG and SVG figures",
    )
    p.add_argument(
        "--dpi",
        type=int,
        default=300,
        help="DPI for PNG output (default: 300)",
    )
    return p.parse_args()


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

def load_summary(path: str) -> pd.DataFrame:
    required = {"row", "col", "cx_m", "cy_m", "in_footprint",
                "coverage_s", "mean_snr_dB", "min_snr_dB", "max_snr_dB"}
    df = pd.read_csv(path)
    missing = required - set(df.columns)
    if missing:
        raise ValueError(f"cell_summary.csv missing columns: {missing}")
    return df


# ---------------------------------------------------------------------------
# Figure D — heatmap + range subplot
# ---------------------------------------------------------------------------

def plot_figure_d(df: pd.DataFrame, out_dir: str, dpi: int) -> None:
    """
    Figure D: two subplots.

    Left  — 5×5 heatmap of per-cell mean SNR (dB), annotated with values.
    Right — per-cell SNR range bar chart (min / mean / max) ordered by
            cell index (row-major), showing variability across the pass.
    """
    fp = df[df["in_footprint"] == 1].copy()
    fp = fp.sort_values(["row", "col"]).reset_index(drop=True)

    d = int(fp["row"].max()) + 1

    # Build d×d grids
    grid_mean = np.full((d, d), np.nan)
    grid_min  = np.full((d, d), np.nan)
    grid_max  = np.full((d, d), np.nan)
    for _, r in fp.iterrows():
        ri, ci = int(r["row"]), int(r["col"])
        grid_mean[ri, ci] = r["mean_snr_dB"]
        grid_min[ri, ci]  = r["min_snr_dB"]
        grid_max[ri, ci]  = r["max_snr_dB"]

    # Flip for map orientation (row=0 at bottom)
    grid_mean_plot = np.flipud(grid_mean)

    # Tick labels (km)
    x_km = sorted(fp["cx_m"].unique())
    x_km = [v / 1e3 for v in x_km]
    y_km = sorted(fp["cy_m"].unique(), reverse=True)
    y_km = [v / 1e3 for v in y_km]

    coverage_s = fp["coverage_s"].iloc[0]

    fig, (ax_heat, ax_range) = plt.subplots(
        1, 2, figsize=(13, 5), constrained_layout=True,
        gridspec_kw={"width_ratios": [1.1, 1.6]},
    )
    fig.suptitle(
        "Figure D — Phase 2.0: Single-Satellite Grid Mode\n"
        f"d=5 ROI Grid, Tokyo ROI (lat=35.676°N, lon=139.650°E)  |  coverage={coverage_s:.1f}s",
        fontsize=11, fontweight="bold",
    )

    # -----------------------------------------------------------------------
    # Left: heatmap
    # -----------------------------------------------------------------------
    vmin = np.floor(np.nanmin(grid_mean_plot))
    vmax = np.ceil(np.nanmax(grid_mean_plot))

    im = ax_heat.imshow(
        grid_mean_plot,
        cmap="RdYlGn",
        vmin=vmin, vmax=vmax,
        aspect="equal",
        interpolation="nearest",
    )
    ax_heat.set_title("Per-Cell Mean SNR (dB)", fontsize=10)
    ax_heat.set_xticks(range(d))
    ax_heat.set_xticklabels([f"{v:.0f}" for v in x_km], fontsize=8)
    ax_heat.set_yticks(range(d))
    ax_heat.set_yticklabels([f"{v:.0f}" for v in y_km], fontsize=8)
    ax_heat.set_xlabel("East offset (km)", fontsize=9)
    ax_heat.set_ylabel("North offset (km)", fontsize=9)

    for ri in range(d):
        for ci in range(d):
            val = grid_mean_plot[ri, ci]
            if not np.isnan(val):
                norm_val = (val - vmin) / (vmax - vmin)
                txt_color = "black" if 0.3 < norm_val < 0.85 else "white"
                ax_heat.text(
                    ci, ri, f"{val:.2f}",
                    ha="center", va="center",
                    fontsize=7, color=txt_color, fontweight="bold",
                )

    fig.colorbar(im, ax=ax_heat, shrink=0.82, pad=0.02, label="Mean SNR (dB)")

    # -----------------------------------------------------------------------
    # Right: min / mean / max range bar chart
    # -----------------------------------------------------------------------
    n = len(fp)
    cell_idx = np.arange(n)
    means = fp["mean_snr_dB"].to_numpy()
    mins  = fp["min_snr_dB"].to_numpy()
    maxs  = fp["max_snr_dB"].to_numpy()

    cell_labels = [f"({int(r['row'])},{int(r['col'])})" for _, r in fp.iterrows()]

    ax_range.bar(
        cell_idx, maxs - mins,
        bottom=mins,
        color="#aec6e8", edgecolor="#4878a4", linewidth=0.7,
        label="Min–Max range",
    )
    ax_range.plot(
        cell_idx, means,
        "o-", color="#d62728", markersize=4, linewidth=1.4,
        label="Mean SNR",
    )

    ax_range.set_xticks(cell_idx)
    ax_range.set_xticklabels(cell_labels, rotation=60, ha="right", fontsize=6.5)
    ax_range.set_xlabel("Cell (row, col)", fontsize=9)
    ax_range.set_ylabel("SNR (dB)", fontsize=9)
    ax_range.set_title("Per-Cell SNR Min / Mean / Max", fontsize=10)
    ax_range.grid(True, axis="y", linestyle=":", alpha=0.5)
    ax_range.legend(fontsize=8, loc="lower center")

    _save(fig, out_dir, "fig_D_grid_snr_heatmap", dpi)
    print(f"  Figure D saved → {out_dir}/fig_D_grid_snr_heatmap.png + .svg")


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------

def _save(fig: plt.Figure, out_dir: str, stem: str, dpi: int) -> None:
    os.makedirs(out_dir, exist_ok=True)
    fig.savefig(os.path.join(out_dir, f"{stem}.png"), dpi=dpi, bbox_inches="tight")
    fig.savefig(os.path.join(out_dir, f"{stem}.svg"), bbox_inches="tight")
    plt.close(fig)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    args = parse_args()

    print(f"\n{'='*60}")
    print("Phase 2.0 — Grid Mode SNR Heatmap")
    print(f"{'='*60}")
    print(f"  summary  : {args.summary}")
    print(f"  out-dir  : {args.out_dir}")

    df = load_summary(args.summary)
    fp = df[df["in_footprint"] == 1]
    d  = int(df["row"].max()) + 1

    print(f"  d={d}  in-footprint cells={len(fp)}")
    print(f"  coverage_s = {fp['coverage_s'].iloc[0]:.1f}s")
    print(f"  mean SNR across cells: {fp['mean_snr_dB'].mean():.3f} dB")
    print(f"  SNR range: [{fp['min_snr_dB'].min():.2f}, {fp['max_snr_dB'].max():.2f}] dB")

    print("\nGenerating Figure D...")
    plot_figure_d(df, args.out_dir, args.dpi)

    print(f"\nDone.  Figure in: {args.out_dir}\n")


if __name__ == "__main__":
    main()
