"""
exp_phase2_4_snr_heatmap_cdf.py  —  Phase 2.4 Figure A + Figure B
===================================================================

Figure A: 5×5 per-cell mean SNR heatmaps for three assignment policies:
  - sat[i]   (iridium-75 45, peak elev 66.4° at t=50s)
  - sat[i+1] (iridium-75 44, peak elev 85.2° at t=580s)
  - Greedy   (per-cell best satellite at each timestep)

  All three share the same colour axis so intensity is directly comparable.
  Each cell is annotated with its numeric SNR value.

Figure B: CDF of per-cell mean SNR across all in-footprint cells.
  One CDF curve per policy, with a vertical dashed line at the
  per-policy mean.

Input:
  --summary   dual_cell_summary.csv   (from Phase 2.2 C++ simulation)
  --overlap   dual_overlap.json       (from Phase 2.3 inline detection)
  --out-dir   output directory for PNG + SVG

Usage:
  python exp_phase2_4_snr_heatmap_cdf.py \\
      --summary 2D/results/dual_d5/dual_cell_summary.csv \\
      --overlap 2D/results/dual_d5/dual_overlap.json \\
      --out-dir 2D/results/dual_d5/figures
"""

import argparse
import json
import os

import matplotlib
matplotlib.use("Agg")  # non-interactive backend for VMware / headless env
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np
import pandas as pd

# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Phase 2.4: SNR heatmap + CDF")
    p.add_argument(
        "--summary",
        default="2D/results/dual_d5/dual_cell_summary.csv",
        help="Path to dual_cell_summary.csv",
    )
    p.add_argument(
        "--overlap",
        default="2D/results/dual_d5/dual_overlap.json",
        help="Path to dual_overlap.json (Phase 2.3 results)",
    )
    p.add_argument(
        "--out-dir",
        default="2D/results/dual_d5/figures",
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
    """Load dual_cell_summary.csv and validate required columns."""
    df = pd.read_csv(path)
    required = {
        "row", "col", "in_footprint",
        "mean_snr_i_dB", "mean_snr_i1_dB", "greedy_mean_snr_dB",
        "coverage_i_s", "coverage_i1_s",
    }
    missing = required - set(df.columns)
    if missing:
        raise ValueError(f"dual_cell_summary.csv missing columns: {missing}")
    return df


def load_overlap(path: str) -> dict:
    """Load dual_overlap.json; return empty dict if file absent."""
    if not os.path.isfile(path):
        print(f"  [warn] {path} not found — Phase 2.3 annotations skipped")
        return {}
    with open(path, "r") as f:
        return json.load(f)


# ---------------------------------------------------------------------------
# Figure A — 5×5 SNR heatmaps (3 subplots)
# ---------------------------------------------------------------------------

def build_snr_grids(df: pd.DataFrame, d: int):
    """
    Reshape the flat DataFrame into three d×d numpy arrays.

    Row index 0 is the top row of the grid (max cy_m value) so that
    the spatial orientation matches a map view (North up, South down).
    The CSV rows are sorted ascending by (row, col), meaning row=0 is
    the southernmost row (most negative cy_m).  We flip vertically so
    the plot shows row=0 at the bottom (South) and row=d-1 at the top.
    """
    fp = df[df["in_footprint"] == 1].copy()
    fp = fp.sort_values(["row", "col"]).reset_index(drop=True)

    grid_i    = np.full((d, d), np.nan)
    grid_i1   = np.full((d, d), np.nan)
    grid_gr   = np.full((d, d), np.nan)

    for _, r in fp.iterrows():
        ri, ci = int(r["row"]), int(r["col"])
        grid_i[ri, ci]  = r["mean_snr_i_dB"]
        grid_i1[ri, ci] = r["mean_snr_i1_dB"]
        grid_gr[ri, ci] = r["greedy_mean_snr_dB"]

    # Flip so row=0 is at the bottom of the imshow plot
    return (
        np.flipud(grid_i),
        np.flipud(grid_i1),
        np.flipud(grid_gr),
    )


def plot_figure_a(
    df: pd.DataFrame,
    overlap: dict,
    d: int,
    out_dir: str,
    dpi: int,
) -> None:
    """
    Figure A: three side-by-side 5×5 SNR heatmaps.

    Design:
    - Shared colour axis: [vmin, vmax] = [floor(global_min), ceil(global_max)]
    - Cell annotations: SNR value printed in the cell centre
    - X/Y tick labels: cell centre position in km
    - Colourbar appended to the right of the last subplot
    """
    grid_i, grid_i1, grid_gr = build_snr_grids(df, d)

    # Shared colour range
    all_vals = np.concatenate([
        grid_i[~np.isnan(grid_i)],
        grid_i1[~np.isnan(grid_i1)],
        grid_gr[~np.isnan(grid_gr)],
    ])
    vmin = np.floor(all_vals.min())
    vmax = np.ceil(all_vals.max())

    # Axis tick labels (km) — from the CSV cx_m / cy_m columns
    fp = df[df["in_footprint"] == 1].copy()
    x_km = sorted(fp["cx_m"].unique())
    x_km = [v / 1e3 for v in x_km]                   # ascending East
    y_km = sorted(fp["cy_m"].unique(), reverse=True)  # descending (top = North)
    y_km = [v / 1e3 for v in y_km]

    policies = [
        (grid_i,   "sat[i]\n(iridium-75 45, peak 66.4°, t=50s)"),
        (grid_i1,  "sat[i+1]\n(iridium-75 44, peak 85.2°, t=580s)"),
        (grid_gr,  "Greedy\n(per-cell best satellite)"),
    ]

    fig, axes = plt.subplots(1, 3, figsize=(14, 5), constrained_layout=True)
    fig.suptitle(
        "Figure A — Per-Cell Time-Averaged Mean SNR (dB)\n"
        "d=5 ROI Grid, Tokyo ROI (lat=35.676°N, lon=139.650°E)",
        fontsize=11,
        fontweight="bold",
    )

    cmap = "RdYlGn"  # green = high SNR, red = low SNR

    im = None
    for ax, (grid, title) in zip(axes, policies):
        im = ax.imshow(
            grid,
            cmap=cmap,
            vmin=vmin,
            vmax=vmax,
            aspect="equal",
            interpolation="nearest",
        )
        ax.set_title(title, fontsize=9, pad=6)

        # X/Y tick labels in km
        ax.set_xticks(range(d))
        ax.set_xticklabels([f"{v:.0f}" for v in x_km], fontsize=7)
        ax.set_yticks(range(d))
        ax.set_yticklabels([f"{v:.0f}" for v in y_km], fontsize=7)
        ax.set_xlabel("East offset (km)", fontsize=8)
        ax.set_ylabel("North offset (km)", fontsize=8)

        # Annotate each cell with its SNR value
        for ri in range(d):
            for ci in range(d):
                val = grid[ri, ci]
                if not np.isnan(val):
                    # Choose text colour based on brightness
                    norm_val = (val - vmin) / (vmax - vmin)
                    txt_color = "black" if 0.3 < norm_val < 0.85 else "white"
                    ax.text(
                        ci, ri,
                        f"{val:.2f}",
                        ha="center", va="center",
                        fontsize=6.5, color=txt_color,
                        fontweight="bold",
                    )

    # Shared colourbar
    cbar = fig.colorbar(im, ax=axes, shrink=0.8, pad=0.02)
    cbar.set_label("Mean SNR (dB)", fontsize=9)

    # Phase 2.3 annotation in figure text
    if overlap:
        times = overlap.get("sat_i1_coverage_times_s", {})
        ann = (
            f"Phase 2.3 overlap thresholds — sat[i+1] coverage:\n"
            f"  10%={times.get('10pct','-')}s  "
            f"25%={times.get('25pct','-')}s  "
            f"50%={times.get('50pct','-')}s  "
            f"75%={times.get('75pct','-')}s  "
            f"90%={times.get('90pct','-')}s"
        )
        fig.text(
            0.01, -0.04, ann,
            fontsize=7, color="#555555",
            ha="left", va="top",
        )

    _save(fig, out_dir, "fig_A_snr_heatmap", dpi)
    print(f"  Figure A saved → {out_dir}/fig_A_snr_heatmap.png + .svg")


# ---------------------------------------------------------------------------
# Figure B — CDF of per-cell mean SNR
# ---------------------------------------------------------------------------

def plot_figure_b(
    df: pd.DataFrame,
    overlap: dict,
    out_dir: str,
    dpi: int,
) -> None:
    """
    Figure B: empirical CDF of per-cell mean SNR for three policies.

    Design:
    - One step-CDF curve per policy
    - Vertical dashed line at the policy mean
    - Legend shows policy name + mean SNR
    - X-axis = Mean SNR (dB), Y-axis = CDF (0–1)
    """
    fp = df[df["in_footprint"] == 1].copy()

    policies = [
        ("sat[i] (iridium-75 45)",    fp["mean_snr_i_dB"].values,    "#1f77b4"),
        ("sat[i+1] (iridium-75 44)",  fp["mean_snr_i1_dB"].values,   "#ff7f0e"),
        ("Greedy",                     fp["greedy_mean_snr_dB"].values, "#2ca02c"),
    ]

    fig, ax = plt.subplots(figsize=(7, 5), constrained_layout=True)

    x_all = []
    for label, vals, color in policies:
        sorted_v = np.sort(vals)
        n = len(sorted_v)
        cdf_y = np.arange(1, n + 1) / n

        # Step CDF: extend to both ends for a clean plot
        x_plot = np.concatenate([[sorted_v[0]], sorted_v])
        y_plot = np.concatenate([[0.0], cdf_y])

        mean_v = np.mean(vals)
        ax.step(
            x_plot, y_plot,
            where="post",
            color=color,
            linewidth=2,
            label=f"{label}  (mean={mean_v:.2f} dB)",
        )
        ax.axvline(
            mean_v,
            color=color,
            linestyle="--",
            linewidth=1.2,
            alpha=0.7,
        )
        x_all.extend(sorted_v.tolist())

    ax.set_xlabel("Per-Cell Mean SNR (dB)", fontsize=11)
    ax.set_ylabel("CDF", fontsize=11)
    ax.set_title(
        "Figure B — CDF of Per-Cell Mean SNR\n"
        "d=5 ROI Grid, 25 cells, Tokyo ROI",
        fontsize=11,
        fontweight="bold",
    )
    ax.set_ylim(0.0, 1.05)

    x_margin = 0.5
    ax.set_xlim(min(x_all) - x_margin, max(x_all) + x_margin)

    ax.yaxis.set_major_formatter(ticker.FormatStrFormatter("%.1f"))
    ax.grid(True, linestyle=":", alpha=0.5)
    ax.legend(fontsize=9, loc="lower right")

    # Annotate coverage times from Phase 2.3
    if overlap:
        times = overlap.get("sat_i1_coverage_times_s", {})
        ann = (
            "Phase 2.3 (sat[i+1] ROI coverage thresholds):\n"
            f"10%={times.get('10pct','-')}s  "
            f"25%={times.get('25pct','-')}s  "
            f"50%={times.get('50pct','-')}s\n"
            f"75%={times.get('75pct','-')}s  "
            f"90%={times.get('90pct','-')}s"
        )
        ax.text(
            0.02, 0.38, ann,
            transform=ax.transAxes,
            fontsize=7.5, color="#555555",
            va="top",
            bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="#cccccc", alpha=0.8),
        )

    _save(fig, out_dir, "fig_B_snr_cdf", dpi)
    print(f"  Figure B saved → {out_dir}/fig_B_snr_cdf.png + .svg")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _save(fig: plt.Figure, out_dir: str, stem: str, dpi: int) -> None:
    """Save figure as PNG (300 dpi) and SVG."""
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
    print("Phase 2.4 — SNR Heatmap + CDF")
    print(f"{'='*60}")
    print(f"  summary : {args.summary}")
    print(f"  overlap : {args.overlap}")
    print(f"  out-dir : {args.out_dir}")

    df      = load_summary(args.summary)
    overlap = load_overlap(args.overlap)

    # Infer d from data
    d = int(df["row"].max()) + 1
    n_fp = int((df["in_footprint"] == 1).sum())
    print(f"  d={d}  in-footprint cells={n_fp}")

    # Print per-policy mean SNR summary
    fp = df[df["in_footprint"] == 1]
    print(f"\n  Policy mean SNR across {n_fp} cells:")
    print(f"    sat[i]   : {fp['mean_snr_i_dB'].mean():.3f} dB")
    print(f"    sat[i+1] : {fp['mean_snr_i1_dB'].mean():.3f} dB")
    print(f"    Greedy   : {fp['greedy_mean_snr_dB'].mean():.3f} dB")

    if overlap:
        thresh = overlap.get("sat_i1_coverage_times_s", {})
        print(f"\n  Phase 2.3 overlap thresholds (sat[i+1] >= 0 dB):")
        for k, v in thresh.items():
            print(f"    {k:6s} = {v:.1f}s")

    print("\nGenerating figures...")
    plot_figure_a(df, overlap, d, args.out_dir, args.dpi)
    plot_figure_b(df, overlap, args.out_dir, args.dpi)

    print(f"\nDone.  Figures in: {args.out_dir}\n")


if __name__ == "__main__":
    main()
