"""
exp_phase2_plots.py  —  Phase 2 unified figure generator
=========================================================

Generates all Phase 2 figures in a single pass:

  Figure A  5×5 per-cell mean SNR heatmaps — sat[i] / sat[i+1] / Greedy
  Figure B  CDF of per-cell mean SNR across the three policies
  Figure C  sat[i+1] ROI coverage buildup curve (Phase 2.3)
  Figure D  Grid mode single-satellite SNR heatmap + min/max range

Inputs:
  --grid-summary   grid/cell_summary.csv     (Phase 2.0 single-sat output)
  --dual-summary   dual/cell_summary.csv     (Phase 2.2 dual-sat output)
  --dual-result    dual/cell_result.csv      (Phase 2.3 raw timestep data)
  --overlap        dual/overlap.json         (Phase 2.3 threshold crossings)
  --out-dir        output directory
  --figures        subset to generate, e.g. --figures A C  (default: all)
  --snr-thresh     SNR threshold for Figure C coverage count (default: 0 dB)
  --dpi            PNG output DPI (default: 300)

Usage (all figures):
  python exp_phase2_plots.py \\
      --grid-summary 2D/phase2/result/grid/cell_summary.csv \\
      --dual-summary 2D/phase2/result/dual/cell_summary.csv \\
      --dual-result  2D/phase2/result/dual/cell_result.csv \\
      --overlap      2D/phase2/result/dual/overlap.json \\
      --out-dir      2D/phase2/figures

Usage (selected figures only):
  python exp_phase2_plots.py \\
      --dual-summary 2D/phase2/result/dual/cell_summary.csv \\
      --overlap      2D/phase2/result/dual/overlap.json \\
      --out-dir      2D/phase2/figures \\
      --figures A B
"""

import argparse
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np
import pandas as pd


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Phase 2 unified figure generator (Figure A–D)"
    )
    p.add_argument("--grid-summary",
                   default="2D/phase2/result/grid/cell_summary.csv",
                   help="grid cell_summary.csv (Phase 2.0)")
    p.add_argument("--dual-summary",
                   default="2D/phase2/result/dual/cell_summary.csv",
                   help="dual cell_summary.csv (Phase 2.2)")
    p.add_argument("--dual-result",
                   default="2D/phase2/result/dual/cell_result.csv",
                   help="dual cell_result.csv (Phase 2.3 raw data)")
    p.add_argument("--overlap",
                   default="2D/phase2/result/dual/overlap.json",
                   help="dual overlap.json (Phase 2.3 threshold crossings)")
    p.add_argument("--out-dir",
                   default="2D/phase2/figures",
                   help="Output directory for PNG and SVG figures")
    p.add_argument("--figures",
                   nargs="+",
                   choices=["A", "B", "C", "D"],
                   default=["A", "B", "C", "D"],
                   help="Which figures to generate (default: all)")
    p.add_argument("--snr-thresh",
                   type=float,
                   default=0.0,
                   help="SNR threshold (dB) for Figure C coverage count")
    p.add_argument("--dpi",
                   type=int,
                   default=300,
                   help="PNG output DPI (default: 300)")
    return p.parse_args()


# ---------------------------------------------------------------------------
# Data loaders
# ---------------------------------------------------------------------------

def _load_csv(path: str, required: set, label: str) -> pd.DataFrame | None:
    if not os.path.isfile(path):
        print(f"  [skip] {label}: file not found — {path}")
        return None
    df = pd.read_csv(path)
    missing = required - set(df.columns)
    if missing:
        print(f"  [skip] {label}: missing columns {missing}")
        return None
    return df


def load_grid_summary(path: str) -> pd.DataFrame | None:
    return _load_csv(
        path,
        {"row", "col", "cx_m", "cy_m", "in_footprint",
         "coverage_s", "mean_snr_dB", "min_snr_dB", "max_snr_dB"},
        "grid cell_summary",
    )


def load_dual_summary(path: str) -> pd.DataFrame | None:
    return _load_csv(
        path,
        {"row", "col", "cx_m", "cy_m", "in_footprint",
         "mean_snr_i_dB", "mean_snr_i1_dB", "greedy_mean_snr_dB",
         "coverage_i_s", "coverage_i1_s"},
        "dual cell_summary",
    )


def load_dual_result(path: str) -> pd.DataFrame | None:
    return _load_csv(
        path,
        {"time_s", "row", "col", "snr_i1_dB"},
        "dual cell_result",
    )


def load_overlap(path: str) -> dict:
    if not os.path.isfile(path):
        print(f"  [warn] overlap.json not found — threshold annotations skipped")
        return {}
    with open(path) as f:
        return json.load(f)


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------

def _save(fig: plt.Figure, out_dir: str, stem: str, dpi: int) -> None:
    os.makedirs(out_dir, exist_ok=True)
    fig.savefig(os.path.join(out_dir, f"{stem}.png"), dpi=dpi, bbox_inches="tight")
    fig.savefig(os.path.join(out_dir, f"{stem}.svg"), bbox_inches="tight")
    plt.close(fig)


def _build_dual_grids(df: pd.DataFrame, d: int):
    """Reshape dual summary into three d×d numpy arrays (flipped for map view)."""
    fp = df[df["in_footprint"] == 1].sort_values(["row", "col"])
    grid_i  = np.full((d, d), np.nan)
    grid_i1 = np.full((d, d), np.nan)
    grid_gr = np.full((d, d), np.nan)
    for _, r in fp.iterrows():
        ri, ci = int(r["row"]), int(r["col"])
        grid_i[ri, ci]  = r["mean_snr_i_dB"]
        grid_i1[ri, ci] = r["mean_snr_i1_dB"]
        grid_gr[ri, ci] = r["greedy_mean_snr_dB"]
    return np.flipud(grid_i), np.flipud(grid_i1), np.flipud(grid_gr)


def _km_ticks(fp: pd.DataFrame, d: int):
    """Return x_km (ascending East) and y_km (descending North) tick labels."""
    x_km = [v / 1e3 for v in sorted(fp["cx_m"].unique())]
    y_km = [v / 1e3 for v in sorted(fp["cy_m"].unique(), reverse=True)]
    return x_km, y_km


def _annotate_heatmap(ax, grid, d, vmin, vmax):
    for ri in range(d):
        for ci in range(d):
            val = grid[ri, ci]
            if not np.isnan(val):
                norm_val = (val - vmin) / (vmax - vmin)
                color = "black" if 0.3 < norm_val < 0.85 else "white"
                ax.text(ci, ri, f"{val:.2f}",
                        ha="center", va="center",
                        fontsize=6.5, color=color, fontweight="bold")


# ---------------------------------------------------------------------------
# Figure A — dual SNR heatmaps (sat[i] / sat[i+1] / Greedy)
# ---------------------------------------------------------------------------

def plot_figure_a(df: pd.DataFrame, overlap: dict,
                  out_dir: str, dpi: int) -> None:
    d   = int(df["row"].max()) + 1
    fp  = df[df["in_footprint"] == 1]
    x_km, y_km = _km_ticks(fp, d)

    grid_i, grid_i1, grid_gr = _build_dual_grids(df, d)
    all_vals = np.concatenate([
        grid_i[~np.isnan(grid_i)],
        grid_i1[~np.isnan(grid_i1)],
        grid_gr[~np.isnan(grid_gr)],
    ])
    vmin, vmax = np.floor(all_vals.min()), np.ceil(all_vals.max())

    policies = [
        (grid_i,  "sat[i]\n(peak 66.4°, t≈50s)"),
        (grid_i1, "sat[i+1]\n(peak 85.2°, t≈580s)"),
        (grid_gr, "Greedy\n(per-cell best)"),
    ]

    fig, axes = plt.subplots(1, 3, figsize=(14, 5), constrained_layout=True)
    fig.suptitle(
        "Figure A — Per-Cell Time-Averaged Mean SNR (dB)\n"
        "d=5 ROI Grid, Tokyo ROI (35.676°N, 139.650°E)",
        fontsize=11, fontweight="bold",
    )

    im = None
    for ax, (grid, title) in zip(axes, policies):
        im = ax.imshow(grid, cmap="RdYlGn", vmin=vmin, vmax=vmax,
                       aspect="equal", interpolation="nearest")
        ax.set_title(title, fontsize=9, pad=6)
        ax.set_xticks(range(d)); ax.set_xticklabels([f"{v:.0f}" for v in x_km], fontsize=7)
        ax.set_yticks(range(d)); ax.set_yticklabels([f"{v:.0f}" for v in y_km], fontsize=7)
        ax.set_xlabel("East offset (km)", fontsize=8)
        ax.set_ylabel("North offset (km)", fontsize=8)
        _annotate_heatmap(ax, grid, d, vmin, vmax)

    cbar = fig.colorbar(im, ax=axes, shrink=0.8, pad=0.02)
    cbar.set_label("Mean SNR (dB)", fontsize=9)

    if overlap:
        times = overlap.get("sat_i1_coverage_times_s", {})
        ann = (f"Phase 2.3 overlap — sat[i+1] coverage: "
               f"10%={times.get('10pct','-')}s  25%={times.get('25pct','-')}s  "
               f"50%={times.get('50pct','-')}s  75%={times.get('75pct','-')}s  "
               f"90%={times.get('90pct','-')}s")
        fig.text(0.01, -0.04, ann, fontsize=7, color="#555555", ha="left", va="top")

    _save(fig, out_dir, "fig_A_snr_heatmap", dpi)
    print(f"  [A] fig_A_snr_heatmap.png + .svg")


# ---------------------------------------------------------------------------
# Figure B — CDF of per-cell mean SNR
# ---------------------------------------------------------------------------

def plot_figure_b(df: pd.DataFrame, overlap: dict,
                  out_dir: str, dpi: int) -> None:
    fp = df[df["in_footprint"] == 1].copy()

    policies = [
        ("sat[i]",   fp["mean_snr_i_dB"].values,    "#1f77b4"),
        ("sat[i+1]", fp["mean_snr_i1_dB"].values,   "#ff7f0e"),
        ("Greedy",   fp["greedy_mean_snr_dB"].values, "#2ca02c"),
    ]

    fig, ax = plt.subplots(figsize=(7, 5), constrained_layout=True)

    x_all = []
    for label, vals, color in policies:
        sv = np.sort(vals)
        n  = len(sv)
        x_plot = np.concatenate([[sv[0]], sv])
        y_plot = np.concatenate([[0.0], np.arange(1, n + 1) / n])
        mean_v = np.mean(vals)
        ax.step(x_plot, y_plot, where="post", color=color, linewidth=2,
                label=f"{label}  (mean={mean_v:.2f} dB)")
        ax.axvline(mean_v, color=color, linestyle="--", linewidth=1.2, alpha=0.7)
        x_all.extend(sv.tolist())

    ax.set_xlabel("Per-Cell Mean SNR (dB)", fontsize=11)
    ax.set_ylabel("CDF", fontsize=11)
    ax.set_title("Figure B — CDF of Per-Cell Mean SNR\nd=5 ROI Grid, 25 cells, Tokyo ROI",
                 fontsize=11, fontweight="bold")
    ax.set_ylim(0.0, 1.05)
    ax.set_xlim(min(x_all) - 0.5, max(x_all) + 0.5)
    ax.yaxis.set_major_formatter(ticker.FormatStrFormatter("%.1f"))
    ax.grid(True, linestyle=":", alpha=0.5)
    ax.legend(fontsize=9, loc="lower right")

    if overlap:
        times = overlap.get("sat_i1_coverage_times_s", {})
        ann = (f"Phase 2.3 sat[i+1] ROI coverage:\n"
               f"10%={times.get('10pct','-')}s  25%={times.get('25pct','-')}s  "
               f"50%={times.get('50pct','-')}s\n"
               f"75%={times.get('75pct','-')}s  90%={times.get('90pct','-')}s")
        ax.text(0.02, 0.38, ann, transform=ax.transAxes, fontsize=7.5,
                color="#555555", va="top",
                bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="#cccccc", alpha=0.8))

    _save(fig, out_dir, "fig_B_snr_cdf", dpi)
    print(f"  [B] fig_B_snr_cdf.png + .svg")


# ---------------------------------------------------------------------------
# Figure C — sat[i+1] ROI coverage buildup
# ---------------------------------------------------------------------------

def _compute_coverage_curve(df: pd.DataFrame, snr_thresh: float):
    n_cells_per_step = int(df.groupby("time_s")["col"].count().iloc[0])

    def count_covered(grp):
        vis = grp["snr_i1_dB"] > -900.0
        return (vis & (grp["snr_i1_dB"] >= snr_thresh)).sum()

    grouped = df.groupby("time_s")
    counts  = grouped.apply(count_covered)
    times_s = counts.index.to_numpy(dtype=float)
    fracs   = counts.to_numpy(dtype=float) / n_cells_per_step
    return times_s, fracs, n_cells_per_step


def plot_figure_c(df_result: pd.DataFrame, overlap: dict,
                  snr_thresh: float, out_dir: str, dpi: int) -> None:
    times_s, fracs, n_cells = _compute_coverage_curve(df_result, snr_thresh)

    threshold_styles = {
        "10pct": ("#1f77b4", "10%"),
        "25pct": ("#2ca02c", "25%"),
        "50pct": ("#d62728", "50%"),
        "75pct": ("#9467bd", "75%"),
        "90pct": ("#8c564b", "90%"),
    }

    fig, ax = plt.subplots(figsize=(9, 5), constrained_layout=True)

    ax.plot(times_s, fracs * 100, color="#ff7f0e", linewidth=1.8,
            label="sat[i+1] ROI coverage")
    ax.fill_between(times_s, fracs * 100, alpha=0.15, color="#ff7f0e")

    if overlap:
        times_dict = overlap.get("sat_i1_coverage_times_s", {})
        for key, (color, label) in threshold_styles.items():
            t = times_dict.get(key)
            if t is not None:
                ax.axvline(t, color=color, linestyle="--", linewidth=1.2, alpha=0.85,
                           label=f"{label} @ {t:.1f}s")
                pct_val = float(label.rstrip("%"))
                ax.text(t + 1.5, pct_val + 1.5, f"{t:.0f}s",
                        fontsize=7, color=color, va="bottom")

    vis_mask = fracs > 0.0
    if vis_mask.any():
        t0, t1 = times_s[vis_mask][0], times_s[vis_mask][-1]
        ax.axvspan(t0, t1, alpha=0.06, color="#ff7f0e",
                   label=f"sat[i+1] visible [{t0:.0f}s–{t1:.0f}s]")

    ax.set_xlabel("Simulation Time (s)", fontsize=11)
    ax.set_ylabel("ROI Cells Covered by sat[i+1] (%)", fontsize=11)
    ax.set_title(
        "Figure C — Phase 2.3: sat[i+1] ROI Coverage Buildup\n"
        f"d=5 grid, {n_cells} in-footprint cells, SNR threshold = {snr_thresh:.1f} dB",
        fontsize=11, fontweight="bold",
    )
    ax.set_ylim(-5, 110)
    ax.set_xlim(times_s[0], times_s[-1])
    ax.yaxis.set_major_formatter(ticker.PercentFormatter(xmax=100, decimals=0))
    ax.grid(True, linestyle=":", alpha=0.5)
    ax.legend(fontsize=8, loc="upper left", framealpha=0.9)

    _save(fig, out_dir, "fig_C_overlap_timeline", dpi)
    print(f"  [C] fig_C_overlap_timeline.png + .svg")


# ---------------------------------------------------------------------------
# Figure D — grid mode heatmap + min/max range
# ---------------------------------------------------------------------------

def plot_figure_d(df: pd.DataFrame, out_dir: str, dpi: int) -> None:
    fp = df[df["in_footprint"] == 1].sort_values(["row", "col"]).reset_index(drop=True)
    d  = int(fp["row"].max()) + 1

    grid_mean = np.full((d, d), np.nan)
    for _, r in fp.iterrows():
        grid_mean[int(r["row"]), int(r["col"])] = r["mean_snr_dB"]
    grid_plot = np.flipud(grid_mean)

    x_km, y_km = _km_ticks(fp, d)
    coverage_s = fp["coverage_s"].iloc[0]

    vmin = np.floor(np.nanmin(grid_plot))
    vmax = np.ceil(np.nanmax(grid_plot))

    fig, (ax_heat, ax_range) = plt.subplots(
        1, 2, figsize=(13, 5), constrained_layout=True,
        gridspec_kw={"width_ratios": [1.1, 1.6]},
    )
    fig.suptitle(
        "Figure D — Phase 2.0: Single-Satellite Grid Mode\n"
        f"d=5 ROI Grid, Tokyo ROI  |  coverage={coverage_s:.1f}s",
        fontsize=11, fontweight="bold",
    )

    im = ax_heat.imshow(grid_plot, cmap="RdYlGn", vmin=vmin, vmax=vmax,
                        aspect="equal", interpolation="nearest")
    ax_heat.set_title("Per-Cell Mean SNR (dB)", fontsize=10)
    ax_heat.set_xticks(range(d)); ax_heat.set_xticklabels([f"{v:.0f}" for v in x_km], fontsize=8)
    ax_heat.set_yticks(range(d)); ax_heat.set_yticklabels([f"{v:.0f}" for v in y_km], fontsize=8)
    ax_heat.set_xlabel("East offset (km)", fontsize=9)
    ax_heat.set_ylabel("North offset (km)", fontsize=9)
    _annotate_heatmap(ax_heat, grid_plot, d, vmin, vmax)
    fig.colorbar(im, ax=ax_heat, shrink=0.82, pad=0.02, label="Mean SNR (dB)")

    n = len(fp)
    cell_idx = np.arange(n)
    means = fp["mean_snr_dB"].to_numpy()
    mins  = fp["min_snr_dB"].to_numpy()
    maxs  = fp["max_snr_dB"].to_numpy()
    labels = [f"({int(r['row'])},{int(r['col'])})" for _, r in fp.iterrows()]

    ax_range.bar(cell_idx, maxs - mins, bottom=mins,
                 color="#aec6e8", edgecolor="#4878a4", linewidth=0.7,
                 label="Min–Max range")
    ax_range.plot(cell_idx, means, "o-", color="#d62728",
                  markersize=4, linewidth=1.4, label="Mean SNR")
    ax_range.set_xticks(cell_idx)
    ax_range.set_xticklabels(labels, rotation=60, ha="right", fontsize=6.5)
    ax_range.set_xlabel("Cell (row, col)", fontsize=9)
    ax_range.set_ylabel("SNR (dB)", fontsize=9)
    ax_range.set_title("Per-Cell SNR Min / Mean / Max", fontsize=10)
    ax_range.grid(True, axis="y", linestyle=":", alpha=0.5)
    ax_range.legend(fontsize=8, loc="lower center")

    _save(fig, out_dir, "fig_D_grid_snr_heatmap", dpi)
    print(f"  [D] fig_D_grid_snr_heatmap.png + .svg")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    args = parse_args()
    figs = set(args.figures)

    print(f"\n{'='*60}")
    print("Phase 2 — Unified Figure Generator")
    print(f"{'='*60}")
    print(f"  figures  : {', '.join(sorted(figs))}")
    print(f"  out-dir  : {args.out_dir}")

    # Load only what is needed
    dual_summary = load_dual_summary(args.dual_summary) if figs & {"A", "B"} else None
    dual_result  = load_dual_result(args.dual_result)   if "C" in figs else None
    grid_summary = load_grid_summary(args.grid_summary) if "D" in figs else None
    overlap      = load_overlap(args.overlap)            if figs & {"A", "B", "C"} else {}

    print()

    if "A" in figs:
        if dual_summary is not None:
            plot_figure_a(dual_summary, overlap, args.out_dir, args.dpi)
        else:
            print("  [A] skipped (dual_summary not available)")

    if "B" in figs:
        if dual_summary is not None:
            plot_figure_b(dual_summary, overlap, args.out_dir, args.dpi)
        else:
            print("  [B] skipped (dual_summary not available)")

    if "C" in figs:
        if dual_result is not None:
            plot_figure_c(dual_result, overlap, args.snr_thresh, args.out_dir, args.dpi)
        else:
            print("  [C] skipped (dual_result not available)")

    if "D" in figs:
        if grid_summary is not None:
            plot_figure_d(grid_summary, args.out_dir, args.dpi)
        else:
            print("  [D] skipped (grid_summary not available)")

    print(f"\nDone.  Output → {args.out_dir}\n")


if __name__ == "__main__":
    main()
