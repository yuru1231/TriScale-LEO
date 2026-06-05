"""
exp_phase2_3_overlap_timeline.py  —  Phase 2.3 Figure C
=========================================================

Figure C: sat[i+1] ROI coverage buildup curve.

X-axis: simulation time (s)
Y-axis: fraction of in-footprint ROI cells where sat[i+1] SNR >= snr_threshold_dB

Overlaid vertical dashed lines mark the 10/25/50/75/90% crossing times
read from dual_overlap.json (Phase 2.3 output).

Input:
  --result    dual cell_result.csv  (raw per-timestep dual simulation data)
  --overlap   dual overlap.json     (Phase 2.3 threshold crossing times)
  --out-dir   output directory for PNG + SVG

Usage:
  python exp_phase2_3_overlap_timeline.py \\
      --result  2D/phase2/result/dual/cell_result.csv \\
      --overlap 2D/phase2/result/dual/overlap.json \\
      --out-dir 2D/phase2/figures
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
    p = argparse.ArgumentParser(description="Phase 2.3: sat[i+1] ROI coverage buildup curve")
    p.add_argument(
        "--result",
        default="2D/phase2/result/dual/cell_result.csv",
        help="Path to dual cell_result.csv",
    )
    p.add_argument(
        "--overlap",
        default="2D/phase2/result/dual/overlap.json",
        help="Path to dual overlap.json (Phase 2.3 threshold crossing times)",
    )
    p.add_argument(
        "--out-dir",
        default="2D/phase2/figures",
        help="Output directory for PNG and SVG figures",
    )
    p.add_argument(
        "--snr-thresh",
        type=float,
        default=0.0,
        help="SNR threshold (dB) for counting a cell as covered (default: 0.0)",
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

def load_result(path: str) -> pd.DataFrame:
    """Load dual cell_result.csv; filter out sentinel rows (snr_i1_dB == -999)."""
    required = {"time_s", "row", "col", "snr_i1_dB", "elev_i1_deg"}
    df = pd.read_csv(path)
    missing = required - set(df.columns)
    if missing:
        raise ValueError(f"cell_result.csv missing columns: {missing}")
    return df


def load_overlap(path: str) -> dict:
    if not os.path.isfile(path):
        print(f"  [warn] {path} not found — threshold annotations skipped")
        return {}
    with open(path) as f:
        return json.load(f)


# ---------------------------------------------------------------------------
# Computation
# ---------------------------------------------------------------------------

def compute_coverage_curve(
    df: pd.DataFrame,
    snr_thresh: float,
) -> tuple[np.ndarray, np.ndarray, int]:
    """
    For each timestep, compute fraction of ROI cells where sat[i+1] SNR >= snr_thresh.

    Sentinel value -999 means sat[i+1] not visible — treated as not covered.
    Returns (times_s, coverage_fraction, n_in_footprint).
    """
    # Infer n_in_footprint from the number of unique cells present in the CSV
    # (all 25 cells appear at every timestep, so group-size gives n_cells_per_step)
    n_cells_per_step = df.groupby("time_s")["col"].count().iloc[0]

    # Count cells per timestep where sat[i+1] is visible AND SNR >= threshold
    def count_covered(group):
        visible = group["snr_i1_dB"] > -900.0  # exclude sentinel
        return (visible & (group["snr_i1_dB"] >= snr_thresh)).sum()

    grouped = df.groupby("time_s")
    covered_counts = grouped.apply(count_covered)
    times_s = covered_counts.index.to_numpy(dtype=float)
    fractions = covered_counts.to_numpy(dtype=float) / n_cells_per_step

    return times_s, fractions, int(n_cells_per_step)


# ---------------------------------------------------------------------------
# Figure C
# ---------------------------------------------------------------------------

def plot_figure_c(
    times_s: np.ndarray,
    fractions: np.ndarray,
    n_cells: int,
    overlap: dict,
    snr_thresh: float,
    out_dir: str,
    dpi: int,
) -> None:
    """
    Figure C: sat[i+1] ROI coverage buildup curve.

    The curve shows the fraction of in-footprint cells where sat[i+1] SNR
    exceeds snr_thresh at each simulation timestep.  Vertical dashed lines
    mark the 10/25/50/75/90% first-crossing times from overlap.json.
    """
    fig, ax = plt.subplots(figsize=(9, 5), constrained_layout=True)

    # Main coverage curve
    ax.plot(
        times_s, fractions * 100,
        color="#ff7f0e", linewidth=1.8, label="sat[i+1] ROI coverage",
    )
    ax.fill_between(times_s, fractions * 100, alpha=0.15, color="#ff7f0e")

    # Threshold crossing annotations from overlap.json
    threshold_colors = {
        "10pct":  ("#1f77b4", "10%"),
        "25pct":  ("#2ca02c", "25%"),
        "50pct":  ("#d62728", "50%"),
        "75pct":  ("#9467bd", "75%"),
        "90pct":  ("#8c564b", "90%"),
    }

    if overlap:
        times_dict = overlap.get("sat_i1_coverage_times_s", {})
        for key, (color, label) in threshold_colors.items():
            t = times_dict.get(key)
            if t is not None:
                ax.axvline(
                    t, color=color, linestyle="--", linewidth=1.2, alpha=0.85,
                    label=f"{label} threshold @ {t:.1f}s",
                )
                ax.text(
                    t + 1.5,
                    float(label.rstrip("%")) + 1.5,
                    f"{t:.0f}s",
                    fontsize=7, color=color, va="bottom",
                )

    # Sat[i+1] visibility window: first and last timestep where snr_i1 > -900
    vis_mask = fractions > 0.0
    if vis_mask.any():
        t_first = times_s[vis_mask][0]
        t_last  = times_s[vis_mask][-1]
        ax.axvspan(t_first, t_last, alpha=0.06, color="#ff7f0e",
                   label=f"sat[i+1] visible [{t_first:.0f}s – {t_last:.0f}s]")

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
    print(f"  Figure C saved → {out_dir}/fig_C_overlap_timeline.png + .svg")


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
    print("Phase 2.3 — Coverage Buildup Timeline")
    print(f"{'='*60}")
    print(f"  result   : {args.result}")
    print(f"  overlap  : {args.overlap}")
    print(f"  out-dir  : {args.out_dir}")
    print(f"  SNR thr  : {args.snr_thresh:.1f} dB")

    df      = load_result(args.result)
    overlap = load_overlap(args.overlap)

    print(f"  loaded {len(df):,} rows from cell_result.csv")

    times_s, fractions, n_cells = compute_coverage_curve(df, args.snr_thresh)

    print(f"  {len(times_s)} unique timesteps  |  {n_cells} cells per step")
    print(f"  max coverage reached : {fractions.max()*100:.1f}%")

    if overlap:
        thresh_times = overlap.get("sat_i1_coverage_times_s", {})
        print(f"\n  Phase 2.3 threshold crossings (from overlap.json):")
        for k, v in thresh_times.items():
            print(f"    {k:6s} = {v:.1f}s")

    print("\nGenerating Figure C...")
    plot_figure_c(times_s, fractions, n_cells, overlap, args.snr_thresh,
                  args.out_dir, args.dpi)

    print(f"\nDone.  Figure in: {args.out_dir}\n")


if __name__ == "__main__":
    main()
