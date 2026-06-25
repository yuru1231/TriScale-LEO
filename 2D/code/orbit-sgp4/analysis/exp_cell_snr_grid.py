"""
exp_cell_snr_grid.py

5x5 ROI cell SNR spatial heatmap — BH mode vs 25-beam simultaneous mode.

Answers the committee question: "Do some cells receive worse SNR than others?
Is there a spatial gradient (center vs edge) in the 5x5 ROI?"

Key filtering rules:
  BH mode (nbeams):   Only rows where beam_idx == cell_idx
                      (beam is focused ON this cell → uses peak gain 60.5 dBi)
  25-beam mode:       All rows directly (no beam_idx column; all beams always on)

Expected result:
  BH mode:     < 1 dB spread across 25 cells → spatial uniformity achieved
  25-beam mode: May show slight edge-cell degradation (lower gain off-axis)
                but all cells roughly equal since all beams active simultaneously

Usage:
  python analysis/exp_cell_snr_grid.py ^
      --dir-bh     ns_result/0611/nbeams/starlink/deg25/Helsinki_out ^
      --dir-25beam ns_result/0611/25beams/deg25/Helsinki_out ^
      --scene      starlink_helsinki_deg25 ^
      --snr-thresh 10.0 ^
      --out-dir    ns_result/0611/figures/paper

Output:
  ns_result/0611/figures/paper/fig_cell_snr_grid_<scene>.png / .svg
"""

import argparse
import csv
import math
import os
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


SNR_COL  = "snr_dB"
GRID_N   = 5  # 5x5 UPA


# ── data loading ───────────────────────────────────────────────────────────────

def load_bh_per_cell_snr(data_dir: str) -> dict:
    """
    BH mode: only rows where beam_idx == cell_idx (beam focused on target cell).
    Returns {cell_idx: {'mean_snr', 'median_snr', 'lat', 'lon', 'n'}}
    """
    cell_snrs = defaultdict(list)
    cell_lats = defaultdict(list)
    cell_lons = defaultdict(list)

    for fname in sorted(os.listdir(data_dir)):
        if not fname.endswith("_cells.csv"):
            continue
        fpath = os.path.join(data_dir, fname)
        with open(fpath, newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f, skipinitialspace=True)
            for row in reader:
                try:
                    beam_idx = int(row["beam_idx"])
                    cell_idx = int(row["cell_idx"])
                    if beam_idx != cell_idx:
                        continue  # only focused-beam rows
                    snr = float(row[SNR_COL])
                    if not math.isfinite(snr):
                        continue
                    lat = float(row["cell_lat_deg"])
                    lon = float(row["cell_lon_deg"])
                except (ValueError, KeyError):
                    continue
                cell_snrs[cell_idx].append(snr)
                cell_lats[cell_idx].append(lat)
                cell_lons[cell_idx].append(lon)

    return _summarise(cell_snrs, cell_lats, cell_lons)


def load_25b_per_cell_snr(data_dir: str) -> dict:
    """
    25-beam mode: no beam_idx column; all beams always active.
    Uses raw snr_dB (not sinr_allbeams_dB) to match BH SNR definition.
    Returns {cell_idx: {'mean_snr', 'median_snr', 'lat', 'lon', 'n'}}
    """
    cell_snrs = defaultdict(list)
    cell_lats = defaultdict(list)
    cell_lons = defaultdict(list)

    for fname in sorted(os.listdir(data_dir)):
        if not fname.endswith("_cells.csv"):
            continue
        fpath = os.path.join(data_dir, fname)
        with open(fpath, newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f, skipinitialspace=True)
            for row in reader:
                try:
                    snr = float(row[SNR_COL])
                    if not math.isfinite(snr):
                        continue
                    cell_idx = int(row["cell_idx"])
                    lat = float(row["cell_lat_deg"])
                    lon = float(row["cell_lon_deg"])
                except (ValueError, KeyError):
                    continue
                cell_snrs[cell_idx].append(snr)
                cell_lats[cell_idx].append(lat)
                cell_lons[cell_idx].append(lon)

    return _summarise(cell_snrs, cell_lats, cell_lons)


def _summarise(cell_snrs, cell_lats, cell_lons) -> dict:
    result = {}
    for cidx in sorted(cell_snrs.keys()):
        snrs = sorted(cell_snrs[cidx])
        n = len(snrs)
        mean_v = sum(snrs) / n
        mid = n // 2
        median_v = snrs[mid] if n % 2 else (snrs[mid - 1] + snrs[mid]) / 2
        result[cidx] = {
            "mean_snr":   round(mean_v, 3),
            "median_snr": round(median_v, 3),
            "lat":        round(sum(cell_lats[cidx]) / n, 6),
            "lon":        round(sum(cell_lons[cidx]) / n, 6),
            "n":          n,
        }
    return result


# ── grid layout ────────────────────────────────────────────────────────────────

def build_grid(per_cell: dict, metric: str = "mean_snr") -> tuple:
    """
    Map cell_idx 0–24 to a 5x5 beam-pattern grid using index arithmetic.

    cell_idx is the beam direction index in the 5x5 UPA (not a fixed geographic
    location — the beam footprint sweeps across the ground as the satellite orbits,
    so geographic lat/lon cannot be used to define the grid layout).

    Layout: row = cell_idx // 5,  col = cell_idx % 5
      cell 0  1  2  3  4    ← row 0 (beam directions top row of UPA)
      cell 5  6  7  8  9    ← row 1
      cell 10 11 12 13 14   ← row 2 (centre row)
      cell 15 16 17 18 19   ← row 3
      cell 20 21 22 23 24   ← row 4

    Returns (grid 5x5 ndarray, row_labels, col_labels)
    """
    grid = np.full((GRID_N, GRID_N), np.nan)

    for cidx, v in per_cell.items():
        r = cidx // GRID_N
        c = cidx % GRID_N
        if 0 <= r < GRID_N and 0 <= c < GRID_N:
            grid[r, c] = v[metric]

    row_labels = [f"row {r}" for r in range(GRID_N)]
    col_labels = [f"col {c}" for c in range(GRID_N)]
    return grid, row_labels, col_labels


# ── figure ─────────────────────────────────────────────────────────────────────

def plot_snr_grids(
    bh:        dict,
    b25:       dict,
    snr_thresh: float,
    scene:     str,
    out_dir:   str,
) -> None:
    """
    Side-by-side 5x5 heatmaps (BH left, 25-beam right).
    Both use mean SNR; cell value annotated inside each tile.
    Shared colour scale across both panels for direct comparison.
    """
    g_bh,  rows, cols = build_grid(bh,  "mean_snr")
    g_25b, _,    _    = build_grid(b25, "mean_snr")

    # colour range: use BH range as upper anchor, 25-beam lower anchor
    valid_bh  = g_bh[np.isfinite(g_bh)]
    valid_25b = g_25b[np.isfinite(g_25b)]
    all_valid = np.concatenate([valid_bh, valid_25b])
    if all_valid.size == 0:
        print("[error] No finite SNR values found in either dataset. "
              "Check directory paths and beam_idx / cell_idx columns.")
        return
    vmin = max(float(np.nanmin(all_valid)) - 1, snr_thresh - 20)
    vmax = float(np.nanmax(all_valid)) + 1

    # spread stats
    def _spread(g):
        v = g[np.isfinite(g)]
        return float(np.max(v) - np.min(v)), float(np.mean(v))

    spread_bh, mean_bh   = _spread(g_bh)
    spread_25, mean_25   = _spread(g_25b)

    fig, axes = plt.subplots(1, 2, figsize=(13, 5.5), constrained_layout=True)

    fig.suptitle(
        f"5×5 ROI Cell Mean SNR Grid — {scene}\n"
        f"BH spread: {spread_bh:.3f} dB   |   25-beam spread: {spread_25:.3f} dB",
        fontsize=12, fontweight="bold",
    )

    titles = [
        f"BH Mode (beam_idx=cell_idx)\nMean SNR = {mean_bh:.1f} dB",
        f"25-beam Simultaneous\nMean SNR = {mean_25:.1f} dB",
    ]
    grids = [g_bh, g_25b]

    for ax, grid, title in zip(axes, grids, titles):
        im = ax.imshow(grid, cmap="RdYlGn", vmin=vmin, vmax=vmax, aspect="auto")
        ax.set_xticks(range(GRID_N))
        ax.set_xticklabels(cols, fontsize=7, rotation=30, ha="right")
        ax.set_yticks(range(GRID_N))
        ax.set_yticklabels(rows, fontsize=7)
        ax.set_title(title, fontsize=11, fontweight="bold")
        ax.set_xlabel("Longitude", fontsize=9)
        ax.set_ylabel("Latitude (N at top)", fontsize=9)

        # annotate cell values
        for r in range(GRID_N):
            for c in range(GRID_N):
                v = grid[r, c]
                if not np.isfinite(v):
                    continue
                frac = (v - vmin) / (vmax - vmin) if vmax > vmin else 0.5
                txt_color = "white" if frac < 0.25 or frac > 0.85 else "black"
                ax.text(c, r, f"{v:.1f}", ha="center", va="center",
                        fontsize=9, color=txt_color, fontweight="bold")

    # single shared colorbar
    cbar = fig.colorbar(im, ax=axes.tolist(), shrink=0.85, pad=0.02)
    cbar.set_label("Mean SNR (dB)", fontsize=10)
    cbar.ax.axhline(snr_thresh, color="black", linestyle="--", linewidth=1.5)
    cbar.ax.text(1.05, snr_thresh, f" {snr_thresh:.0f} dB",
                 va="center", fontsize=8,
                 transform=cbar.ax.get_yaxis_transform())

    os.makedirs(out_dir, exist_ok=True)
    fname = f"fig_cell_snr_grid_{scene}"
    for ext in ("png", "svg"):
        p = os.path.join(out_dir, f"{fname}.{ext}")
        fig.savefig(p, dpi=300 if ext == "png" else None, bbox_inches="tight")
        print(f"[write] {p}")
    plt.close(fig)


# ── CLI ────────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="5x5 Cell SNR spatial grid: BH mode vs 25-beam mode"
    )
    p.add_argument("--dir-bh",     required=True,
                   help="Directory with BH-mode sat_*_cells.csv")
    p.add_argument("--dir-25beam", required=True,
                   help="Directory with 25-beam-mode sat_*_cells.csv")
    p.add_argument("--scene",      required=True,
                   help="Scene label used in output filename")
    p.add_argument("--snr-thresh", type=float, default=10.0,
                   help="Reference SNR threshold in dB (default: 10.0)")
    p.add_argument("--out-dir",    default="ns_result/0611/figures/paper",
                   help="Output directory for PNG / SVG")
    return p.parse_args()


def main():
    args = parse_args()

    print(f"[load] BH mode from:    {args.dir_bh}")
    bh = load_bh_per_cell_snr(args.dir_bh)
    print(f"       {len(bh)} cells found")
    if bh:
        snrs = [v["mean_snr"] for v in bh.values()]
        print(f"       mean SNR range: {min(snrs):.2f} – {max(snrs):.2f} dB  "
              f"(spread {max(snrs)-min(snrs):.3f} dB)")

    print(f"[load] 25-beam mode from: {args.dir_25beam}")
    b25 = load_25b_per_cell_snr(args.dir_25beam)
    print(f"       {len(b25)} cells found")
    if b25:
        snrs = [v["mean_snr"] for v in b25.values()]
        print(f"       mean SNR range: {min(snrs):.2f} – {max(snrs):.2f} dB  "
              f"(spread {max(snrs)-min(snrs):.3f} dB)")

    if not bh or not b25:
        print("[error] Empty dataset — check directory paths.")
        return

    # quick diagnostic: print per-cell spread to stdout
    def _report(label, data):
        snrs = sorted(v["mean_snr"] for v in data.values())
        print(f"[{label}] cell mean SNR: "
              f"min={snrs[0]:.2f}  max={snrs[-1]:.2f}  "
              f"spread={snrs[-1]-snrs[0]:.3f} dB  "
              f"overall_mean={sum(snrs)/len(snrs):.2f} dB")

    _report("BH   ", bh)
    _report("25-bm", b25)

    plot_snr_grids(bh, b25, args.snr_thresh, args.scene, args.out_dir)
    print("[done]")


if __name__ == "__main__":
    main()
