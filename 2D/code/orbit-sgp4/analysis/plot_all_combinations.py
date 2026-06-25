"""
analysis/plot_all_combinations.py
----------------------------------
Comprehensive beam gain pattern grids covering EVERY valid combination of
constellation × city × elevation mask in the 0611 simulation run.

Three output figures (PNG, 300 dpi):
  all_beam_25beams.png          3×3 grid — 25beams mode (3 cities × 3 elev)
  all_beam_nbeams_starlink.png  3×2 grid — nbeams Starlink (3 cities × deg25/deg37)
  all_beam_nbeams_iridium.png   3×3 grid — nbeams Iridium (3 cities × 3 elev)

Each subplot: 25×25 SNR heatmap (beam_idx × cell_idx) from the peak-elevation
frame of the longest qualifying pass in that (city, elev) dataset.
Colorscale is shared within each grid so subplots are directly comparable.
N/A combinations (Starlink deg5, not simulated per spec) render as grey panels.

Usage
-----
cd 2D/code/orbit-sgp4/analysis
python plot_all_combinations.py [--out-dir <path>]
"""

import argparse
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
import pandas as pd
import json

# ---------------------------------------------------------------------------
# Paths  (same convention as plot_0611_25beams.py)
# ---------------------------------------------------------------------------

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE       = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "ns_result", "0611"))
B25        = os.path.join(BASE, "25beams")
BN         = os.path.join(BASE, "nbeams")

ELEV_KEYS_ALL    = ["5", "25", "37"]
ELEV_KEYS_NO5    = ["25", "37"]           # Starlink deg5 not simulated
ELEV_DIRS        = {"5": "deg5", "25": "deg25", "37": "deg37"}
ELEV_LABELS      = {"5": "≥5°", "25": "≥25°", "37": "≥37°"}

CITIES     = ["Helsinki_out", "singapore", "tokyo_out"]
CITY_LABEL = {"Helsinki_out": "Helsinki\n(60°N)",
              "singapore":    "Singapore\n(1°N)",
              "tokyo_out":    "Tokyo\n(35°N)"}

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def load_json(path: str) -> dict | None:
    """Load a JSON file; return None if missing or malformed."""
    if not os.path.exists(path):
        return None
    with open(path, encoding="utf-8") as f:
        text = f.read()
    if not text.strip():
        return None
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        # Strip raw control characters inside quoted strings (TLE name issue)
        cleaned, in_str, esc = [], False, False
        for ch in text:
            if in_str:
                if esc:   cleaned.append(ch); esc = False; continue
                if ch == "\\": cleaned.append(ch); esc = True; continue
                if ch == '"':  cleaned.append(ch); in_str = False; continue
                cleaned.append(" " if ord(ch) < 0x20 else ch)
                continue
            cleaned.append(ch)
            if ch == '"': in_str = True
        try:
            return json.loads("".join(cleaned))
        except json.JSONDecodeError:
            return None


def _longest_pass_df(folder: str):
    """Return (DataFrame, pass_dict) for the longest qualifying pass, or (None, None)."""
    status = load_json(os.path.join(folder, "constellation_status.json"))
    if status is None or not status.get("passes"):
        return None, None
    bp = max(status["passes"],
             key=lambda p: p["window_end_s"] - p["window_start_s"])
    csv_path = os.path.join(folder, bp["csv_file"])
    if not os.path.exists(csv_path):
        return None, None
    return pd.read_csv(csv_path), bp


def peak_elev_pivot(folder: str) -> np.ndarray | None:
    """
    For nbeams data: return a (25, 25) ndarray of SNR (beam_idx × cell_idx)
    at the peak-elevation frame of the longest pass.
    Returns None if folder or data is unavailable.
    """
    df, _ = _longest_pass_df(folder)
    if df is None:
        return None
    t_peak = df.groupby("time_s")["elevation_deg"].first().idxmax()
    sub    = df[df["time_s"] == t_peak]
    pivot  = sub.pivot_table(
        values="snr_dB", index="beam_idx", columns="cell_idx", aggfunc="mean"
    ).reindex(index=range(25), columns=range(25))
    return pivot.values


def peak_elev_snr_5x5(folder: str) -> np.ndarray | None:
    """
    For 25beams data: return a (5, 5) ndarray of per-cell SNR at the
    peak-elevation frame of the longest pass. Cells 0–24 are arranged
    row-major: row r = cells [5r .. 5r+4].
    Returns None if folder or data is unavailable.
    """
    df, _ = _longest_pass_df(folder)
    if df is None:
        return None
    t_peak = df.groupby("time_s")["elevation_deg"].first().idxmax()
    sub    = df[df["time_s"] == t_peak].sort_values("cell_idx")
    snr    = sub["snr_dB"].values
    if len(snr) < 25:
        return None
    return snr[:25].reshape(5, 5)


def nn_dir_with_fallback(const: str, deg_key: str, city: str) -> str:
    """
    Return nbeams directory path, falling back to <city>_out if <city> is absent.
    Handles the Singapore nbeams/starlink/deg25 re-run stored in singapore_out.
    """
    base = os.path.join(BN, const, ELEV_DIRS[deg_key], city)
    if not os.path.isdir(base):
        alt = base + "_out"
        if os.path.isdir(alt):
            return alt
    return base


def draw_na_panel(ax):
    """Render a grey N/A panel in *ax*."""
    ax.set_facecolor("#d0d0d0")
    ax.text(0.5, 0.5, "N/A", transform=ax.transAxes,
            ha="center", va="center", fontsize=11, color="#555555",
            fontweight="bold")
    ax.set_xticks([])
    ax.set_yticks([])


def build_beam_pivot_grid(pivots: list[list], cities: list[str],
                          elev_keys: list[str], out_path: str):
    """
    rows=cities × cols=elev_keys grid of 25×25 beam-gain heatmaps (nbeams mode).
    pivots[row][col]: (25,25) ndarray or None → grey N/A panel.
    Shared colorscale across all valid panels.
    """
    nrows, ncols = len(cities), len(elev_keys)
    all_vals = [v for row in pivots for v in row if v is not None]
    vmin = min(m.min() for m in all_vals) if all_vals else -20
    vmax = max(m.max() for m in all_vals) if all_vals else 20

    fig = plt.figure(figsize=(4 * ncols + 1.2, 4 * nrows))
    gs  = gridspec.GridSpec(nrows, ncols + 1, figure=fig,
                            wspace=0.08, hspace=0.35,
                            width_ratios=[1] * ncols + [0.06])
    diag, last_im = np.arange(25), None

    for r, city in enumerate(cities):
        for c, dk in enumerate(elev_keys):
            ax  = fig.add_subplot(gs[r, c])
            piv = pivots[r][c]
            if piv is None:
                draw_na_panel(ax)
            else:
                last_im = ax.imshow(piv, cmap="RdYlGn", vmin=vmin, vmax=vmax,
                                    aspect="auto", origin="upper")
                ax.plot(diag, diag, "w--", linewidth=0.8, alpha=0.6)
            if r == 0:
                ax.set_title(ELEV_LABELS[dk], fontsize=9, pad=4)
            if c == 0:
                ax.set_ylabel(CITY_LABEL[city], fontsize=8, labelpad=6)
            else:
                ax.set_yticklabels([])
            ax.set_xticks(range(0, 25, 5))
            ax.set_yticks(range(0, 25, 5))
            ax.tick_params(labelsize=6)

    if last_im is not None:
        cax = fig.add_subplot(gs[:, ncols])
        fig.colorbar(last_im, cax=cax, label="SNR (dB)")

    fig.text(0.5, 0.02, "Cell index", ha="center", fontsize=9)
    fig.text(0.04, 0.5, "Beam index", va="center", rotation="vertical",
             fontsize=9)
    _save(fig, out_path)


def build_snr5x5_grid(panels: list[list], cities: list[str],
                      elev_keys: list[str], out_path: str):
    """
    rows=cities × cols=elev_keys grid of 5×5 per-cell SNR heatmaps (25beams mode).
    panels[row][col]: (5,5) ndarray or None → grey N/A panel.
    Each cell in the 5×5 represents one of the 25 ROI cells (row-major order).
    Shared colorscale across all valid panels.
    """
    nrows, ncols = len(cities), len(elev_keys)
    all_vals = [v for row in panels for v in row if v is not None]
    vmin = min(m.min() for m in all_vals) if all_vals else 0
    vmax = max(m.max() for m in all_vals) if all_vals else 15

    fig = plt.figure(figsize=(3.5 * ncols + 1.2, 3.5 * nrows))
    gs  = gridspec.GridSpec(nrows, ncols + 1, figure=fig,
                            wspace=0.12, hspace=0.35,
                            width_ratios=[1] * ncols + [0.06])
    last_im = None

    for r, city in enumerate(cities):
        for c, dk in enumerate(elev_keys):
            ax  = fig.add_subplot(gs[r, c])
            pnl = panels[r][c]
            if pnl is None:
                draw_na_panel(ax)
            else:
                last_im = ax.imshow(pnl, cmap="RdYlGn", vmin=vmin, vmax=vmax,
                                    aspect="equal", origin="upper")
                # Annotate each cell with its SNR value
                for ri in range(5):
                    for ci in range(5):
                        ax.text(ci, ri, f"{pnl[ri, ci]:.1f}",
                                ha="center", va="center", fontsize=6,
                                color="black")
            if r == 0:
                ax.set_title(ELEV_LABELS[dk], fontsize=9, pad=4)
            if c == 0:
                ax.set_ylabel(CITY_LABEL[city], fontsize=8, labelpad=6)
            ax.set_xticks(range(5))
            ax.set_yticks(range(5))
            ax.set_xticklabels([f"c{i*5}" for i in range(5)], fontsize=5)
            ax.set_yticklabels([f"r{i}" for i in range(5)], fontsize=5)

    if last_im is not None:
        cax = fig.add_subplot(gs[:, ncols])
        fig.colorbar(last_im, cax=cax, label="SNR (dB)")

    fig.text(0.5, 0.02, "Column", ha="center", fontsize=9)
    fig.text(0.04, 0.5, "Row", va="center", rotation="vertical", fontsize=9)
    _save(fig, out_path)


def _save(fig, out_path: str):
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"  → {out_path}")
    plt.close(fig)

# ---------------------------------------------------------------------------
# Three grid generators
# ---------------------------------------------------------------------------

def plot_25beams_grid(out_dir: str):
    """
    3×3 grid for 25beams mode.
    Each subplot: 5×5 per-cell SNR heatmap at peak elevation (no beam_idx in 25beams CSV).
    """
    print("[Grid] 25beams per-cell SNR (5×5) ...")
    panels = []
    for city in CITIES:
        row = []
        for dk in ELEV_KEYS_ALL:
            folder = os.path.join(B25, ELEV_DIRS[dk], city)
            row.append(peak_elev_snr_5x5(folder))
        panels.append(row)
    build_snr5x5_grid(panels, CITIES, ELEV_KEYS_ALL,
                      out_path=os.path.join(out_dir, "all_beam_25beams.png"))


def plot_nbeams_starlink_grid(out_dir: str):
    """
    3×2 beam gain grid for nbeams Starlink.
    deg5 excluded (below Starlink spec, not simulated).
    """
    print("[Grid] nbeams Starlink beam gain pattern (25×25) ...")
    pivots = []
    for city in CITIES:
        row = []
        for dk in ELEV_KEYS_NO5:
            folder = nn_dir_with_fallback("starlink", dk, city)
            row.append(peak_elev_pivot(folder))
        pivots.append(row)
    build_beam_pivot_grid(pivots, CITIES, ELEV_KEYS_NO5,
                          out_path=os.path.join(out_dir, "all_beam_nbeams_starlink.png"))


def plot_nbeams_iridium_grid(out_dir: str):
    """3×3 beam gain grid for nbeams Iridium (all three elevation masks)."""
    print("[Grid] nbeams Iridium beam gain pattern (25×25) ...")
    pivots = []
    for city in CITIES:
        row = []
        for dk in ELEV_KEYS_ALL:
            folder = nn_dir_with_fallback("iridium", dk, city)
            row.append(peak_elev_pivot(folder))
        pivots.append(row)
    build_beam_pivot_grid(pivots, CITIES, ELEV_KEYS_ALL,
                          out_path=os.path.join(out_dir, "all_beam_nbeams_iridium.png"))

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--out-dir", default=None,
                   help="Output directory (default: ns_result/0611/figures/all)")
    return p.parse_args()


def main():
    args    = parse_args()
    out_dir = args.out_dir or os.path.join(BASE, "figures", "all")

    print(f"Output : {out_dir}\n")

    plot_25beams_grid(out_dir)
    plot_nbeams_starlink_grid(out_dir)
    plot_nbeams_iridium_grid(out_dir)

    print("\nDone.")


if __name__ == "__main__":
    main()
