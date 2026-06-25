"""
analysis/plot_0611_25beams.py
------------------------------
Four figures for the 0611 simulation run (25-beam ROI model).

  Fig 1  25×25 beam-gain pattern heatmap — validates the ROI beam model
  Fig 2  Spatial snapshot series (25beams vs nbeams, 4 time points each)
  Fig 3  Per-cell SNR: 1-beam focused vs 25-beam simultaneous vs 25-beam SINR
  Fig 4  Coverage availability — Starlink vs Iridium, 3 cities × 3 elevation masks

Usage
-----
cd 2D/code/orbit-sgp4/analysis
python plot_0611_25beams.py [--snr-thresh 3.0] [--out-dir <path>]
"""

import argparse
import glob
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.cm as cm
import matplotlib.colors as mcolors
import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE       = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "ns_result", "0611"))
B25        = os.path.join(BASE, "25beams")   # 25 simultaneous beams
BN         = os.path.join(BASE, "nbeams")    # 1-beam hopping (n_beams=1)

ELEV_KEYS  = ["5", "25", "37"]
ELEV_DIRS  = {"5": "deg5", "25": "deg25", "37": "deg37"}
CITIES     = ["Helsinki_out", "singapore", "tokyo_out"]
CITY_SHORT = {"Helsinki_out": "Helsinki (60°N)",
              "singapore":    "Singapore (1°N)",
              "tokyo_out":    "Tokyo (35°N)"}
CONSTS     = ["starlink", "iridium"]
CONST_LABEL = {"starlink": "Starlink", "iridium": "Iridium"}
CONST_COLOR = {"starlink": "#2980b9",  "iridium": "#e67e22"}

# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--snr-thresh", type=float, default=3.0)
    p.add_argument("--out-dir", default=None)
    return p.parse_args()

# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------

def load_json(path: str) -> dict | None:
    if not os.path.exists(path):
        return None
    with open(path, encoding="utf-8") as f:
        text = f.read()
    if not text.strip():
        print(f"  Warning: empty JSON, skipping {path}")
        return None
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        # Some generated status files contain raw control characters embedded
        # inside quoted strings (for example a satellite name split by a newline).
        # Sanitize only those in-string control chars so plotting can continue.
        cleaned = []
        in_string = False
        escape = False
        for ch in text:
            if in_string:
                if escape:
                    cleaned.append(ch)
                    escape = False
                    continue
                if ch == "\\":
                    cleaned.append(ch)
                    escape = True
                    continue
                if ch == "\"":
                    cleaned.append(ch)
                    in_string = False
                    continue
                if ord(ch) < 0x20:
                    cleaned.append(" ")
                    continue
                cleaned.append(ch)
                continue
            cleaned.append(ch)
            if ch == "\"":
                in_string = True
        try:
            return json.loads("".join(cleaned))
        except json.JSONDecodeError:
            print(f"  Warning: malformed JSON, skipping {path}")
            return None


def n25_dir(deg_key: str, city: str) -> str:
    return os.path.join(B25, ELEV_DIRS[deg_key], city)


def nn_dir(const: str, deg_key: str, city: str) -> str:
    # Primary path.  If the city directory doesn't exist but a "<city>_out"
    # variant does (e.g. nbeams/starlink/deg25/singapore_out), use that instead.
    # This handles the Singapore nbeams/starlink re-run that wrote into singapore_out.
    base = os.path.join(BN, const, ELEV_DIRS[deg_key], city)
    if not os.path.isdir(base):
        alt = base + "_out"
        if os.path.isdir(alt):
            return alt
    return base


def coverage_pct(passes: list, window_s: int = 6000) -> float:
    if not passes:
        return 0.0
    cnt = np.zeros(window_s + 1, dtype=int)
    for p in passes:
        s = int(p["window_start_s"])
        e = min(int(p["window_end_s"]) + 1, len(cnt))
        cnt[s:e] += 1
    return 100.0 * float(np.mean(cnt > 0))


def save_fig(fig, out_dir: str, name: str):
    os.makedirs(out_dir, exist_ok=True)
    png_path = os.path.join(out_dir, name)
    svg_path = png_path.replace(".png", ".svg")
    fig.savefig(png_path, dpi=300, bbox_inches="tight")
    fig.savefig(svg_path, bbox_inches="tight")
    print(f"  → {png_path}")
    print(f"  → {svg_path}")
    plt.close(fig)

# ---------------------------------------------------------------------------
# Fig 1 — 25×25 Beam Gain Pattern heatmap
# ---------------------------------------------------------------------------

def plot_fig1_beam_gain_pattern(out_dir: str):
    """
    Two-panel 25×25 beam gain heatmap (beam_idx × cell_idx, SNR in dB).

    Dataset: Starlink / Singapore / elev ≥ 25°.
    Reason for Singapore: at 1°N latitude, Starlink (53° incl) satellites can
    pass near-zenith (~80–90°), making the snapshot vs mean comparison visually
    most distinct.  Helsinki (60°N) and Tokyo (35°N) show the same diagonal-
    dominant structure (it is a UPA geometry property, not orbit-dependent) but
    with lower peak elevations the contrast is less dramatic.

    Left  (a): snapshot at the peak-elevation frame of the longest qualifying
               pass.  Near-zenith geometry means UPA beams are steered near
               broadside — the direction of narrowest main lobe and lowest
               sidelobes.  Off-diagonal gains are minimised, giving the
               SHARPEST diagonal contrast in the dataset.  Absolute SNR is
               also highest here (minimum slant range and atmospheric loss).
    Right (b): element-wise mean across the peak-elevation frame of every
               qualifying pass.  Off-nadir passes require off-axis beam
               steering; UPA sidelobes rise and the main lobe broadens,
               elevating off-diagonal elements.  Averaging across many passes
               (25°–90° peak elevations) reduces the apparent contrast relative
               to the near-zenith snapshot.

    Purpose: Left panel shows the sharpest beam-to-cell discrimination (UPA
    near-broadside); Right panel proves the diagonal-dominant structure holds
    consistently across all pass geometries, not just the ideal near-zenith
    case.

    Shared colour scale so both panels can be compared directly.
    """
    print("[Fig 1] Beam gain pattern ...")

    # Singapore has near-zenith passes (elev ~90°) giving maximum beam contrast.
    # This makes snapshot vs mean visually distinct: snapshot = tight diagonal at
    # high gain; mean = averaged across 25°–90° passes, diagonal still dominant
    # but contrast is reduced.
    folder = nn_dir("starlink", "25", "singapore")
    status = load_json(os.path.join(folder, "constellation_status.json"))

    # ---- Panel (a): longest pass, peak-elevation frame ----------------------
    bp = max(status["passes"],
             key=lambda p: p["window_end_s"] - p["window_start_s"])
    df_snap = pd.read_csv(os.path.join(folder, bp["csv_file"]))
    t_peak  = df_snap.groupby("time_s")["elevation_deg"].first().idxmax()
    sub     = df_snap[df_snap["time_s"] == t_peak]
    pivot_snap = sub.pivot_table(
        values="snr_dB", index="beam_idx", columns="cell_idx", aggfunc="mean"
    ).reindex(index=range(25), columns=range(25))

    # ---- Panel (b): mean across all passes (peak-elevation frame each) ------
    matrices = []
    for p in status["passes"]:
        csv_path = os.path.join(folder, p["csv_file"])
        if not os.path.exists(csv_path):
            continue
        df = pd.read_csv(csv_path)
        t_pk = df.groupby("time_s")["elevation_deg"].first().idxmax()
        piv  = df[df["time_s"] == t_pk].pivot_table(
            values="snr_dB", index="beam_idx", columns="cell_idx", aggfunc="mean"
        ).reindex(index=range(25), columns=range(25))
        matrices.append(piv.values)
    pivot_mean = np.nanmean(np.stack(matrices), axis=0)

    # ---- Shared colour scale -------------------------------------------------
    all_vals = np.concatenate([pivot_snap.values.flatten(),
                                pivot_mean.flatten()])
    vmin = np.nanmin(all_vals)
    vmax = np.nanmax(all_vals)

    # ---- Layout -------------------------------------------------------------
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.5))
    fig.subplots_adjust(right=0.88, wspace=0.12)

    diag_xy = np.arange(25)
    for ax, data, title in zip(
        axes,
        [pivot_snap.values, pivot_mean],
        ["(a) Peak-elevation snapshot", "(b) Mean across all passes"]
    ):
        im = ax.imshow(data, cmap="RdYlGn", vmin=vmin, vmax=vmax,
                       aspect="auto", origin="upper")
        # White dashed line marks the on-target diagonal
        ax.plot(diag_xy, diag_xy, "w--", linewidth=1.0, alpha=0.7)
        ax.set_xlabel("Cell index", fontsize=9)
        ax.set_ylabel("Beam index", fontsize=9)
        ax.set_xticks(range(0, 25, 5))
        ax.set_yticks(range(0, 25, 5))
        ax.set_title(title, fontsize=9)

    # Shared colorbar to the right of both panels
    cbar_ax = fig.add_axes([0.91, 0.15, 0.02, 0.70])
    fig.colorbar(im, cax=cbar_ax, label="SNR (dB)")

    save_fig(fig, out_dir, "fig1_beam_gain_pattern.png")

# ---------------------------------------------------------------------------
# Fig 2 — Spatial snapshot series: 25beams (top) vs nbeams (bottom)
# ---------------------------------------------------------------------------

def plot_fig2_spatial_snapshots(out_dir: str, snr_thresh: float):
    """
    Longest pass from each dataset independently (25beams and nbeams use different
    TLE sources so no common satellites exist).  4 evenly-spaced time points each.
    Top row    : 25beams — all 25 cells illuminated simultaneously, SNR ~3-5 dB
    Bottom row : nbeams  — one focused beam per cell, SNR ~17 dB when on-target
    """
    print("[Fig 2] Spatial snapshot series ...")

    def best_pass_csv(status_path: str) -> tuple:
        """Return (csv_path, pass_dict) for the longest qualifying pass."""
        st = load_json(status_path)
        if st is None or not st.get("passes"):
            return None, None
        bp = max(st["passes"],
                 key=lambda p: p["window_end_s"] - p["window_start_s"])
        csv_path = os.path.join(os.path.dirname(status_path), bp["csv_file"])
        return (csv_path, bp) if os.path.exists(csv_path) else (None, None)

    csv25, pass25 = best_pass_csv(
        os.path.join(n25_dir("25", "Helsinki_out"), "constellation_status.json"))
    csvN,  passN  = best_pass_csv(
        os.path.join(nn_dir("starlink", "25", "Helsinki_out"), "constellation_status.json"))

    if csv25 is None or csvN is None:
        print("  CSV not found, skipping Fig 2.")
        return

    df25 = pd.read_csv(csv25)
    dfN  = pd.read_csv(csvN)

    def pick_times(df: pd.DataFrame) -> list:
        ts = sorted(df["time_s"].unique())
        n  = len(ts)
        return [ts[0], ts[n // 3], ts[2 * n // 3], ts[-1]]

    picks25 = pick_times(df25)
    picksN  = pick_times(dfN)

    # Per-row colour scales: 25-beam SNR is ~3-10 dB, 1-beam is ~5-25 dB.
    # A shared scale compresses the 25-beam row to uniform dark colour.
    # Independent scales make each row's spatial variation readable.
    snr25_all = df25["snr_dB"].values
    snrN_all  = dfN[dfN["beam_idx"] == dfN["cell_idx"]]["snr_dB"].values
    row_norms = [
        mcolors.Normalize(vmin=snr25_all.min(), vmax=snr25_all.max()),
        mcolors.Normalize(vmin=snrN_all.min(),  vmax=snrN_all.max()),
    ]
    cmap = cm.plasma

    # ROI centre from status
    st25 = load_json(os.path.join(n25_dir("25", "Helsinki_out"), "constellation_status.json"))
    roi_lat = st25["roi_lat_deg"]
    roi_lon = st25["roi_lon_deg"]

    fig = plt.figure(figsize=(15, 7))
    gs  = gridspec.GridSpec(2, 5, figure=fig,
                            wspace=0.08, hspace=0.45,
                            width_ratios=[1, 1, 1, 1, 0.06])

    row_meta = [
        (df25, picks25, pass25, "25-beam simultaneous"),
        (dfN,  picksN,  passN,  "1-beam focused"),
    ]

    for row, (df, picks, bp, row_label) in enumerate(row_meta):
        norm = row_norms[row]
        t0 = picks[0]
        for col, t in enumerate(picks):
            if row == 0:
                sub = df[df["time_s"] == t]
            else:
                sub = df[(df["time_s"] == t) & (df["beam_idx"] == df["cell_idx"])]

            ax = fig.add_subplot(gs[row, col])
            if len(sub):
                ax.scatter(
                    sub["cell_lon_deg"], sub["cell_lat_deg"],
                    c=sub["snr_dB"], cmap=cmap, norm=norm,
                    s=80, marker="s", linewidths=0.4, edgecolors="white"
                )
            ax.axhline(roi_lat, color="white", linewidth=0.6, linestyle=":", alpha=0.5)
            ax.axvline(roi_lon, color="white", linewidth=0.6, linestyle=":", alpha=0.5)
            ax.plot(roi_lon, roi_lat, "w*", markersize=9,
                    markeredgecolor="black", markeredgewidth=0.7, zorder=5)

            ax.set_xlim(roi_lon - 4, roi_lon + 4)
            ax.set_ylim(roi_lat - 2, roi_lat + 2)
            ax.tick_params(labelsize=6)

            elev = sub["elevation_deg"].iloc[0] if len(sub) else 0
            ax.set_title(f"t+{t - t0:.0f} s\nelev={elev:.1f}°", fontsize=8)
            if col == 0:
                ax.set_ylabel("Latitude (°)", fontsize=7)
            else:
                ax.set_yticklabels([])
            if row == 1:
                ax.set_xlabel("Longitude (°)", fontsize=7)

    # Per-row colorbars: independent scales for 25-beam and 1-beam rows.
    # Row 0 (25-beam): SNR ~0-10 dB. Draw the threshold line here (3 dB is within range).
    # Row 1 (1-beam):  SNR ~5-25 dB. 3 dB threshold is below vmin → omit line.
    cax0 = fig.add_subplot(gs[0, 4])
    sm0  = cm.ScalarMappable(cmap=cmap, norm=row_norms[0])
    sm0.set_array([])
    plt.colorbar(sm0, cax=cax0, label="SNR (dB)")
    cax0.axhline(snr_thresh, color="red", linewidth=1.5, linestyle="--")
    cax0.text(0.5, snr_thresh, f" {snr_thresh:.0f} dB",
              transform=cax0.get_yaxis_transform(), color="red", fontsize=7, va="bottom")

    cax1 = fig.add_subplot(gs[1, 4])
    sm1  = cm.ScalarMappable(cmap=cmap, norm=row_norms[1])
    sm1.set_array([])
    plt.colorbar(sm1, cax=cax1, label="SNR (dB)")

    # Row labels on the left margin (outside subplots, avoid ylabel conflict)
    row_labels = ["25-beam simultaneous", "1-beam focused"]
    for row_i, label in enumerate(row_labels):
        # y position: centre of each row in figure coordinates
        y_pos = 0.75 - row_i * 0.52
        fig.text(0.01, y_pos, label, rotation=90, va="center",
                 ha="center", fontsize=8, fontweight="bold")

    save_fig(fig, out_dir, "fig2_spatial_snapshots.png")

# ---------------------------------------------------------------------------
# Fig 3 — Per-cell SNR: 1-beam vs 25-beam vs 25-beam SINR
# ---------------------------------------------------------------------------

def plot_fig3_per_cell_snr(out_dir: str, snr_thresh: float):
    """
    Each dataset uses its own longest pass (different TLE sources → no common satellite).
    For both passes at their respective peak elevation times, compare per cell:
      Blue  bars : 25beams SNR    (25 beams simultaneous, no ext. interference)
      Red   bars : 25beams SINR   (all constellation beams → heavy interference)
      Green line : nbeams SNR     (1 focused beam pointed directly at that cell)
    """
    print("[Fig 3] Per-cell SNR comparison ...")

    def best_pass_csv(status_path: str) -> tuple:
        st = load_json(status_path)
        if st is None or not st.get("passes"):
            return None, None
        bp = max(st["passes"],
                 key=lambda p: p["window_end_s"] - p["window_start_s"])
        csv_path = os.path.join(os.path.dirname(status_path), bp["csv_file"])
        return (csv_path, bp) if os.path.exists(csv_path) else (None, None)

    csv25, pass25 = best_pass_csv(
        os.path.join(n25_dir("25", "Helsinki_out"), "constellation_status.json"))
    csvN,  passN  = best_pass_csv(
        os.path.join(nn_dir("starlink", "25", "Helsinki_out"), "constellation_status.json"))

    if csv25 is None or csvN is None:
        print("  CSV not found, skipping Fig 3.")
        return

    df25 = pd.read_csv(csv25)
    dfN  = pd.read_csv(csvN)

    # Each dataset's own peak elevation time
    t_peak25 = df25.groupby("time_s")["elevation_deg"].first().idxmax()
    t_peakN  = dfN.groupby("time_s")["elevation_deg"].first().idxmax()

    sub25 = df25[df25["time_s"] == t_peak25].sort_values("cell_idx")
    subN  = dfN[(dfN["time_s"] == t_peakN) &
                (dfN["beam_idx"] == dfN["cell_idx"])].sort_values("cell_idx")

    cells  = sub25["cell_idx"].values
    snr25  = sub25["snr_dB"].values
    sinr25 = sub25["sinr_allbeams_dB"].values
    snrN   = subN.set_index("cell_idx").reindex(cells)["snr_dB"].values

    x    = np.arange(len(cells))
    w    = 0.28
    fig, ax = plt.subplots(figsize=(14, 5.5))

    ax.bar(x - w,  snr25,  width=w, color="#3498db", alpha=0.85,
           label="25-beam SNR (simultaneous, no ext. interference)")
    ax.bar(x,      sinr25, width=w, color="#e74c3c", alpha=0.85,
           label="25-beam SINR (full constellation co-channel interference)")
    ax.plot(x + w/2, snrN, "o-", color="#27ae60", linewidth=2,
            markersize=6, label="1-beam focused SNR (direct beam on each cell)")

    ax.axhline(snr_thresh, color="black", linewidth=1.0, linestyle="--", alpha=0.6)
    ax.text(25.3, snr_thresh, f"{snr_thresh:.0f} dB threshold",
            fontsize=8, va="center")

    # Row boundary annotations
    for r in range(1, 5):
        ax.axvline(r * 5 - 0.5, color="gray", linewidth=0.6, linestyle=":", alpha=0.5)
    for r in range(5):
        ax.text(r * 5 + 2, ax.get_ylim()[0] + 1 if ax.get_ylim()[0] > -15 else -12,
                f"Row {r}", ha="center", fontsize=7, color="gray")

    ax.set_xlabel("Cell index (0–24, arranged as 5 rows × 5 columns)")
    ax.set_ylabel("SNR / SINR (dB)")
    ax.set_xticks(x)
    ax.set_xticklabels(cells, fontsize=7)
    ax.legend(fontsize=9, loc="upper right")
    ax.grid(True, axis="y", alpha=0.25)
    save_fig(fig, out_dir, "fig3_per_cell_snr.png")

# ---------------------------------------------------------------------------
# Fig 4 — Coverage availability: Starlink vs Iridium
# ---------------------------------------------------------------------------

def plot_fig4_coverage(out_dir: str):
    """
    For each city: grouped bars showing coverage fraction (% of 6000 s with
    ≥1 satellite visible) for Starlink and Iridium at all three elevation masks.

    Key result: Iridium (polar orbit, 86° inclination) serves Helsinki at deg37
    while Starlink (53° inclination) has zero qualifying passes.
    """
    print("[Fig 4] Coverage availability ...")

    fig, axes = plt.subplots(1, 3, figsize=(14, 5.5),
                             sharey=True, gridspec_kw={"wspace": 0.12})

    x      = np.arange(len(ELEV_KEYS))
    w      = 0.32
    offset = {"starlink": -w / 2, "iridium": w / 2}

    for ax_idx, city in enumerate(CITIES):
        ax = axes[ax_idx]

        from matplotlib.patches import Patch
        legend_handles = []
        for const in CONSTS:
            vals = []
            for dk in ELEV_KEYS:
                st = load_json(
                    os.path.join(nn_dir(const, dk, city), "constellation_status.json")
                )
                # None = N/A (not simulated — e.g. Starlink deg5 is below spec)
                # 0.0  = simulated but zero qualifying passes
                vals.append(
                    None if st is None
                    else coverage_pct(st.get("passes", []),
                                      int(st.get("window_s", 6000)))
                )

            legend_handles.append(
                Patch(facecolor=CONST_COLOR[const], alpha=0.85,
                      label=CONST_LABEL[const])
            )

            for i, val in enumerate(vals):
                xi = x[i] + offset[const]
                if val is None:
                    ax.bar(xi, 8, width=w * 0.92,
                           color="lightgrey", alpha=0.7,
                           edgecolor="grey", linewidth=0.8, hatch="//")
                    ax.text(xi, 9.5, "N/A",
                            ha="center", va="bottom", fontsize=7, color="grey")
                else:
                    ax.bar(xi, val, width=w * 0.92,
                           color=CONST_COLOR[const], alpha=0.85,
                           edgecolor="white", linewidth=0.4)
                    ax.text(xi, val + 1.2, f"{val:.0f}%",
                            ha="center", va="bottom", fontsize=8)

        ax.set_title(CITY_SHORT[city], fontsize=9)
        ax.set_xticks(x)
        ax.set_xticklabels(["≥5°", "≥25°", "≥37°"])
        ax.set_ylim(0, 115)
        ax.set_ylabel("Coverage (% of 6000 s)" if ax_idx == 0 else "")
        ax.grid(True, axis="y", alpha=0.25)
        if ax_idx == 0:
            ax.legend(handles=legend_handles, fontsize=9)

    # Annotate the Helsinki / deg37 contrast
    ax0 = axes[0]
    ax0.annotate(
        "Iridium polar orbit\nserves Helsinki at 37°\n(Starlink: 0%)",
        xy=(2 + offset["iridium"], 17.6),
        xytext=(1.2, 55),
        fontsize=8, color=CONST_COLOR["iridium"],
        arrowprops=dict(arrowstyle="->", color=CONST_COLOR["iridium"], lw=1.2)
    )

    save_fig(fig, out_dir, "fig4_coverage_availability.png")

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    args    = parse_args()
    out_dir = args.out_dir or os.path.join(BASE, "25beams", "figures")

    print(f"Output        : {out_dir}")
    print(f"SNR threshold : {args.snr_thresh} dB\n")

    plot_fig1_beam_gain_pattern(out_dir)
    plot_fig2_spatial_snapshots(out_dir, args.snr_thresh)
    plot_fig3_per_cell_snr(out_dir, args.snr_thresh)
    plot_fig4_coverage(out_dir)

    print("\nDone.")


if __name__ == "__main__":
    main()
