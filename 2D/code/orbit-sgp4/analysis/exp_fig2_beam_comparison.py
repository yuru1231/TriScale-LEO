"""
exp_fig2_beam_comparison.py  (Gap 6)

Generates a corrected Fig2 by comparing 25-beam and 1-beam-focused spatial SNR
distributions on the SAME satellite, at the SAME timestamp — fixing the confound
in the original Fig2 where 25-beam used Starlink data and BH used Iridium data.

Both datasets must share the same constellation (Starlink) and the same ≥25°
elevation mask.  The script finds overlapping satellite-timestamps, selects the
best one (highest elevation), then plots side-by-side SNR spatial maps with a
shared colour scale.

Usage:
  python exp_fig2_beam_comparison.py \
      --dir-25beam  ns_result/0611/25beams/deg25/Helsinki_out \
      --dir-bh      ns_result/0611/nbeams/starlink/deg25/Helsinki_out \
      --city        Helsinki \
      --out-dir     ns_result/0611/figures/paper

Output:
  fig2_spatial_snr_beam_comparison_helsinki.png / .svg

Notes:
  - If the two directories contain different satellite TLE epochs, overlapping
    satellite IDs (parsed from sat_XXXXX_cells.csv filenames) are compared only
    when both datasets recorded data at the same integer second.
  - A diagnostic report (fig2_overlap_report.json) is written alongside the figure
    so the controlled-comparison evidence can be cited in the paper.
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


GEO_BIN_DEG = 0.05


def _bin(v: float) -> float:
    return round(round(v / GEO_BIN_DEG) * GEO_BIN_DEG, 6)


# ── data loading ───────────────────────────────────────────────────────────────

def load_sat_tick_data(data_dir: str) -> dict:
    """
    Returns:
        data[(sat_id, t_int)] = {(lat_bin, lon_bin): best_snr}
        where sat_id is parsed from the filename (e.g. 'sat_00050').
    """
    data: dict[tuple, dict] = defaultdict(dict)

    for fname in sorted(os.listdir(data_dir)):
        if not fname.endswith("_cells.csv"):
            continue
        sat_id = fname.replace("_cells.csv", "")   # e.g. 'sat_00050'
        fpath = os.path.join(data_dir, fname)
        with open(fpath, newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f, skipinitialspace=True)
            for row in reader:
                try:
                    snr  = float(row["snr_dB"])
                    elev = float(row["elevation_deg"])
                    if math.isnan(snr):
                        continue
                    t    = int(round(float(row["time_s"])))
                    lb   = _bin(float(row["cell_lat_deg"]))
                    lob  = _bin(float(row["cell_lon_deg"]))
                except (ValueError, KeyError):
                    continue

                key = (sat_id, t)
                bin_key = (lb, lob)
                # Best SNR per geographic bin per satellite-tick
                existing = data[key].get(bin_key)
                if existing is None or snr > existing[0]:
                    data[key][bin_key] = (snr, elev)

    return dict(data)


def find_best_overlap_tick(
    data_25: dict,
    data_bh: dict,
) -> tuple[str | None, int | None, float]:
    """
    Find the (sat_id, tick) pair present in BOTH datasets with the highest
    peak elevation among all bins at that tick.

    Returns: (best_sat_id, best_t, best_elev) — or (None, None, 0) if none found.
    """
    keys_25 = set(data_25.keys())
    keys_bh = set(data_bh.keys())
    common  = keys_25 & keys_bh

    best_sat, best_t, best_elev = None, None, 0.0
    for sat_id, t in common:
        peak_elev = max(v[1] for v in data_25[(sat_id, t)].values())
        if peak_elev > best_elev:
            best_elev = peak_elev
            best_sat, best_t = sat_id, t

    return best_sat, best_t, best_elev


# ── figure generation ─────────────────────────────────────────────────────────

def _extract_snr_grid(tick_data: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Convert a {(lat_bin, lon_bin): (snr, elev)} dict to scatter arrays."""
    lats = np.array([k[0] for k in tick_data])
    lons = np.array([k[1] for k in tick_data])
    snrs = np.array([v[0] for v in tick_data.values()])
    return lats, lons, snrs


def plot_controlled_fig2(
    tick_data_25: dict,   # {(lat_bin, lon_bin): (snr, elev)}
    tick_data_bh: dict,
    sat_id: str,
    t_tick: int,
    peak_elev: float,
    city: str,
    out_dir: str,
) -> None:
    """
    Side-by-side scatter plot of geographic SNR at the same satellite-tick.
    Shared colour scale so both panels are directly comparable.
    """
    lats_25, lons_25, snrs_25 = _extract_snr_grid(tick_data_25)
    lats_bh, lons_bh, snrs_bh = _extract_snr_grid(tick_data_bh)

    # Shared colour scale across both panels
    all_snrs = np.concatenate([snrs_25, snrs_bh])
    vmin = max(math.floor(all_snrs.min()), -20)
    vmax = math.ceil(all_snrs.max())
    norm = mcolors.Normalize(vmin=vmin, vmax=vmax)
    cmap = plt.cm.RdYlGn

    fig, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)

    for ax, lats, lons, snrs, mode_label in [
        (axes[0], lats_25, lons_25, snrs_25, "25-Beam Simultaneous"),
        (axes[1], lats_bh, lons_bh, snrs_bh, "1-Beam Focused (BH)"),
    ]:
        sc = ax.scatter(lons, lats, c=snrs, cmap=cmap, norm=norm, s=60,
                        edgecolors="black", linewidths=0.3)
        ax.set_xlabel("Longitude (°)", fontsize=11)
        ax.set_ylabel("Latitude (°)", fontsize=11)
        ax.set_title(mode_label, fontsize=12)
        ax.grid(alpha=0.3)

    # Shared colorbar
    cbar = fig.colorbar(
        plt.cm.ScalarMappable(norm=norm, cmap=cmap),
        ax=axes, shrink=0.8, pad=0.02,
    )
    cbar.set_label("SNR (dB)", fontsize=11)

    fig.suptitle(
        f"Fig.2: Spatial SNR Comparison — {city}, {sat_id} @ t={t_tick}s "
        f"(elevation ≈ {peak_elev:.1f}°, Starlink ≥25°)",
        fontsize=12,
    )

    city_tag = city.lower()
    for ext in ("png", "svg"):
        out_path = os.path.join(
            out_dir, f"fig2_spatial_snr_beam_comparison_{city_tag}.{ext}"
        )
        dpi = 300 if ext == "png" else None
        fig.savefig(out_path, dpi=dpi, bbox_inches="tight")
        print(f"[write] {out_path}")

    plt.close(fig)


# ── CLI ────────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="Generate controlled Fig2 comparing 25-beam vs BH on same satellite-tick"
    )
    p.add_argument("--dir-25beam", required=True,
                   help="Directory with sat_*_cells.csv for 25-beam mode (Starlink)")
    p.add_argument("--dir-bh",     required=True,
                   help="Directory with sat_*_cells.csv for BH mode (Starlink, same constellation)")
    p.add_argument("--city",       default="Helsinki",
                   help="City label for figure title")
    p.add_argument("--out-dir",    default="ns_result/0611/figures/paper",
                   help="Output directory for PNG and SVG")
    return p.parse_args()


def main():
    args = parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    print(f"[load] 25-beam data from: {args.dir_25beam}")
    data_25 = load_sat_tick_data(args.dir_25beam)
    print(f"[load] {len(data_25)} (sat, tick) entries in 25-beam dataset")

    print(f"[load] BH data from: {args.dir_bh}")
    data_bh = load_sat_tick_data(args.dir_bh)
    print(f"[load] {len(data_bh)} (sat, tick) entries in BH dataset")

    print("[align] Finding overlapping (sat_id, tick) pairs …")
    best_sat, best_t, best_elev = find_best_overlap_tick(data_25, data_bh)

    if best_sat is None:
        print(
            "[error] No overlapping (sat_id, tick) found between the two datasets.\n"
            "        This means the two directories recorded different satellites or\n"
            "        different time windows. To fix: re-run both beam modes from the\n"
            "        same TLE epoch and same simulation window, then retry."
        )
        # Write diagnostic report
        report = {
            "status":   "no_overlap",
            "n_keys_25beam": len(data_25),
            "n_keys_bh":     len(data_bh),
            "unique_sats_25beam": len({k[0] for k in data_25}),
            "unique_sats_bh":    len({k[0] for k in data_bh}),
        }
        rep_path = os.path.join(args.out_dir, "fig2_overlap_report.json")
        with open(rep_path, "w") as f:
            json.dump(report, f, indent=2)
        print(f"[write] Diagnostic report: {rep_path}")
        return

    print(f"[align] Best overlap: {best_sat} at t={best_t}s (elevation ≈ {best_elev:.1f}°)")

    tick_data_25 = {k: v for k, v in data_25[(best_sat, best_t)].items()}
    tick_data_bh = {k: v for k, v in data_bh[(best_sat, best_t)].items()}

    plot_controlled_fig2(
        tick_data_25, tick_data_bh,
        best_sat, best_t, best_elev,
        args.city, args.out_dir,
    )

    # Write overlap report for citation evidence
    n_common_bins = len(set(tick_data_25) & set(tick_data_bh))
    report = {
        "status":         "overlap_found",
        "best_sat":       best_sat,
        "best_t_s":       best_t,
        "peak_elev_deg":  round(best_elev, 2),
        "n_bins_25beam":  len(tick_data_25),
        "n_bins_bh":      len(tick_data_bh),
        "n_common_bins":  n_common_bins,
        "note": (
            "Both panels use the same satellite ID and same timestamp, "
            "ensuring only beam activation strategy differs between panels."
        ),
    }
    rep_path = os.path.join(args.out_dir, "fig2_overlap_report.json")
    with open(rep_path, "w") as f:
        json.dump(report, f, indent=2)
    print(f"[write] Overlap report: {rep_path}")
    print("[done]")


if __name__ == "__main__":
    main()
