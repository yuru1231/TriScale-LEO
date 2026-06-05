"""
analysis/handover_analysis.py
------------------------------
Analyses handover behaviour across the Iridium-NEXT constellation
pass over Tokyo ROI.

Input  : results/  (or --out-dir)
         sat_XXXXX_cells.csv  — per-satellite per-cell SNR time series
         status.json          — pass metadata
Output : results/handover_events.csv
         results/figures/fig_serving_sat.png
         results/figures/fig_snr_timeline_cell<N>.png
         results/figures/fig_handover_count.png

Usage
-----
cd 2D/code/orbit-sgp4/analysis
python handover_analysis.py [--out-dir ../results] [--hysteresis 0] [--snr-min -100]
"""

import argparse
import json
import os
import sys

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from link_budget import mrc_combine_snr_db

# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    default_results = os.path.normpath(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ns_result")
    )
    p = argparse.ArgumentParser()
    p.add_argument("--out-dir",    default=default_results,
                   help=f"Directory containing CSVs and status.json "
                        f"(default: {default_results})")
    p.add_argument("--hysteresis", type=float, default=0.0,
                   help="Handover hysteresis margin (dB).")
    p.add_argument("--snr-min",    type=float, default=-100.0,
                   help="Minimum SNR (dB) to consider a satellite usable.")
    return p.parse_args()

# ---------------------------------------------------------------------------
# Load data
# ---------------------------------------------------------------------------

def load_all(out_dir: str, status: dict) -> pd.DataFrame:
    frames = []
    for pass_info in status["passes"]:
        csv_path = os.path.join(out_dir, pass_info["csv_file"])
        if not os.path.exists(csv_path):
            print(f"[WARN] missing {csv_path}")
            continue
        df = pd.read_csv(csv_path)
        df["sat_index"] = pass_info["sat_index"]
        df["sat_name"]  = pass_info["sat_name"]
        frames.append(df)
    if not frames:
        raise RuntimeError("No CSV files found.")
    all_data = pd.concat(frames, ignore_index=True)
    all_data = all_data[np.isfinite(all_data["snr_dB"])].copy()
    all_data["time_s"] = all_data["time_s"].round(0).astype(int)
    return all_data

# ---------------------------------------------------------------------------
# Best-satellite selection
# ---------------------------------------------------------------------------

def select_serving(all_data: pd.DataFrame,
                   hysteresis: float,
                   snr_min: float) -> pd.DataFrame:
    usable   = all_data[all_data["snr_dB"] >= snr_min].copy()
    best_idx = usable.groupby(["time_s", "cell_idx"])["snr_dB"].idxmax()
    best     = usable.loc[best_idx].reset_index(drop=True)
    best     = best[["time_s", "cell_idx", "sat_index", "sat_name",
                      "elevation_deg", "snr_dB", "sinr_dB"]].copy()
    best.sort_values(["cell_idx", "time_s"], inplace=True)

    if hysteresis <= 0:
        return best

    snr_lookup  = (usable.set_index(["time_s", "cell_idx", "sat_index"])["snr_dB"]
                         .to_dict())
    result_rows = []
    for cell_id, grp in best.groupby("cell_idx"):
        grp         = grp.sort_values("time_s").reset_index(drop=True)
        current_sat = None
        current_snr = -np.inf
        for _, row in grp.iterrows():
            t             = row["time_s"]
            candidate_sat = row["sat_index"]
            candidate_snr = row["snr_dB"]
            if current_sat is not None:
                live_snr = snr_lookup.get((t, cell_id, current_sat), None)
                if live_snr is None:
                    current_sat = None
                    current_snr = -np.inf
                else:
                    current_snr = live_snr
            if current_sat is None:
                current_sat = candidate_sat
                current_snr = candidate_snr
            elif current_sat == candidate_sat:
                current_snr = candidate_snr
            elif candidate_snr > current_snr + hysteresis:
                current_sat = candidate_sat
                current_snr = candidate_snr
            result_rows.append({
                "time_s":        t,
                "cell_idx":      cell_id,
                "sat_index":     current_sat,
                "sat_name":      row["sat_name"],
                "elevation_deg": row["elevation_deg"],
                "snr_dB":        current_snr,
                "sinr_dB":       row["sinr_dB"],
            })
    return pd.DataFrame(result_rows)

# ---------------------------------------------------------------------------
# Handover detection
# ---------------------------------------------------------------------------

def detect_handovers(serving: pd.DataFrame) -> pd.DataFrame:
    events = []
    for cell_id, grp in serving.groupby("cell_idx"):
        grp = grp.sort_values("time_s").reset_index(drop=True)
        for i in range(1, len(grp)):
            prev = grp.iloc[i - 1]
            curr = grp.iloc[i]
            if curr["time_s"] - prev["time_s"] > 1:
                continue
            if prev["sat_index"] != curr["sat_index"]:
                events.append({
                    "cell_idx":      cell_id,
                    "time_s":        curr["time_s"],
                    "from_sat":      int(prev["sat_index"]),
                    "to_sat":        int(curr["sat_index"]),
                    "snr_before_dB": prev["snr_dB"],
                    "snr_after_dB":  curr["snr_dB"],
                    "snr_gain_dB":   curr["snr_dB"] - prev["snr_dB"],
                })
    return pd.DataFrame(events)

# ---------------------------------------------------------------------------
# Figures
# ---------------------------------------------------------------------------

def _sat_color_map(sat_indices):
    cmap = plt.colormaps["tab20"].resampled(len(sat_indices))
    return {s: cmap(i) for i, s in enumerate(sorted(sat_indices))}

def fig_serving_satellite(serving, events, fig_dir):
    cells   = sorted(serving["cell_idx"].unique())
    sat_ids = sorted(serving["sat_index"].unique())
    cmap    = _sat_color_map(sat_ids)

    fig, axes = plt.subplots(len(cells), 1,
                             figsize=(14, 0.45 * len(cells) + 1.5),
                             sharex=True)
    if len(cells) == 1:
        axes = [axes]

    for ax, cell_id in zip(axes, cells):
        grp   = serving[serving["cell_idx"] == cell_id].sort_values("time_s")
        times = grp["time_s"].values
        sats  = grp["sat_index"].values
        for j in range(len(times)):
            ax.barh(0, 1, left=times[j] - 0.5, height=1,
                    color=cmap[sats[j]], linewidth=0)
        ho = events[events["cell_idx"] == cell_id]
        ax.vlines(ho["time_s"], -0.5, 0.5, colors="white",
                  linewidth=1.2, zorder=3)
        ax.set_yticks([])
        ax.set_ylabel(f"C{cell_id}", fontsize=7, rotation=0, labelpad=18)
        ax.set_xlim(0, 3600)

    patches = [mpatches.Patch(color=cmap[s], label=f"Sat {s}") for s in sat_ids]
    fig.legend(handles=patches, loc="upper right", fontsize=7,
               ncol=4, framealpha=0.9)
    fig.supxlabel("Time (s)", fontsize=9)
    fig.suptitle("Serving Satellite per Cell\nWhite lines = handover events",
                 fontsize=10)
    plt.tight_layout(rect=[0, 0.03, 1, 0.97])
    path = os.path.join(fig_dir, "fig_serving_sat.png")
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"[fig] {path}")

def fig_snr_timeline(serving, all_data, cell_id, fig_dir):
    sat_ids = sorted(all_data["sat_index"].unique())
    cmap    = _sat_color_map(sat_ids)

    fig, ax = plt.subplots(figsize=(12, 4))
    for sat in sat_ids:
        sub = (all_data[(all_data["cell_idx"] == cell_id) &
                        (all_data["sat_index"] == sat)]
               .sort_values("time_s"))
        if sub.empty:
            continue
        ax.plot(sub["time_s"], sub["snr_dB"],
                color=cmap[sat], linewidth=0.8, alpha=0.6,
                label=f"Sat {sat}")

    best = serving[serving["cell_idx"] == cell_id].sort_values("time_s")
    ax.plot(best["time_s"], best["snr_dB"],
            color="black", linewidth=1.8, label="Greedy (best sat)")

    cell_data = all_data[all_data["cell_idx"] == cell_id]
    mrc_times, mrc_snr = [], []
    for t, grp in cell_data.groupby("time_s"):
        snr_list = grp["snr_dB"].dropna().tolist()
        if snr_list:
            mrc_times.append(t)
            mrc_snr.append(mrc_combine_snr_db(snr_list))
    if mrc_times:
        ax.plot(mrc_times, mrc_snr,
                color="red", linewidth=1.5, linestyle="--",
                label="MRC combined")

    ax.axhline(0, color="gray", linestyle="--", linewidth=0.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("SNR (dB)")
    ax.set_title(f"SNR Timeline — Cell {cell_id}\n"
                 "Red dashed = MRC combined, Black = greedy best")
    ax.legend(fontsize=7, ncol=4)
    ax.set_xlim(0, 3600)
    plt.tight_layout()
    path = os.path.join(fig_dir, f"fig_snr_timeline_cell{cell_id}.png")
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"[fig] {path}")

def fig_handover_count(events, n_cells, fig_dir):
    counts = (events.groupby("cell_idx").size()
              .reindex(range(n_cells), fill_value=0))
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.bar(counts.index, counts.values, color="steelblue", edgecolor="white")
    ax.set_xlabel("Cell index")
    ax.set_ylabel("Number of handovers")
    ax.set_title("Handover Count per Cell")
    ax.set_xticks(range(n_cells))
    plt.tight_layout()
    path = os.path.join(fig_dir, "fig_handover_count.png")
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"[fig] {path}")

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    args    = parse_args()
    out_dir = os.path.abspath(args.out_dir)
    fig_dir = os.path.join(out_dir, "figures")
    os.makedirs(fig_dir, exist_ok=True)

    status_path = os.path.join(out_dir, "constellation_status.json")
    with open(status_path) as f:
        status = json.load(f)
    print(f"[info] {len(status['passes'])} qualifying satellites")

    all_data = load_all(out_dir, status)
    print(f"[info] {len(all_data):,} rows loaded  "
          f"(cells: {all_data['cell_idx'].nunique()}, "
          f"sats: {all_data['sat_index'].nunique()})")

    serving = select_serving(all_data, args.hysteresis, args.snr_min)
    events  = detect_handovers(serving)

    print(f"\n=== Handover Summary ===")
    print(f"Total events : {len(events)}")
    if not events.empty:
        print(f"Cells with HO: {events['cell_idx'].nunique()} / "
              f"{serving['cell_idx'].nunique()}")
        print(f"Mean HO/cell : {len(events)/serving['cell_idx'].nunique():.1f}")

    events_path = os.path.join(out_dir, "handover_events.csv")
    events.to_csv(events_path, index=False)
    print(f"\n[saved] {events_path}")

    n_cells    = serving["cell_idx"].nunique()
    total_time = status["window_s"]
    coverage   = serving.groupby("cell_idx")["time_s"].count()
    print(f"\n=== Coverage ===")
    print(f"Window         : {total_time:.0f} s")
    print(f"Mean covered   : {coverage.mean():.0f} s / cell  "
          f"({100*coverage.mean()/total_time:.1f}%)")

    print("\n[info] generating figures ...")
    fig_serving_satellite(serving, events, fig_dir)
    centre_cell = int(serving["cell_idx"].median())
    fig_snr_timeline(serving, all_data, centre_cell, fig_dir)
    fig_handover_count(events, n_cells, fig_dir)
    print("\nDone.")

if __name__ == "__main__":
    main()
