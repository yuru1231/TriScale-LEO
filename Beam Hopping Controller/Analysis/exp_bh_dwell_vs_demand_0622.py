"""
exp_bh_dwell_vs_demand_0622.py
------------------------------
Analyse Phase G beam hopping dwell time vs expected traffic demand for the
starlink25 scenario (2026-06-22 run).

Key question: Do the assumed BHTP hotspot beams match the true traffic hotspot
beams?  BuildStaticBhtp treats beams 1..numHotspotBeams(=5) as hotspot.
sat-bh-example.cc wires 600 kbps traffic to beams {1,4,13,19,22} (5x demand).
This script visualises the mismatch.

Usage (from project root on VMware):
    python3 "Beam Hopping Controller/Analysis/exp_bh_dwell_vs_demand_0622.py" \
        --metrics "Beam Hopping Controller/Outputs/bh_dynamic_1584/bh-metrics-starlink25_20260622.csv" \
        --sat_id 498 \
        --output "Beam Hopping Controller/Analysis/fig_bh_dwell_demand_0622.png"
"""

import argparse
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import pandas as pd


# ── Constants ─────────────────────────────────────────────────────────────

# Traffic hotspot beams wired in sat-bh-example.cc (kHotspotBeams)
TRAFFIC_HOTSPOT = {1, 4, 13, 19, 22}        # 600 kbps each
TRAFFIC_DEMAND  = {b: 600.0 for b in TRAFFIC_HOTSPOT}
for b in range(1, 26):
    if b not in TRAFFIC_DEMAND:
        TRAFFIC_DEMAND[b] = 120.0            # non-hotspot: 120 kbps

# BHTP hotspot beams (beams 1..numHotspotBeams = 1..5 in starlink25 config)
BHTP_HOTSPOT = set(range(1, 6))             # beams 1,2,3,4,5

# Label categories for colouring
def beam_category(b):
    """Return colour label for beam b."""
    in_traffic = b in TRAFFIC_HOTSPOT
    in_bhtp    = b in BHTP_HOTSPOT
    if in_traffic and in_bhtp:
        return "Both hotspot"          # beam 1, 4
    elif in_traffic:
        return "Traffic hotspot only"  # beam 13, 19, 22  ← MISMATCH
    elif in_bhtp:
        return "BHTP hotspot only"     # beam 2, 3, 5     ← MISMATCH
    else:
        return "Non-hotspot"           # beam 6..25 except 13,19,22


CATEGORY_COLOR = {
    "Both hotspot":          "#d62728",   # red
    "Traffic hotspot only":  "#ff7f0e",   # orange  (gets 600kbps but low dwell)
    "BHTP hotspot only":     "#1f77b4",   # blue    (gets extra dwell but only 120kbps)
    "Non-hotspot":           "#aec7e8",   # light blue
}


# ── Data loading ──────────────────────────────────────────────────────────

def load_metrics(path: str, sat_id: int) -> pd.DataFrame:
    df = pd.read_csv(path)
    df.columns = [c.strip() for c in df.columns]
    # Filter: active satellite only, after warm-up (time_s > 0)
    df = df[(df["sat_id"] == sat_id) & (df["time_s"] > 0)].copy()
    return df


def per_beam_stats(df: pd.DataFrame) -> pd.DataFrame:
    grp = df.groupby("beam_id").agg(
        avg_dwell_ms    = ("dwell_time_ms",   "mean"),
        avg_throughput  = ("throughput_mbps", "mean"),
        avg_slot_util   = ("slot_util_pct",   "mean"),
        avg_jain        = ("jain_fairness_index", "mean"),
        n_periods       = ("dwell_time_ms",   "count"),
    ).reset_index()
    grp["traffic_demand_kbps"] = grp["beam_id"].map(TRAFFIC_DEMAND)
    grp["category"]            = grp["beam_id"].apply(beam_category)
    return grp.sort_values("beam_id").reset_index(drop=True)


# ── Plotting ──────────────────────────────────────────────────────────────

def plot_dwell_vs_demand(stats: pd.DataFrame, output_path: str, sat_id: int):
    beams  = stats["beam_id"].values
    dwell  = stats["avg_dwell_ms"].values
    demand = stats["traffic_demand_kbps"].values
    cats   = stats["category"].values
    colors = [CATEGORY_COLOR[c] for c in cats]

    fig, axes = plt.subplots(2, 1, figsize=(14, 9),
                             gridspec_kw={"height_ratios": [3, 1.5]})
    fig.suptitle(
        f"Phase G — Dwell Time vs Traffic Demand per Beam\n"
        f"starlink25, sat_id={sat_id}, 2026-06-22 run (no Phase F demand)",
        fontsize=13, fontweight="bold"
    )

    # ── Top panel: dwell time ─────────────────────────────────────────────
    ax1 = axes[0]
    bars = ax1.bar(beams, dwell, color=colors, edgecolor="white", linewidth=0.5)

    # Annotate traffic hotspot beams that are wrongly classified by BHTP
    for i, (b, d, cat) in enumerate(zip(beams, dwell, cats)):
        if cat == "Traffic hotspot only":
            ax1.annotate(
                f"⚠ beam {b}\n600kbps\nbut LARGE",
                xy=(b, d), xytext=(b, d + 12),
                ha="center", fontsize=7.5, color="#ff7f0e",
                arrowprops=dict(arrowstyle="-", color="#ff7f0e", lw=0.8)
            )
        elif cat == "BHTP hotspot only":
            ax1.annotate(
                f"beam {b}\nSMALL but\n120kbps",
                xy=(b, d), xytext=(b, d + 12),
                ha="center", fontsize=7.5, color="#1f77b4",
                arrowprops=dict(arrowstyle="-", color="#1f77b4", lw=0.8)
            )

    ax1.set_ylabel("Average Dwell Time (ms)", fontsize=11)
    ax1.set_xlim(0.3, 25.7)
    ax1.set_xticks(beams)
    ax1.set_xticklabels([str(b) for b in beams], fontsize=8)
    ax1.yaxis.grid(True, linestyle="--", alpha=0.5)
    ax1.set_axisbelow(True)

    # Reference lines
    if len(dwell) > 0:
        ax1.axhline(dwell[dwell > 50].mean() if (dwell > 50).any() else dwell.max(),
                    color="gray", linestyle=":", linewidth=1, label="hotspot mean")

    # Legend
    legend_patches = [
        mpatches.Patch(color=CATEGORY_COLOR[k], label=k)
        for k in CATEGORY_COLOR
    ]
    ax1.legend(handles=legend_patches, loc="upper right", fontsize=9)

    # ── Bottom panel: traffic demand ──────────────────────────────────────
    ax2 = axes[1]
    ax2.bar(beams, demand, color=colors, edgecolor="white", linewidth=0.5)
    ax2.axhline(600, color="#d62728", linestyle="--", linewidth=1, label="600 kbps (hotspot)")
    ax2.axhline(120, color="#aec7e8", linestyle="--", linewidth=1, label="120 kbps (non-hotspot)")
    ax2.set_xlabel("Beam ID", fontsize=11)
    ax2.set_ylabel("Traffic Demand (kbps)", fontsize=11)
    ax2.set_xlim(0.3, 25.7)
    ax2.set_xticks(beams)
    ax2.set_xticklabels([str(b) for b in beams], fontsize=8)
    ax2.set_yticks([0, 120, 300, 600])
    ax2.yaxis.grid(True, linestyle="--", alpha=0.5)
    ax2.set_axisbelow(True)
    ax2.legend(loc="upper right", fontsize=9)

    plt.tight_layout()
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    fig.savefig(output_path, dpi=200, bbox_inches="tight")
    print(f"[saved] {output_path}")
    plt.close(fig)


def plot_mismatch_summary(stats: pd.DataFrame, output_path: str, sat_id: int):
    """Scatter plot: traffic demand (X) vs dwell time (Y) per beam."""
    fig, ax = plt.subplots(figsize=(7, 6))

    for _, row in stats.iterrows():
        ax.scatter(
            row["traffic_demand_kbps"], row["avg_dwell_ms"],
            color=CATEGORY_COLOR[row["category"]],
            s=80, zorder=3
        )
        ax.annotate(
            str(int(row["beam_id"])),
            (row["traffic_demand_kbps"], row["avg_dwell_ms"]),
            textcoords="offset points", xytext=(5, 3),
            fontsize=8
        )

    ax.set_xlabel("Traffic Demand (kbps)", fontsize=11)
    ax.set_ylabel("Average Dwell Time (ms)", fontsize=11)
    ax.set_title(
        f"Demand vs Dwell Alignment — sat_id={sat_id}\n"
        "Ideal: high-demand beams should also have high dwell\n"
        "(Phase G without Phase F: alignment is random)",
        fontsize=11
    )
    ax.grid(True, linestyle="--", alpha=0.4)

    legend_patches = [
        mpatches.Patch(color=CATEGORY_COLOR[k], label=k)
        for k in CATEGORY_COLOR
    ]
    ax.legend(handles=legend_patches, fontsize=9)

    scatter_path = output_path.replace(".png", "_scatter.png")
    fig.savefig(scatter_path, dpi=200, bbox_inches="tight")
    print(f"[saved] {scatter_path}")
    plt.close(fig)


# ── Main ──────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="BH dwell vs demand analysis")
    parser.add_argument("--metrics", required=True,
                        help="Path to bh-metrics CSV")
    parser.add_argument("--sat_id", type=int, default=498,
                        help="Active satellite ID to filter (default: 498)")
    parser.add_argument("--output",  default="fig_bh_dwell_demand_0622.png",
                        help="Output PNG path")
    args = parser.parse_args()

    print(f"[load] {args.metrics}  sat_id={args.sat_id}")
    df = load_metrics(args.metrics, args.sat_id)
    if df.empty:
        print(f"[warn] No rows found for sat_id={args.sat_id}. "
              "Check --sat_id; try: awk -F',' 'NR>1{print $2}' <metrics.csv> | sort -n | uniq")
        return

    stats = per_beam_stats(df)
    print(stats.to_string(index=False))

    # Save per-beam stats as JSON alongside figure
    json_path = args.output.replace(".png", "_stats.json")
    stats_dict = stats.to_dict(orient="records")
    with open(json_path, "w") as f:
        json.dump(stats_dict, f, indent=2)
    print(f"[saved] {json_path}")

    # Mismatch report
    print("\n=== Mismatch Report ===")
    for _, row in stats.iterrows():
        if row["category"] in ("Traffic hotspot only", "BHTP hotspot only"):
            print(f"  beam {int(row['beam_id']):2d}  [{row['category']}]"
                  f"  dwell={row['avg_dwell_ms']:.1f}ms"
                  f"  demand={row['traffic_demand_kbps']:.0f}kbps")

    plot_dwell_vs_demand(stats, args.output, args.sat_id)
    plot_mismatch_summary(stats, args.output, args.sat_id)

    print("\nDone.")


if __name__ == "__main__":
    main()
