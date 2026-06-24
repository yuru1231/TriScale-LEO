"""
bh_dwell_analysis.py

Layer 1 Validation: analyse hotspot vs non-hotspot beam service allocation
using dwell_time_ms from bh-metrics_starlink25_*.csv.

Only satId=498 is analysed (the sole satellite with real UTs in starlink25).

Usage:
    python bh_dwell_analysis.py bh-metrics_starlink25.csv
    python bh_dwell_analysis.py bh-metrics_starlink25_20260623_161646.csv
"""

import sys
import pandas as pd
import matplotlib.pyplot as plt

HOTSPOT_BEAMS = {1, 4, 13, 19, 22}
TARGET_SAT    = 498
N_BEAMS       = 25


def main(csv_path: str) -> None:
    df = pd.read_csv(csv_path)

    # ── Filter to the only physically meaningful satellite ────────────────────
    df498 = df[df["sat_id"] == TARGET_SAT].copy()
    if df498.empty:
        print(f"[ERROR] No rows for sat_id={TARGET_SAT} in {csv_path}")
        print("        Check that you are using a starlink25 run and the correct file.")
        return

    df498["beam_type"] = df498["beam_id"].apply(
        lambda b: "hotspot" if b in HOTSPOT_BEAMS else "non-hotspot"
    )

    n_hot  = len(HOTSPOT_BEAMS)
    n_cold = N_BEAMS - n_hot

    # ── Analysis 1: per-beam statistics ──────────────────────────────────────
    beam_stats = (
        df498.groupby(["beam_id", "beam_type"])
             .agg(count      =("dwell_time_ms", "count"),
                  total_dwell=("dwell_time_ms", "sum"),
                  mean_dwell =("dwell_time_ms", "mean"))
             .reset_index()
             .sort_values("total_dwell", ascending=False)
    )

    print("\n=== Per-beam Statistics (sat_id=498) ===")
    print(beam_stats.to_string(index=False))

    # ── Analysis 2: hotspot vs non-hotspot aggregate ──────────────────────────
    summary = (
        df498.groupby("beam_type")
             .agg(total_rows =("dwell_time_ms", "count"),
                  total_dwell=("dwell_time_ms", "sum"),
                  mean_dwell =("dwell_time_ms", "mean"))
             .reset_index()
    )

    print("\n=== Hotspot vs Non-hotspot Summary (sat_id=498) ===")
    print(summary.to_string(index=False))

    hot_rows  = summary[summary["beam_type"] == "hotspot"]
    cold_rows = summary[summary["beam_type"] == "non-hotspot"]

    if not hot_rows.empty and not cold_rows.empty:
        hot  = hot_rows.iloc[0]
        cold = cold_rows.iloc[0]
        if cold["total_rows"] > 0:
            freq_ratio  = (hot["total_rows"]  / n_hot)  / (cold["total_rows"]  / n_cold)
            dwell_ratio = (hot["total_dwell"] / n_hot)  / (cold["total_dwell"] / n_cold)
            print(f"\nService frequency ratio (per beam)  hotspot / non-hotspot = {freq_ratio:.2f}x")
            print(f"Total dwell ratio       (per beam)  hotspot / non-hotspot = {dwell_ratio:.2f}x")
        else:
            print("\nnon-hotspot beam did NOT appear in sat_id=498 metrics")
            print("→ Scheduler allocated ALL capacity to hotspot beams (only starvation forced-in for non-hotspot)")
    elif cold_rows.empty:
        print("\nnon-hotspot beam did NOT appear at all → scheduler fully favours hotspot")

    # ── Analysis 3: hotspot selection rate per T_p ───────────────────────────
    # "current" beam: dwell_time_ms >= 48 ms  (= 6 slots × 8 ms usable)
    # "carry-over":   dwell_time_ms =  8 ms   (tail-end of previous period)
    df498["is_current"] = df498["dwell_time_ms"] >= 48
    current = df498[df498["is_current"]]

    per_period = (
        current.groupby("time_s")
               .agg(total_current  =("beam_id",    "count"),
                    hotspot_current=("beam_type",
                                     lambda x: (x == "hotspot").sum()))
               .reset_index()
    )
    per_period["hotspot_pct"] = (
        per_period["hotspot_current"] / per_period["total_current"] * 100
    )

    print(f"\n=== Per-period hotspot selection rate (sat_id=498) ===")
    print(f"Total periods analysed : {len(per_period)}")
    print(f"Mean hotspot%%          : {per_period['hotspot_pct'].mean():.1f}%%")
    print(f"Random baseline        : {n_hot / N_BEAMS * 100:.1f}%%  (= {n_hot}/{N_BEAMS} beams)")
    print(f"Periods at 100%% hotspot: {(per_period['hotspot_pct'] >= 99).sum()}")
    print(f"Periods with non-hotspot selected: "
          f"{(per_period['hotspot_pct'] < 99).sum()}"
          f"  (starvation forced-in events)")

    # ── Figures ───────────────────────────────────────────────────────────────
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    fig.suptitle(f"BH Phase G — Layer 1 Validation: dwell_time_ms\n"
                 f"sat_id=498, starlink25  |  source: {csv_path.split('/')[-1]}")

    # Figure 1: per-beam total dwell time (horizontal bar)
    colors = [
        "tab:orange" if b in HOTSPOT_BEAMS else "tab:steelblue"
        for b in beam_stats["beam_id"]
    ]
    axes[0].barh(beam_stats["beam_id"].astype(str), beam_stats["total_dwell"],
                 color=colors, edgecolor="white", linewidth=0.5)
    axes[0].set_xlabel("Total Dwell Time (ms)")
    axes[0].set_ylabel("Beam ID")
    axes[0].set_title("Per-beam Total Dwell Time\n(orange = hotspot)")

    hot_mean = beam_stats[beam_stats["beam_type"] == "hotspot"]["total_dwell"].mean()
    axes[0].axvline(hot_mean, color="tab:orange", linestyle="--",
                    linewidth=1.5, label=f"hotspot mean = {hot_mean:.0f} ms")
    cold_stats = beam_stats[beam_stats["beam_type"] == "non-hotspot"]
    if not cold_stats.empty:
        cold_mean = cold_stats["total_dwell"].mean()
        axes[0].axvline(cold_mean, color="tab:steelblue", linestyle="--",
                        linewidth=1.5, label=f"non-hotspot mean = {cold_mean:.0f} ms")
    axes[0].legend(fontsize=8)

    # Figure 2: hotspot selection % per period
    axes[1].plot(per_period["time_s"], per_period["hotspot_pct"],
                 alpha=0.6, linewidth=0.8, color="tab:orange")
    axes[1].axhline(n_hot / N_BEAMS * 100, color="gray", linestyle="--",
                    label=f"random baseline ({n_hot / N_BEAMS * 100:.0f}%)")
    axes[1].axhline(100, color="tab:orange", linestyle=":",
                    linewidth=1.5, label="100% hotspot")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Hotspot beams in active selection (%)")
    axes[1].set_title("Hotspot Selection Rate per T_p")
    axes[1].set_ylim(-5, 115)
    axes[1].legend(fontsize=8)

    plt.tight_layout()
    out = csv_path.replace(".csv", "_dwell_analysis.png")
    plt.savefig(out, dpi=150)
    print(f"\nFigure saved: {out}")
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python bh_dwell_analysis.py <bh-metrics_starlink25*.csv>")
        sys.exit(1)
    main(sys.argv[1])
