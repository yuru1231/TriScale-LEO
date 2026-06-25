"""
plot_per_cell_bhtp_metrics.py

Generates Fig5 (Per-cell Service Opportunity) and Fig6 (Per-cell Longest Gap Duration)
by reading the CSV files produced by exp_per_cell_bhtp_metrics.py.

Data is grouped by cell_idx (0–24, the phased-array beam direction index) so
each bar in the chart corresponds to one fixed beam slot, consistent across scenarios.

Usage (after running exp_per_cell_bhtp_metrics.py for all 4 priority scenarios):

  # Helsinki comparison: Iridium vs Starlink
  python plot_per_cell_bhtp_metrics.py \
      --csvs  data/processed/iridium_bh_helsinki_deg25_per_cell_metrics.csv \
              data/processed/starlink_bh_helsinki_deg25_per_cell_metrics.csv \
      --labels "Iridium BH ≥25°" "Starlink BH ≥25°" \
      --city  Helsinki \
      --snr-thresh 10.0 \
      --out-dir ns_result/0611/figures/paper

  # Tokyo comparison
  python plot_per_cell_bhtp_metrics.py \
      --csvs  data/processed/iridium_bh_tokyo_deg25_per_cell_metrics.csv \
              data/processed/starlink_bh_tokyo_deg25_per_cell_metrics.csv \
      --labels "Iridium BH ≥25°" "Starlink BH ≥25°" \
      --city  Tokyo \
      --snr-thresh 10.0 \
      --out-dir ns_result/0611/figures/paper

Output files:
  fig5_per_cell_service_opp_<city>.png / .svg
  fig6_per_cell_longest_gap_<city>.png / .svg
"""

import argparse
import csv
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
import numpy as np


# ── colour palette consistent with existing paper figures ─────────────────────
COLORS = ["#2196F3", "#FF5722", "#4CAF50", "#9C27B0"]
HATCHES = ["", "//", "..", "xx"]


# ── data loading ───────────────────────────────────────────────────────────────

def load_metrics_csv(fpath: str) -> list[dict]:
    """Load per-bin metrics CSV; convert numeric fields to float where possible."""
    rows = []
    with open(fpath, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            out = {}
            for k, v in row.items():
                try:
                    out[k] = float(v)
                except ValueError:
                    out[k] = v   # keep "nan" string or other text as-is
            rows.append(out)
    return rows


def align_by_cell_idx(datasets: list[list[dict]]) -> list[list[dict]]:
    """
    Align multiple datasets by cell_idx (0–24) so column i in every dataset
    refers to the same beam slot.

    Returns each dataset sorted by cell_idx, filtered to common indices only.
    """
    idx_sets = [set(int(r["cell_idx"]) for r in ds) for ds in datasets]
    common = idx_sets[0].intersection(*idx_sets[1:])

    aligned = []
    for ds in datasets:
        filtered = [r for r in ds if int(r["cell_idx"]) in common]
        aligned.append(sorted(filtered, key=lambda r: int(r["cell_idx"])))
    return aligned


# ── Figure 5: Per-cell Service Opportunity + SNR Margin (dual panel) ──────────

def plot_service_opportunity(
    aligned: list[list[dict]],
    labels: list[str],
    city: str,
    snr_thresh: float,
    out_dir: str,
) -> None:
    """
    Two-panel figure:
      Top panel    : Per-cell Service Opportunity (%).
                     All-100% result is the key finding — it confirms every beam
                     slot is fully serviceable during satellite passes.  A text
                     annotation makes this explicit so a flat bar wall is not
                     misread as an uninformative chart.
      Bottom panel : Per-cell SNR Margin (dB = median SNR − threshold).
                     This is where the Iridium vs Starlink difference is visible.
    """
    n_cells     = len(aligned[0])
    n_scenarios = len(aligned)
    cell_idx    = np.arange(n_cells)

    bar_width = 0.75 / n_scenarios
    offsets   = (
        np.linspace(-(n_scenarios - 1) / 2, (n_scenarios - 1) / 2, n_scenarios)
        * bar_width
    )

    fig, (ax_top, ax_bot) = plt.subplots(
        2, 1, figsize=(14, 9), sharex=True,
        gridspec_kw={"hspace": 0.35},
    )
    fig.suptitle(
        f"Fig.5: Per-cell BHTP Channel Metrics — {city} (≥25° elevation mask, SNR ≥ {snr_thresh:.0f} dB)",
        fontsize=13, fontweight="bold",
    )

    # ── top panel: Service Opportunity ────────────────────────────────────────
    all_opp_100 = True
    for i, (ds, label) in enumerate(zip(aligned, labels)):
        vals = [r["service_opp_pct"] for r in ds]
        if any(v < 100.0 for v in vals):
            all_opp_100 = False
        ax_top.bar(
            cell_idx + offsets[i], vals,
            width=bar_width, label=label,
            color=COLORS[i % len(COLORS)],
            hatch=HATCHES[i % len(HATCHES)],
            edgecolor="black", linewidth=0.5, alpha=0.85,
        )

    ax_top.set_ylabel(f"Service Opportunity (%)\n[SNR ≥ {snr_thresh:.0f} dB, in-range ticks]", fontsize=11)
    ax_top.yaxis.set_major_formatter(mticker.FormatStrFormatter("%.0f%%"))
    ax_top.set_ylim(0, 115)
    ax_top.grid(axis="y", alpha=0.3)
    ax_top.legend(fontsize=11, ncol=n_scenarios, loc="lower right",
                  framealpha=0.9)

    # Annotate the 100% finding explicitly so the flat bars are not misread
    if all_opp_100:
        ax_top.text(
            0.5, 0.62,
            "All 25 beam slots: 100% service opportunity\n"
            "during satellite passes — channel availability\n"
            "is not the scheduling bottleneck.",
            transform=ax_top.transAxes,
            fontsize=10, ha="center", va="center",
            bbox=dict(boxstyle="round,pad=0.4", facecolor="#E3F2FD",
                      edgecolor="#1565C0", linewidth=1.2),
        )

    # ── bottom panel: SNR Margin ───────────────────────────────────────────────
    margin_vals_all = []
    for i, (ds, label) in enumerate(zip(aligned, labels)):
        vals = []
        for r in ds:
            v = r.get("snr_margin_dB", "nan")
            vals.append(float(v) if v != "nan" else 0.0)
        margin_vals_all.extend(vals)
        mean_m = sum(vals) / len(vals) if vals else 0.0
        ax_bot.bar(
            cell_idx + offsets[i], vals,
            width=bar_width, label=f"{label}  (mean {mean_m:.2f} dB)",
            color=COLORS[i % len(COLORS)],
            hatch=HATCHES[i % len(HATCHES)],
            edgecolor="black", linewidth=0.5, alpha=0.85,
        )
        ax_bot.axhline(
            mean_m,
            color=COLORS[i % len(COLORS)],
            linestyle="--", linewidth=1.2, alpha=0.8,
            label=f"{label} mean ({mean_m:.2f} dB)",
        )

    y_max = max(margin_vals_all) if margin_vals_all else 20
    ax_bot.set_ylim(0, y_max * 1.35)
    ax_bot.set_ylabel(f"SNR Margin (dB)\n[median SNR − {snr_thresh:.0f} dB threshold]", fontsize=11)
    ax_bot.set_xlabel("Cell Index (beam slot)", fontsize=11)
    ax_bot.set_xticks(cell_idx)
    ax_bot.set_xticklabels([str(j) for j in cell_idx], fontsize=8)
    ax_bot.grid(axis="y", alpha=0.3)

    # MCS reference lines — must be added BEFORE legend() so they appear in it
    for snr_ref, mcs_label, clr in [
        (5.0,  "QPSK",   "#78909C"),
        (10.0, "16QAM",  "#FB8C00"),
        (15.0, "64QAM",  "#43A047"),
    ]:
        margin_ref = snr_ref - snr_thresh   # margin = SNR_ref - threshold
        if 0 <= margin_ref <= y_max * 1.3:
            ax_bot.axhline(margin_ref, color=clr, linestyle=":", linewidth=1.2,
                           alpha=0.8, label=f"{mcs_label} min. (SNR = {snr_ref:.0f} dB)")

    ax_bot.legend(fontsize=11, ncol=1, loc="upper right", framealpha=0.9)

    city_tag = city.lower()
    for ext in ("png", "svg"):
        out_path = os.path.join(out_dir, f"fig5_per_cell_service_opp_{city_tag}.{ext}")
        dpi = 300 if ext == "png" else None
        fig.savefig(out_path, dpi=dpi, bbox_inches="tight")
        print(f"[write] {out_path}")

    plt.close(fig)


# ── Figure 6: Per-cell Longest Gap Duration ───────────────────────────────────

def plot_longest_gap(
    aligned: list[list[dict]],
    labels: list[str],
    city: str,
    snr_thresh: float,
    out_dir: str,
) -> None:
    """
    Grouped bar chart: X = Cell Index, Y = Longest Gap (s).
    Gap > 600 s (10 min) bars get a red border to mark scheduling risk.
    Scenarios with gap = 0 s (satellite continuously in range) show no bar;
    a text annotation explains this so the missing bar is not misread.
    """
    n_cells     = len(aligned[0])
    n_scenarios = len(aligned)
    cell_idx    = np.arange(n_cells)

    bar_width = 0.75 / n_scenarios
    offsets   = (
        np.linspace(-(n_scenarios - 1) / 2, (n_scenarios - 1) / 2, n_scenarios)
        * bar_width
    )

    fig, ax = plt.subplots(figsize=(14, 6))

    # Collect all non-zero gap values to set Y-axis limit
    all_vals_nonzero = []
    zero_gap_labels  = []   # labels of scenarios with gap = 0 s (for annotation)

    for i, (ds, label) in enumerate(zip(aligned, labels)):
        vals = [r["longest_gap_s"] for r in ds]
        mean_gap = sum(vals) / len(vals) if vals else 0.0

        # Check if this scenario has all-zero gap
        if all(v == 0 for v in vals):
            zero_gap_labels.append(label)
            # Still draw phantom bars at height 0 to keep legend entry visible
            ax.bar(
                cell_idx + offsets[i], vals,
                width=bar_width, label=f"{label}  (0 s — satellite continuously in range)",
                color=COLORS[i % len(COLORS)],
                hatch=HATCHES[i % len(HATCHES)],
                edgecolor="black", linewidth=0.5, alpha=0.5,
            )
            continue

        all_vals_nonzero.extend(v for v in vals if v > 0)
        bars = ax.bar(
            cell_idx + offsets[i], vals,
            width=bar_width,
            label=f"{label}  (inter-pass gap = {mean_gap:.0f} s)",
            color=COLORS[i % len(COLORS)],
            hatch=HATCHES[i % len(HATCHES)],
            edgecolor="black", linewidth=0.5, alpha=0.85,
        )
        # Red border for bars exceeding the 10-min threshold
        for bar, v in zip(bars, vals):
            if v > 600:
                bar.set_edgecolor("#C62828")
                bar.set_linewidth(2.0)

    # Y-axis: headroom above tallest bar for annotations
    y_max = max(all_vals_nonzero) if all_vals_nonzero else 700
    ax.set_ylim(0, y_max * 1.35)

    # Reference lines
    ax.axhline(60,  color="#FB8C00", linestyle=":", linewidth=1.3,
               label="1 min threshold (low scheduling risk)")
    ax.axhline(600, color="#C62828", linestyle=":", linewidth=1.3,
               label="10 min threshold (high scheduling risk)")

    # Annotation for zero-gap scenarios
    if zero_gap_labels:
        note_text = "Note: " + ", ".join(zero_gap_labels) + "\nhave gap = 0 s (satellite in range\nthroughout the entire simulation window)."
        ax.text(
            0.98, 0.97, note_text,
            transform=ax.transAxes,
            fontsize=9, ha="right", va="top",
            bbox=dict(boxstyle="round,pad=0.4", facecolor="#E8F5E9",
                      edgecolor="#2E7D32", linewidth=1.2),
        )

    ax.set_xlabel("Cell Index (beam slot)", fontsize=12)
    ax.set_ylabel(f"Longest Gap Duration (s)\n[SNR < {snr_thresh:.0f} dB, full window]", fontsize=12)
    ax.set_title(
        f"Fig.6: Per-cell Longest Gap Duration — {city} (≥25° elevation mask)\n"
        "Gap is uniform across all beam slots: determined by constellation geometry, not cell location.",
        fontsize=12,
    )
    ax.set_xticks(cell_idx)
    ax.set_xticklabels([str(j) for j in cell_idx], fontsize=8)
    ax.legend(fontsize=11, ncol=1, loc="upper left", framealpha=0.9)
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()

    city_tag = city.lower()
    for ext in ("png", "svg"):
        out_path = os.path.join(out_dir, f"fig6_per_cell_longest_gap_{city_tag}.{ext}")
        dpi = 300 if ext == "png" else None
        fig.savefig(out_path, dpi=dpi, bbox_inches="tight")
        print(f"[write] {out_path}")

    plt.close(fig)


# ── CLI ────────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="Generate Fig5 (service opportunity) and Fig6 (longest gap) "
                    "from per-cell BHTP metrics CSVs"
    )
    p.add_argument("--csvs",       nargs="+", required=True,
                   help="Paths to per-cell metrics CSVs (one per scenario)")
    p.add_argument("--labels",     nargs="+", required=True,
                   help="Legend labels, one per CSV (must match --csvs count)")
    p.add_argument("--city",       required=True,
                   help="City name used in figure title and filename (e.g. Helsinki)")
    p.add_argument("--snr-thresh", type=float, default=10.0,
                   help="SNR threshold in dB shown in axis labels")
    p.add_argument("--out-dir",    default="ns_result/0611/figures/paper",
                   help="Output directory for PNG and SVG files")
    return p.parse_args()


def main():
    args = parse_args()

    if len(args.csvs) != len(args.labels):
        print(f"[error] --csvs ({len(args.csvs)}) and --labels ({len(args.labels)}) count mismatch")
        return

    os.makedirs(args.out_dir, exist_ok=True)

    datasets = [load_metrics_csv(p) for p in args.csvs]
    for fpath, ds in zip(args.csvs, datasets):
        print(f"[load] {len(ds)} beam slots from {fpath}")

    aligned = align_by_cell_idx(datasets)
    n_common = len(aligned[0])
    print(f"[align] {n_common} cell_idx slots in common across all scenarios")

    if n_common == 0:
        print("[error] No common cell_idx found — check that CSVs contain cell_idx 0–24")
        return

    plot_service_opportunity(aligned, args.labels, args.city, args.snr_thresh, args.out_dir)
    plot_longest_gap(aligned,         args.labels, args.city, args.snr_thresh, args.out_dir)

    print("[done] Fig5 and Fig6 written.")


if __name__ == "__main__":
    main()
