"""
Starlink Downlink Throughput Analysis
Source: Zenodo dataset (PIMRC'23) - iperf3 JSON files
Goal: Extract throughput time series, detect handover drops, compute statistics
"""

import json
import os
import glob
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

# ── CONFIG ──────────────────────────────────────────────────────────────────
DATA_DIR = r"C:\Users\wenj\AppData\Local\Temp"   # change to your extracted folder
FILE_PATTERN = "**/*iperf3*.json"                 # recursive search
OUTPUT_DIR = r"C:\Users\wenj\Desktop\TriScale-LEO\PanLAB\figures"
DROP_THRESHOLD_MBPS = 10.0   # bps below this = potential handover drop
os.makedirs(OUTPUT_DIR, exist_ok=True)


def parse_iperf3_json(filepath: str) -> dict | None:
    """Parse one iperf3 JSON file and return a dict with time-series data."""
    with open(filepath, "r") as f:
        data = json.load(f)

    # extract per-interval receiver-side throughput
    times, throughputs = [], []
    for interval in data.get("intervals", []):
        s = interval["sum"]
        t_mid = (s["start"] + s["end"]) / 2          # midpoint timestamp (s)
        bps = s["bits_per_second"]
        times.append(t_mid)
        throughputs.append(bps / 1e6)                 # convert to Mbps

    if not times:
        return None

    end_info = data.get("end", {})
    summary = {
        "filepath":    filepath,
        "duration_s":  times[-1],
        "mean_Mbps":   float(np.mean(throughputs)),
        "median_Mbps": float(np.median(throughputs)),
        "std_Mbps":    float(np.std(throughputs)),
        "p5_Mbps":     float(np.percentile(throughputs, 5)),
        "p95_Mbps":    float(np.percentile(throughputs, 95)),
        "min_Mbps":    float(np.min(throughputs)),
        "max_Mbps":    float(np.max(throughputs)),
        "retransmits": end_info.get("sum_sent", {}).get("retransmits", None),
        "times":       times,
        "throughputs": throughputs,
    }
    return summary


def detect_drops(times: list, throughputs: list, threshold: float = DROP_THRESHOLD_MBPS):
    """Find intervals where throughput drops below threshold after being above it."""
    drops = []
    was_high = False
    for t, tp in zip(times, throughputs):
        if tp >= threshold:
            was_high = True
        elif was_high and tp < threshold:
            drops.append(t)
            was_high = False
    return drops


def plot_timeseries(result: dict, ax: plt.Axes, label: str = ""):
    times = result["times"]
    tps   = result["throughputs"]
    drops = detect_drops(times, tps)

    ax.plot(times, tps, linewidth=0.8, alpha=0.8, label=label or os.path.basename(result["filepath"]))
    ax.axhline(result["mean_Mbps"], linestyle="--", color="red", linewidth=1.2,
               label=f"Mean {result['mean_Mbps']:.1f} Mbps")

    # mark drop events
    for d in drops:
        ax.axvline(d, color="orange", alpha=0.6, linewidth=1.0)

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Downlink Throughput (Mbps)")
    ax.set_title(f"Starlink Downlink Throughput — {os.path.basename(result['filepath'])}\n"
                 f"Mean={result['mean_Mbps']:.1f} | Std={result['std_Mbps']:.1f} | "
                 f"Retransmits={result['retransmits']} | Drops≈{len(drops)}")
    ax.legend(fontsize=8)
    ax.yaxis.set_major_formatter(ticker.FormatStrFormatter("%.0f"))
    ax.grid(True, alpha=0.3)

    # annotate drop intervals
    if drops:
        diffs = [drops[i+1] - drops[i] for i in range(len(drops)-1)]
        if diffs:
            print(f"  Drop intervals: {[f'{d:.1f}s' for d in diffs]}")
            print(f"  Mean inter-drop interval: {np.mean(diffs):.1f}s")


def print_summary(result: dict):
    fname = os.path.basename(result["filepath"])
    print(f"\n{'='*60}")
    print(f"File: {fname}")
    print(f"  Duration:     {result['duration_s']:.1f} s")
    print(f"  Mean:         {result['mean_Mbps']:.2f} Mbps")
    print(f"  Median:       {result['median_Mbps']:.2f} Mbps")
    print(f"  Std dev:      {result['std_Mbps']:.2f} Mbps")
    print(f"  P5 / P95:     {result['p5_Mbps']:.2f} / {result['p95_Mbps']:.2f} Mbps")
    print(f"  Min / Max:    {result['min_Mbps']:.2f} / {result['max_Mbps']:.2f} Mbps")
    print(f"  Retransmits:  {result['retransmits']}")


def main():
    # ── find all JSON files ────────────────────────────────────────────────
    pattern = os.path.join(DATA_DIR, FILE_PATTERN)
    files = glob.glob(pattern, recursive=True)

    if not files:
        # fallback: let user specify a single file
        print(f"No files found in {DATA_DIR}. Using example file...")
        files = []

    if not files:
        print("ERROR: No iperf3 JSON files found.")
        print(f"Set DATA_DIR to the folder containing your extracted iperf3 JSON files.")
        return

    print(f"Found {len(files)} iperf3 JSON file(s).")
    results = []
    for fp in files[:10]:   # limit to first 10 for now
        print(f"  Parsing: {os.path.basename(fp)}")
        r = parse_iperf3_json(fp)
        if r:
            results.append(r)

    if not results:
        print("No valid results parsed.")
        return

    # ── per-file time-series plots ─────────────────────────────────────────
    for r in results:
        print_summary(r)
        drops = detect_drops(r["times"], r["throughputs"])
        print(f"  Throughput drops (<{DROP_THRESHOLD_MBPS} Mbps): {len(drops)}")
        if len(drops) > 1:
            diffs = [drops[i+1] - drops[i] for i in range(len(drops)-1)]
            print(f"  Inter-drop intervals: {[f'{d:.1f}s' for d in diffs]}")
            print(f"  Mean inter-drop: {np.mean(diffs):.1f}s  (expected ~15s for handover)")

        fig, ax = plt.subplots(figsize=(14, 4))
        plot_timeseries(r, ax)
        plt.tight_layout()
        safe_name = os.path.splitext(os.path.basename(r["filepath"]))[0]
        out_path = os.path.join(OUTPUT_DIR, f"throughput_{safe_name}.png")
        plt.savefig(out_path, dpi=150)
        print(f"  Saved → {out_path}")
        plt.close()

    # ── aggregate statistics across all files ─────────────────────────────
    if len(results) > 1:
        all_means = [r["mean_Mbps"] for r in results]
        print(f"\n{'='*60}")
        print(f"AGGREGATE ({len(results)} files):")
        print(f"  Grand mean:   {np.mean(all_means):.2f} Mbps")
        print(f"  Grand std:    {np.std(all_means):.2f} Mbps")
        print(f"  Range:        {np.min(all_means):.2f} – {np.max(all_means):.2f} Mbps")


if __name__ == "__main__":
    main()
