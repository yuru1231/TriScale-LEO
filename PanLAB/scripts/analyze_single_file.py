"""
Quick analysis of a single iperf3 JSON file.
Usage: python analyze_single_file.py <path_to_json>

Run this first to verify the script works on the file you already have open.
"""

import json
import sys
import numpy as np
import matplotlib.pyplot as plt

def analyze(filepath: str):
    with open(filepath, "r") as f:
        data = json.load(f)

    # extract intervals
    times, mbps = [], []
    for iv in data["intervals"]:
        s = iv["sum"]
        times.append((s["start"] + s["end"]) / 2)
        mbps.append(s["bits_per_second"] / 1e6)

    times = np.array(times)
    mbps  = np.array(mbps)

    # summary stats
    print(f"File:      {filepath}")
    print(f"Duration:  {times[-1]:.1f} s")
    print(f"Intervals: {len(times)}")
    print(f"Mean:      {np.mean(mbps):.2f} Mbps")
    print(f"Median:    {np.median(mbps):.2f} Mbps")
    print(f"Std:       {np.std(mbps):.2f} Mbps")
    print(f"P5/P95:    {np.percentile(mbps,5):.2f} / {np.percentile(mbps,95):.2f} Mbps")
    print(f"Min/Max:   {np.min(mbps):.2f} / {np.max(mbps):.2f} Mbps")

    end = data.get("end", {})
    retx = end.get("sum_sent", {}).get("retransmits", "N/A")
    print(f"Retransmits: {retx}")

    # detect drops (throughput < 10 Mbps after being >= 10 Mbps)
    threshold = 10.0
    drops = []
    was_high = False
    for t, tp in zip(times, mbps):
        if tp >= threshold:
            was_high = True
        elif was_high and tp < threshold:
            drops.append(t)
            was_high = False

    print(f"\nThroughput drops (<{threshold} Mbps): {len(drops)}")
    if drops:
        print(f"Drop times: {[f'{t:.2f}s' for t in drops]}")
        if len(drops) > 1:
            diffs = np.diff(drops)
            print(f"Inter-drop intervals: {[f'{d:.1f}s' for d in diffs]}")
            print(f"Mean inter-drop: {np.mean(diffs):.1f}s")
            print(f"  (Starlink handover expected every ~15s — check if this matches)")

    # plot
    fig, axes = plt.subplots(2, 1, figsize=(14, 8))

    # top: time series
    ax = axes[0]
    ax.plot(times, mbps, linewidth=0.7, color="steelblue", label="Downlink")
    ax.axhline(np.mean(mbps), color="red", linestyle="--", linewidth=1.2,
               label=f"Mean {np.mean(mbps):.1f} Mbps")
    for d in drops:
        ax.axvline(d, color="orange", alpha=0.7, linewidth=1.2)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Throughput (Mbps)")
    ax.set_title("Starlink Downlink Throughput Time Series")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # bottom: histogram
    ax2 = axes[1]
    ax2.hist(mbps, bins=50, color="steelblue", alpha=0.7, edgecolor="white")
    ax2.axvline(np.mean(mbps), color="red", linestyle="--", linewidth=1.5,
                label=f"Mean {np.mean(mbps):.1f}")
    ax2.axvline(np.median(mbps), color="green", linestyle="-.", linewidth=1.5,
                label=f"Median {np.median(mbps):.1f}")
    ax2.set_xlabel("Throughput (Mbps)")
    ax2.set_ylabel("Count")
    ax2.set_title("Throughput Distribution")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    outpath = filepath.replace(".json", "_analysis.png")
    plt.savefig(outpath, dpi=150)
    print(f"\nSaved plot → {outpath}")
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        # default to the file you have open
        default = r"C:\Users\wenj\AppData\Local\Temp\ee29d3df-a8c9-4584-80fa-5c8585b411fa_data-20230913-20230917.tar.zst.1fa\data\2023-09-13\iperf3-2m-2023-09-13-00-40-00.json"
        analyze(default)
    else:
        analyze(sys.argv[1])
