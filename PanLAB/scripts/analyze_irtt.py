"""
Starlink Latency Analysis — IRTT JSON format
Metrics: RTT, one-way delay (UL/DL), jitter (IPDV), packet loss
Usage: python analyze_irtt.py <path_to_irtt_json>
"""

import json
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

NS = 1e6  # nanoseconds → milliseconds


def parse_irtt(filepath: str) -> dict:
    with open(filepath, "r") as f:
        data = json.load(f)

    stats = data["stats"]
    interval_ns = data["config"]["params"]["interval"]   # 10,000,000 ns = 10ms

    # per-packet arrays
    times_s, rtts, send_d, recv_d = [], [], [], []
    ipdv_rtt, ipdv_send, ipdv_recv = [], [], []
    lost_times = []

    for rt in data["round_trips"]:
        t = rt["seqno"] * interval_ns / 1e9     # seconds from start
        if rt["lost"] != "false":
            lost_times.append(t)
            continue

        delay = rt.get("delay", {})
        ipdv  = rt.get("ipdv", {})

        times_s.append(t)
        rtts.append(delay.get("rtt", None))
        send_d.append(delay.get("send", None))
        recv_d.append(delay.get("receive", None))

        if ipdv:
            ipdv_rtt.append(ipdv.get("rtt", None))
            ipdv_send.append(ipdv.get("send", None))
            ipdv_recv.append(ipdv.get("receive", None))
        else:
            ipdv_rtt.append(None)
            ipdv_send.append(None)
            ipdv_recv.append(None)

    # convert ns → ms, filter None
    to_ms = lambda arr: [x / NS for x in arr if x is not None]

    return {
        "filepath":    filepath,
        "interval_ms": interval_ns / 1e6,
        "stats":       stats,
        "times_s":     times_s,
        "rtt_ms":      to_ms(rtts),
        "send_ms":     to_ms(send_d),
        "recv_ms":     to_ms(recv_d),
        "ipdv_rtt_ms": [x / NS for x in ipdv_rtt if x is not None],
        "ipdv_send_ms":[x / NS for x in ipdv_send if x is not None],
        "ipdv_recv_ms":[x / NS for x in ipdv_recv if x is not None],
        "lost_times_s": lost_times,
    }


def print_summary(d: dict):
    st = d["stats"]
    print(f"\n{'='*65}")
    print(f"IRTT Summary")
    print(f"  Packets sent/received: {st['packets_sent']} / {st['packets_received']}")
    print(f"  Packet loss:           {st['packet_loss_percent']:.3f}%")
    print(f"    Upstream loss:       {st['upstream_loss_percent']:.3f}%")
    print(f"    Downstream loss:     {st['downstream_loss_percent']:.4f}%")
    print(f"")
    print(f"  RTT  mean/median/std:  {st['rtt']['mean']/NS:.2f} / {st['rtt']['median']/NS:.2f} / {st['rtt']['stddev']/NS:.2f} ms")
    print(f"  RTT  min/max:          {st['rtt']['min']/NS:.2f} / {st['rtt']['max']/NS:.2f} ms")
    print(f"")
    print(f"  Send (UL) mean/median: {st['send_delay']['mean']/NS:.2f} / {st['send_delay']['median']/NS:.2f} ms")
    print(f"  Recv (DL) mean/median: {st['receive_delay']['mean']/NS:.2f} / {st['receive_delay']['median']/NS:.2f} ms")
    print(f"  UL/DL asymmetry:       +{(st['send_delay']['mean']-st['receive_delay']['mean'])/NS:.2f} ms (UL > DL)")
    print(f"")
    print(f"  IPDV RTT mean/median:  {st['ipdv_round_trip']['mean']/NS:.2f} / {st['ipdv_round_trip']['median']/NS:.2f} ms")
    print(f"  IPDV Send mean/median: {st['ipdv_send']['mean']/NS:.2f} / {st['ipdv_send']['median']/NS:.2f} ms")
    print(f"  IPDV Recv mean/median: {st['ipdv_receive']['mean']/NS:.2f} / {st['ipdv_receive']['median']/NS:.2f} ms")


def detect_spikes(times, values, threshold_ms=60):
    """Find RTT spikes above threshold — likely handover events."""
    spikes = [(t, v) for t, v in zip(times, values) if v > threshold_ms]
    if len(spikes) < 2:
        return spikes
    # cluster spikes within 1s window into single events
    events = [spikes[0]]
    for t, v in spikes[1:]:
        if t - events[-1][0] > 1.0:
            events.append((t, v))
    return events


def plot_irtt(d: dict, outpath: str):
    times = np.array(d["times_s"])
    rtt   = np.array(d["rtt_ms"])
    send  = np.array(d["send_ms"])
    recv  = np.array(d["recv_ms"])

    # align arrays (rtt may have fewer points than times due to None)
    t_full = d["times_s"]
    min_len = min(len(t_full), len(rtt), len(send), len(recv))
    times = np.array(t_full[:min_len])
    rtt   = rtt[:min_len]
    send  = send[:min_len]
    recv  = recv[:min_len]

    spikes = detect_spikes(times.tolist(), rtt.tolist(), threshold_ms=60)
    print(f"\n  RTT spikes (>60ms): {len(spikes)} events")
    if spikes:
        spike_times = [s[0] for s in spikes]
        if len(spike_times) > 1:
            diffs = np.diff(spike_times)
            print(f"  Spike times: {[f'{t:.1f}s' for t in spike_times]}")
            print(f"  Inter-spike intervals: {[f'{d:.1f}s' for d in diffs]}")
            print(f"  Mean inter-spike: {np.mean(diffs):.1f}s  (Starlink handover ~15s)")

    fig = plt.figure(figsize=(16, 12))
    gs  = gridspec.GridSpec(3, 2, figure=fig, hspace=0.45, wspace=0.35)

    # ── 1. RTT time series ─────────────────────────────────────────────────
    ax1 = fig.add_subplot(gs[0, :])
    ax1.plot(times, rtt, linewidth=0.6, color="steelblue", alpha=0.8, label="RTT")
    ax1.plot(times, send, linewidth=0.6, color="tomato",   alpha=0.6, label="Send (UL)")
    ax1.plot(times, recv, linewidth=0.6, color="seagreen", alpha=0.6, label="Recv (DL)")
    ax1.axhline(np.mean(rtt),  color="steelblue", linestyle="--", linewidth=1.2,
                label=f"RTT mean {np.mean(rtt):.1f}ms")
    for t_s, _ in spikes:
        ax1.axvline(t_s, color="orange", alpha=0.5, linewidth=1.2)
    # mark lost packets
    for lt in d["lost_times_s"]:
        ax1.axvline(lt, color="red", alpha=0.15, linewidth=0.8)
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Delay (ms)")
    ax1.set_title("Starlink One-Way & Round-Trip Delay (orange=RTT spike, red=packet loss)")
    ax1.legend(fontsize=8, ncol=4)
    ax1.grid(True, alpha=0.3)

    # ── 2. RTT histogram ──────────────────────────────────────────────────
    ax2 = fig.add_subplot(gs[1, 0])
    ax2.hist(rtt, bins=60, color="steelblue", alpha=0.7, edgecolor="white")
    ax2.axvline(np.mean(rtt),   color="red",   linestyle="--", label=f"Mean {np.mean(rtt):.1f}ms")
    ax2.axvline(np.median(rtt), color="green", linestyle="-.", label=f"Median {np.median(rtt):.1f}ms")
    ax2.set_xlabel("RTT (ms)")
    ax2.set_ylabel("Count")
    ax2.set_title("RTT Distribution")
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)

    # ── 3. UL vs DL histogram ─────────────────────────────────────────────
    ax3 = fig.add_subplot(gs[1, 1])
    ax3.hist(send, bins=50, alpha=0.6, color="tomato",   label=f"UL (send) mean={np.mean(send):.1f}ms")
    ax3.hist(recv, bins=50, alpha=0.6, color="seagreen", label=f"DL (recv) mean={np.mean(recv):.1f}ms")
    ax3.set_xlabel("One-Way Delay (ms)")
    ax3.set_ylabel("Count")
    ax3.set_title("Uplink vs Downlink Delay Distribution")
    ax3.legend(fontsize=8)
    ax3.grid(True, alpha=0.3)

    # ── 4. IPDV (jitter) time series ──────────────────────────────────────
    ax4 = fig.add_subplot(gs[2, 0])
    ipdv_rtt  = np.array(d["ipdv_rtt_ms"])
    ipdv_send = np.array(d["ipdv_send_ms"])
    ipdv_recv = np.array(d["ipdv_recv_ms"])
    t_ipdv = times[1:len(ipdv_rtt)+1] if len(ipdv_rtt) <= len(times) else times[:len(ipdv_rtt)]
    ax4.plot(t_ipdv, ipdv_rtt,  linewidth=0.5, color="steelblue", label="IPDV RTT")
    ax4.plot(t_ipdv, ipdv_send, linewidth=0.5, color="tomato",   label="IPDV UL")
    ax4.plot(t_ipdv, ipdv_recv, linewidth=0.5, color="seagreen", label="IPDV DL")
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("IPDV (ms)")
    ax4.set_title("Inter-Packet Delay Variation (Jitter)")
    ax4.legend(fontsize=8)
    ax4.grid(True, alpha=0.3)

    # ── 5. IPDV histogram ─────────────────────────────────────────────────
    ax5 = fig.add_subplot(gs[2, 1])
    ax5.hist(np.abs(ipdv_rtt),  bins=50, alpha=0.6, color="steelblue",
             label=f"|IPDV RTT| median={np.median(np.abs(ipdv_rtt)):.1f}ms")
    ax5.hist(np.abs(ipdv_send), bins=50, alpha=0.6, color="tomato",
             label=f"|IPDV UL| median={np.median(np.abs(ipdv_send)):.1f}ms")
    ax5.set_xlabel("|IPDV| (ms)")
    ax5.set_ylabel("Count")
    ax5.set_title("Jitter Distribution (absolute IPDV)")
    ax5.legend(fontsize=8)
    ax5.grid(True, alpha=0.3)

    fig.suptitle("Starlink Latency Analysis (IRTT)\n"
                 f"RTT={np.mean(rtt):.1f}±{np.std(rtt):.1f}ms  "
                 f"UL={np.mean(send):.1f}ms  DL={np.mean(recv):.1f}ms  "
                 f"Loss={d['stats']['packet_loss_percent']:.3f}%",
                 fontsize=11, fontweight="bold")

    plt.savefig(outpath, dpi=150, bbox_inches="tight")
    print(f"\n  Saved → {outpath}")
    plt.show()


def main():
    if len(sys.argv) < 2:
        # default to the irtt file in temp
        import glob, os
        pattern = r"C:\Users\wenj\AppData\Local\Temp\**\irtt-*.json"
        files = glob.glob(pattern, recursive=True)
        if not files:
            print("No IRTT JSON files found. Pass filepath as argument.")
            return
        filepath = files[0]
        print(f"Using: {filepath}")
    else:
        filepath = sys.argv[1]

    d = parse_irtt(filepath)
    print_summary(d)

    import os
    outname = os.path.splitext(os.path.basename(filepath))[0] + "_analysis.png"
    outpath = os.path.join(r"C:\Users\wenj\Desktop\TriScale-LEO\PanLAB\figures", outname)
    plot_irtt(d, outpath)


if __name__ == "__main__":
    main()
