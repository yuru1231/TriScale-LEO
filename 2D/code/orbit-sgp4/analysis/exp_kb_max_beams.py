"""
analysis/exp_kb_max_beams.py
----------------------------
Experiment: Determine maximum simultaneous beam count (K_b) and validate
            frame configuration for Dynamic BHTP on Starlink-1584 over Tokyo.

Three analyses:
  Part 1 - K_b vs SNR : power budget impact of simultaneous beams
  Part 2 - Interference: Gaussian model pairwise check on fixed hot cells
  Part 3 - Pass windows: satellite handover buffer analysis (C2a gap)
  Part 4 - Frame layout: slot allocation for 70 ms vs 80 ms frames

Usage (Windows CMD)
-------------------
python 2D\\code\\orbit-sgp4\\analysis\\exp_kb_max_beams.py ^
    --altitude_km 511 ^
    --elev_deg 37 ^
    --freq_ghz 30 ^
    --tx_power_w 63 ^
    --antenna_gain_db 60.5 ^
    --n_beams 25 ^
    --noise_figure_db 7.0 ^
    --kb_max 6 ^
    --snr_thresh_db 3.0 ^
    --beam_half_angle_deg 2.0 ^
    --hot_cells 2,10,12,18,21 ^
    --slot_ms 10 ^
    --frame_ms_list 70,80 ^
    --status_json ns_result/0611/25beams/deg37/tokyo_out/constellation_status.json ^
    --output_dir ns_result/bhtp_exp
"""

import argparse
import json
import math
import os
import sys

import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from link_budget import (
    snr_at_elevation,
    critical_elevation,
    DEFAULT_CFG,
)

# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(
        description="K_b experiment: max simultaneous beam count for BHTP (Starlink-1584)"
    )
    # Link budget
    p.add_argument("--altitude_km",         type=float, default=511.0,
                   help="Satellite altitude km [Starlink-1584 SGP4 mean]")
    p.add_argument("--elev_deg",            type=float, default=37.0,
                   help="Minimum elevation angle threshold deg")
    p.add_argument("--freq_ghz",            type=float, default=30.0,
                   help="Center frequency GHz, Ka-band")
    p.add_argument("--tx_power_w",          type=float, default=63.0,
                   help="Total satellite Tx power W")
    p.add_argument("--antenna_gain_db",     type=float, default=60.5,
                   help="UPA peak antenna gain dBi")
    p.add_argument("--n_beams",             type=int,   default=25,
                   help="Number of beams in 5x5 elliptic grid")
    p.add_argument("--noise_figure_db",     type=float, default=7.0,
                   help="Receiver noise figure dB")
    p.add_argument("--snr_thresh_db",       type=float, default=3.0,
                   help="Minimum serviceable SNR threshold dB")
    p.add_argument("--kb_max",              type=int,   default=6,
                   help="Maximum K_b to sweep")
    # Beam geometry
    p.add_argument("--beam_half_angle_deg", type=float, default=2.0,
                   help="Single beam half-angle deg; uniform MIDDLE pattern")
    p.add_argument("--hot_cells",           type=str,   default="2,10,12,18,21",
                   help="Comma-separated fixed hot cell indices (0-indexed, 5x5 grid)")
    # Frame parameters
    p.add_argument("--slot_ms",             type=float, default=10.0,
                   help="Slot duration ms")
    p.add_argument("--frame_ms_list",       type=str,   default="70,80",
                   help="Comma-separated frame durations ms to compare")
    # I/O
    p.add_argument("--status_json",         type=str,   default=None,
                   help="Path to constellation_status.json for pass window analysis")
    p.add_argument("--output_dir",          type=str,   default=None,
                   help="Output directory for tables and figures")
    return p.parse_args()


# ---------------------------------------------------------------------------
# Link budget helpers
# ---------------------------------------------------------------------------

def build_cfg(args):
    """Build link_budget-compatible cfg dict for Starlink parameters."""
    return {
        "h_satellite_m":      args.altitude_km * 1e3,
        "center_freq_hz":     args.freq_ghz * 1e9,
        "transmit_power_w":   args.tx_power_w,
        "antenna_gain_db":    args.antenna_gain_db,
        "n_beams":            args.n_beams,
        "noise_figure_db":    args.noise_figure_db,
        "atm_loss_zenith_db": 0.4,
        "bandwidth_hz":       25e6,
        "temperature_k":      300.0,
        "r_earth_m":          6_371e3,
        "speed_of_light_ms":  299_792_458.0,
        "boltzmann":          1.3806485e-23,
    }


# ---------------------------------------------------------------------------
# Part 1: K_b vs SNR
# ---------------------------------------------------------------------------

def compute_kb_snr(args, cfg):
    """
    For K_b = 1..kb_max:
      - Power per beam = tx_power_w / K_b  →  SNR penalty = 10*log10(K_b) dB
      - Effective SNR = base_SNR - power_penalty
      - Critical elevation = min elevation at which SINR >= snr_thresh with K_b beams
    """
    snr_base = snr_at_elevation(args.elev_deg, cfg)   # full power, 1 beam
    rows = []
    for kb in range(1, args.kb_max + 1):
        penalty_db   = 10.0 * math.log10(kb)
        snr_net      = snr_base - penalty_db
        # min elevation needed: raise thresh by penalty and solve
        eff_thresh   = args.snr_thresh_db + penalty_db
        crit_cfg     = dict(cfg)
        crit_elev    = critical_elevation(crit_cfg, snr_thresh_db=eff_thresh)
        rows.append({
            "kb":               kb,
            "power_per_beam_w": args.tx_power_w / kb,
            "penalty_dB":       penalty_db,
            "snr_at_min_elev":  snr_net,
            "crit_elev_deg":    crit_elev,
            "ok":               snr_net >= args.snr_thresh_db,
        })
    return rows, snr_base


def snr_vs_elev_for_kb(args, cfg, kb_list, elev_range=None):
    """SNR curves vs elevation for selected K_b values, for plotting."""
    if elev_range is None:
        elev_range = np.linspace(args.elev_deg, 90, 200)
    curves = {}
    for kb in kb_list:
        penalty = 10.0 * math.log10(kb)
        curves[kb] = [snr_at_elevation(e, cfg) - penalty for e in elev_range]
    return elev_range, curves


# ---------------------------------------------------------------------------
# Part 2: Inter-beam interference
# ---------------------------------------------------------------------------

def cell_rc(idx, n_side=5):
    """(row, col) from linear cell index."""
    return divmod(idx, n_side)


def beam_pair_distance_km(ci, cj, beam_radius_km, n_side=5):
    """Euclidean distance between two beam centres (km). Cell pitch = 2 * r_beam."""
    ri, ci_ = cell_rc(ci, n_side)
    rj, cj_ = cell_rc(cj, n_side)
    pitch = 2.0 * beam_radius_km
    return math.sqrt(((ci_ - cj_) * pitch) ** 2 + ((ri - rj) * pitch) ** 2)


def gaussian_omega(d_km, r_beam_km):
    """
    Gaussian beam overlap weight (BH scheduler model, sat-bh-scheduler.cc):
        ω = exp(-2.77 * (d / r_beam)^2)
    ω = 1.0 at d=0 (same beam), drops to 0.5 at d=r_beam (3 dB point).
    """
    return math.exp(-2.77 * (d_km / r_beam_km) ** 2)


def compute_interference(hot_cells, beam_radius_km, n_side=5):
    """
    Pairwise Gaussian interference ω_{i,j} for all hot-cell pairs.
    Returns rows sorted by ascending distance (worst interference first).
    """
    rows = []
    for i, ci in enumerate(hot_cells):
        for cj in hot_cells[i + 1:]:
            d    = beam_pair_distance_km(ci, cj, beam_radius_km, n_side)
            omega = gaussian_omega(d, beam_radius_km)
            rows.append({
                "pair":     f"C{ci}↔C{cj}",
                "cell_i":   ci,
                "cell_j":   cj,
                "dist_km":  d,
                "omega":    omega,
                "omega_dB": 10.0 * math.log10(max(omega, 1e-30)),
                "verdict":  "negligible" if omega < 1e-2 else "significant",
            })
    return sorted(rows, key=lambda r: r["dist_km"])


# ---------------------------------------------------------------------------
# Part 3: Pass window statistics
# ---------------------------------------------------------------------------

def load_passes(status_path):
    with open(status_path) as f:
        status = json.load(f)
    passes = sorted([
        {
            "sat_idx":   p["sat_index"],
            "start_s":   p["window_start_s"],
            "end_s":     p["window_end_s"],
            "dur_s":     p["window_end_s"] - p["window_start_s"],
            "peak_elev": p["peak_elev_deg"],
        }
        for p in status["passes"]
    ], key=lambda x: x["start_s"])
    return passes, status


def compute_overlaps(passes):
    """
    For each consecutive satellite pair, overlap = end_i - start_{i+1}.
    Positive = simultaneous coverage (handover buffer available).
    Negative = gap (should not occur for Starlink at 37°).
    """
    overlaps = []
    for i in range(len(passes) - 1):
        a, b = passes[i], passes[i + 1]
        overlap = a["end_s"] - b["start_s"]
        overlaps.append({
            "from_sat":  a["sat_idx"],
            "to_sat":    b["sat_idx"],
            "overlap_s": overlap,
            "is_gap":    overlap < 0,
        })
    return overlaps


def pass_stats(passes, overlaps, slot_ms, frame_ms_list):
    durs = [p["dur_s"] for p in passes]
    ovl  = [o["overlap_s"] for o in overlaps]
    gaps = [o for o in overlaps if o["is_gap"]]
    siml = [o for o in overlaps if not o["is_gap"]]

    worst_ovl_s = min([o["overlap_s"] for o in siml], default=0.0)
    worst_ovl_ms = worst_ovl_s * 1000.0

    stats = {
        "n_passes":      len(passes),
        "dur_min_s":     min(durs),
        "dur_mean_s":    sum(durs) / len(durs),
        "dur_max_s":     max(durs),
        "n_transitions": len(overlaps),
        "n_gaps":        len(gaps),
        "n_simultaneous": len(siml),
        "ovl_min_s":     min(ovl),
        "ovl_mean_s":    sum(ovl) / len(ovl) if ovl else 0,
        "ovl_max_s":     max(ovl) if ovl else 0,
        "worst_ovl_ms":  worst_ovl_ms,
    }
    for fms in frame_ms_list:
        n_slots = int(fms / slot_ms)
        n_frames_in_worst = worst_ovl_ms / fms if fms > 0 else 0
        stats[f"f{int(fms)}_slots"]  = n_slots
        stats[f"f{int(fms)}_frames"] = n_frames_in_worst
    return stats


# ---------------------------------------------------------------------------
# Part 4: Frame allocation
# ---------------------------------------------------------------------------

def frame_layout(hot_cells, n_beams, kb, slot_ms, frame_ms_list):
    """
    Show slot allocation for hot vs non-hot cells per (frame, K_b) config.
    Model: hot cells get priority (minimum 1 slot each);
           remaining capacity shared among non-hot cells.
    """
    n_hot    = len(hot_cells)
    n_nonhot = n_beams - n_hot
    rows = []
    for fms in frame_ms_list:
        n_slots  = int(fms / slot_ms)
        capacity = n_slots * kb       # total beam-slots per frame
        # Hot cells guaranteed 1 slot each minimum; distribute remaining proportionally
        hot_min  = n_hot              # min beam-slots for hot cells
        avail    = capacity - hot_min
        extra_hot = avail // 3        # hot cells get extra 1/3 of remaining capacity
        hot_total = hot_min + extra_hot
        hot_each  = hot_total // n_hot if n_hot else 0
        nonhot_total = capacity - hot_each * n_hot
        nonhot_each  = nonhot_total // n_nonhot if n_nonhot else 0
        rows.append({
            "frame_ms":      fms,
            "kb":            kb,
            "n_slots":       n_slots,
            "capacity":      capacity,
            "hot_each":      hot_each,
            "nonhot_each":   nonhot_each,
            "hot_pct":       100.0 * hot_each / n_slots if n_slots else 0,
            "nonhot_pct":    100.0 * nonhot_each / n_slots if n_slots else 0,
        })
    return rows


# ---------------------------------------------------------------------------
# Figures
# ---------------------------------------------------------------------------

def fig_kb_snr(kb_rows, snr_base, elev_deg, snr_thresh, cfg, args, out_dir):
    kbs   = [r["kb"]             for r in kb_rows]
    snrs  = [r["snr_at_min_elev"] for r in kb_rows]
    crits = [r["crit_elev_deg"]  for r in kb_rows]
    colors = ["#2196F3" if r["ok"] else "#F44336" for r in kb_rows]

    elev_range, curves = snr_vs_elev_for_kb(args, cfg, [1, 2, 3, 4])

    fig, axes = plt.subplots(1, 3, figsize=(16, 4))

    # (a) Bar: SNR @ min_elev vs K_b
    ax = axes[0]
    ax.bar(kbs, snrs, color=colors, edgecolor="white", zorder=3)
    ax.axhline(snr_thresh, color="orange", linewidth=1.5, linestyle="--",
               label=f"Threshold {snr_thresh} dB")
    ax.set_xlabel("K_b (simultaneous beams)")
    ax.set_ylabel("SNR (dB)")
    ax.set_title(f"SNR at {elev_deg}° vs K_b\nBlue=OK, Red=fail")
    ax.set_xticks(kbs)
    ax.legend(fontsize=8)
    ax.grid(axis="y", alpha=0.3, zorder=0)

    # (b) Critical elevation vs K_b
    ax = axes[1]
    ax.plot(kbs, crits, "o-", color="#4CAF50", linewidth=2, markersize=7)
    ax.axhline(elev_deg, color="red", linewidth=1.5, linestyle="--",
               label=f"Min elev = {elev_deg}°")
    ax.fill_between(kbs, crits, elev_deg,
                    where=[c <= elev_deg for c in crits],
                    alpha=0.15, color="green", label="Viable")
    ax.fill_between(kbs, crits, elev_deg,
                    where=[c > elev_deg for c in crits],
                    alpha=0.15, color="red", label="Not viable at 37°")
    ax.set_xlabel("K_b")
    ax.set_ylabel("Critical elevation (deg)")
    ax.set_title("Min elevation required per K_b\n(must be ≤ 37° to guarantee service)")
    ax.set_xticks(kbs)
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    # (c) SNR curves vs elevation
    ax = axes[2]
    cmap = plt.colormaps["Set1"].resampled(4)
    for i, kb in enumerate([1, 2, 3, 4]):
        ax.plot(elev_range, curves[kb], color=cmap(i), linewidth=1.8,
                label=f"K_b={kb}")
    ax.axhline(snr_thresh, color="black", linewidth=1.2, linestyle="--",
               label=f"Threshold {snr_thresh} dB")
    ax.axvline(elev_deg, color="gray", linewidth=1.0, linestyle=":",
               label=f"Min elev {elev_deg}°")
    ax.set_xlabel("Elevation (deg)")
    ax.set_ylabel("SNR (dB)")
    ax.set_title("SNR vs Elevation per K_b\n(power split only, interference excluded)")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    plt.tight_layout()
    path = os.path.join(out_dir, "fig_kb_snr_analysis.png")
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"  [fig] {path}")


def fig_interference(irows, beam_radius_km, out_dir):
    if not irows:
        return
    pairs  = [r["pair"]     for r in irows]
    dists  = [r["dist_km"]  for r in irows]
    omegas = [r["omega_dB"] for r in irows]

    fig, ax = plt.subplots(figsize=(10, 4))
    bars = ax.bar(pairs, omegas, color="#9C27B0", edgecolor="white", zorder=3)
    ax.axhline(-20, color="orange", linestyle="--", linewidth=1.3,
               label="-20 dB (negligible threshold)")
    for bar, d in zip(bars, dists):
        ypos = bar.get_height() + 0.5
        ax.text(bar.get_x() + bar.get_width() / 2, ypos,
                f"{d:.0f} km", ha="center", fontsize=7, va="bottom")
    ax.set_xlabel("Hot cell pair")
    ax.set_ylabel("ω_{i,j} (dB)")
    ax.set_title(f"Inter-beam Interference — Fixed Hot Cells\n"
                 f"Beam radius = {beam_radius_km:.1f} km, "
                 f"Cell pitch = {2*beam_radius_km:.1f} km")
    ax.legend()
    ax.grid(axis="y", alpha=0.3, zorder=0)
    plt.tight_layout()
    path = os.path.join(out_dir, "fig_interference_hot_cells.png")
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"  [fig] {path}")


def fig_pass_windows(passes, overlaps, out_dir):
    durs = [p["dur_s"]     for p in passes]
    ovls = [o["overlap_s"] for o in overlaps]

    fig, axes = plt.subplots(1, 2, figsize=(13, 4))

    axes[0].hist(durs, bins=30, color="#2196F3", edgecolor="white", alpha=0.85)
    mean_dur = sum(durs) / len(durs)
    axes[0].axvline(mean_dur, color="red", linestyle="--", linewidth=1.5,
                    label=f"Mean = {mean_dur:.0f} s")
    axes[0].set_xlabel("Pass duration (s)")
    axes[0].set_ylabel("Number of passes")
    axes[0].set_title(f"Pass Duration @ Tokyo 37°  (n={len(passes)} passes)")
    axes[0].legend()

    axes[1].hist(ovls, bins=30, color="#4CAF50", edgecolor="white", alpha=0.85)
    axes[1].axvline(0, color="red", linewidth=1.5, linestyle="-",
                    label="0 s boundary (gap = negative)")
    axes[1].set_xlabel("Overlap with next satellite (s)")
    axes[1].set_ylabel("Count")
    axes[1].set_title("Handover Buffer Between Consecutive Satellites\n"
                      "Positive = simultaneous coverage available")
    axes[1].legend()

    plt.tight_layout()
    path = os.path.join(out_dir, "fig_pass_window_stats.png")
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"  [fig] {path}")


# ---------------------------------------------------------------------------
# Print helpers
# ---------------------------------------------------------------------------

def hdr(title):
    print(f"\n{'='*62}")
    print(f"  {title}")
    print(f"{'='*62}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    args      = parse_args()
    cfg       = build_cfg(args)
    hot_cells = [int(x.strip()) for x in args.hot_cells.split(",")]
    frame_ms  = [float(x.strip()) for x in args.frame_ms_list.split(",")]

    if args.output_dir:
        out_dir = os.path.abspath(args.output_dir)
    else:
        out_dir = os.path.normpath(
            os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "..", "ns_result", "bhtp_exp")
        )
    os.makedirs(out_dir, exist_ok=True)

    beam_radius_km = args.altitude_km * math.tan(math.radians(args.beam_half_angle_deg))
    cell_pitch_km  = 2.0 * beam_radius_km

    # ----------------------------------------------------------------
    hdr("Configuration")
    print(f"  Constellation     : Starlink-1584")
    print(f"  Altitude          : {args.altitude_km:.0f} km")
    print(f"  Frequency         : {args.freq_ghz:.0f} GHz (Ka-band)")
    print(f"  Tx power (total)  : {args.tx_power_w:.0f} W")
    print(f"  Antenna gain (UPA): {args.antenna_gain_db:.1f} dBi peak")
    print(f"  Noise figure      : {args.noise_figure_db:.0f} dB")
    print(f"  N_beams           : {args.n_beams} (5x5 elliptic grid)")
    print(f"  Min elevation     : {args.elev_deg:.0f}°  (per-plan constraint)")
    print(f"  SNR threshold     : {args.snr_thresh_db:.1f} dB")
    print(f"  Beam half-angle   : {args.beam_half_angle_deg:.1f}° (uniform MIDDLE pattern)")
    print(f"  Beam radius       : {beam_radius_km:.2f} km")
    print(f"  Cell pitch        : {cell_pitch_km:.2f} km")
    print(f"  Hot cells (fixed) : {hot_cells}")
    print(f"  Slot duration     : {args.slot_ms:.0f} ms")
    print(f"  Frame candidates  : {[int(f) for f in frame_ms]} ms")

    # ----------------------------------------------------------------
    hdr("Part 1 — K_b vs SNR  (Power Budget)")

    kb_rows, snr_base = compute_kb_snr(args, cfg)
    viable_kb = [r["kb"] for r in kb_rows if r["ok"]]
    kb_max_ok = max(viable_kb) if viable_kb else 0

    print(f"\n  Base SNR (K_b=1, full {args.tx_power_w:.0f} W, "
          f"elev={args.elev_deg:.0f}°): {snr_base:.2f} dB\n")

    header_row = ["K_b", "P/beam (W)", "Penalty (dB)",
                  f"SNR@{args.elev_deg:.0f}° (dB)", "Crit elev (°)", "OK?"]
    col_w = [5, 12, 13, 16, 14, 6]
    fmt = "  " + "  ".join(f"{{:<{w}}}" for w in col_w)
    print(fmt.format(*header_row))
    print("  " + "  ".join("-" * w for w in col_w))
    for r in kb_rows:
        ok_str = "YES ✓" if r["ok"] else "NO  ✗"
        print(fmt.format(
            str(r["kb"]),
            f"{r['power_per_beam_w']:.2f}",
            f"-{r['penalty_dB']:.2f}",
            f"{r['snr_at_min_elev']:.2f}",
            f"{r['crit_elev_deg']:.1f}",
            ok_str,
        ))

    print(f"\n  → Max K_b serviceable at {args.elev_deg:.0f}° min elevation: "
          f"K_b = {kb_max_ok}")
    print(f"    (Larger K_b still viable above their critical elevation;")
    print(f"     see 'Crit elev' column to check threshold per K_b.)")

    fig_kb_snr(kb_rows, snr_base, args.elev_deg, args.snr_thresh_db, cfg, args, out_dir)

    # ----------------------------------------------------------------
    hdr("Part 2 — Inter-Beam Interference  (Gaussian Model)")

    irows = compute_interference(hot_cells, beam_radius_km)
    max_omega_db = max(r["omega_dB"] for r in irows) if irows else -999.0

    print(f"\n  Beam radius = {beam_radius_km:.2f} km | Cell pitch = {cell_pitch_km:.2f} km")
    print(f"  ω = exp(-2.77 × (d / r_beam)²)  [3-dB point at d = r_beam]\n")

    i_hdr = ["Pair", "Dist (km)", "ω", "ω (dB)", "Verdict"]
    i_w   = [10, 10, 10, 10, 12]
    i_fmt = "  " + "  ".join(f"{{:<{w}}}" for w in i_w)
    print(i_fmt.format(*i_hdr))
    print("  " + "  ".join("-" * w for w in i_w))
    for r in irows:
        print(i_fmt.format(
            r["pair"],
            f"{r['dist_km']:.1f}",
            f"{r['omega']:.2e}",
            f"{r['omega_dB']:.1f}",
            r["verdict"],
        ))

    print(f"\n  → Worst-case inter-hot-cell interference: {max_omega_db:.1f} dB")
    if max_omega_db < -20.0:
        print(f"    Conclusion: ALL pairs < -20 dB → interference is NEGLIGIBLE.")
        print(f"    K_b limit is governed purely by POWER BUDGET (Part 1), not interference.")
    else:
        print(f"    Conclusion: interference is NON-negligible → include in SINR calculation.")

    fig_interference(irows, beam_radius_km, out_dir)

    # ----------------------------------------------------------------
    hdr("Part 3 — Pass Window Statistics  (C2a: Satellite Handover)")

    if args.status_json and os.path.exists(args.status_json):
        passes, status = load_passes(args.status_json)
        overlaps       = compute_overlaps(passes)
        stats          = pass_stats(passes, overlaps, args.slot_ms, frame_ms)

        print(f"\n  Source: {args.status_json}")
        print(f"  ROI   : Tokyo {status['roi_lat_deg']:.2f}°N "
              f"{status['roi_lon_deg']:.2f}°E  "
              f"(min_elev={status['min_elev_deg']:.0f}°, "
              f"window={status['window_s']:.0f} s)\n")

        print(f"  Pass duration (n={stats['n_passes']} passes):")
        print(f"    Min  : {stats['dur_min_s']:>7.0f} s")
        print(f"    Mean : {stats['dur_mean_s']:>7.0f} s")
        print(f"    Max  : {stats['dur_max_s']:>7.0f} s")

        print(f"\n  Handover buffer — overlap between consecutive satellite windows:")
        print(f"    Transitions total      : {stats['n_transitions']}")
        print(f"    With overlap (buffer≥0): {stats['n_simultaneous']}")
        print(f"    With gap    (buffer<0) : {stats['n_gaps']}  "
              f"{'<-- GAP PROBLEM!' if stats['n_gaps'] > 0 else '(none = coverage continuous)'}")
        print(f"    Overlap min  : {stats['ovl_min_s']:>7.1f} s  "
              f"= {stats['ovl_min_s']*1000:.0f} ms  (worst handover window)")
        print(f"    Overlap mean : {stats['ovl_mean_s']:>7.1f} s")
        print(f"    Overlap max  : {stats['ovl_max_s']:>7.1f} s")

        print(f"\n  BHTP frame adequacy for worst-case handover:")
        for fms in frame_ms:
            fi = int(fms)
            n  = stats[f"f{fi}_frames"]
            print(f"    Frame {fi:>2} ms ({int(fms/args.slot_ms)} slots): "
                  f"worst overlap = {n:.1f} frames → "
                  f"{'OK: pre-compute within window' if n >= 1.0 else 'MUST pre-compute BEFORE window'}")

        if stats["n_gaps"] == 0:
            print(f"\n  C2a result: NO coverage gaps (Starlink 1584 @ 37° confirmed).")
        print(f"  Worst-case handover buffer = {stats['worst_ovl_ms']:.0f} ms"
              f" = {stats['worst_ovl_ms']/max(frame_ms):.1f}× max frame.")
        print(f"  Recommendation: pre-compute next BHTP during overlap window.")

        fig_pass_windows(passes, overlaps, out_dir)

    else:
        print(f"\n  [SKIP] Provide --status_json to enable this analysis.")
        print(f"         Expected path: ns_result/0611/25beams/deg37/tokyo_out/"
              f"constellation_status.json")

    # ----------------------------------------------------------------
    hdr("Part 4 — Frame Configuration Comparison  (70 ms vs 80 ms)")

    print(f"\n  Hot cells ({len(hot_cells)}): {hot_cells}")
    print(f"  Non-hot cells ({args.n_beams - len(hot_cells)}): remaining {args.n_beams - len(hot_cells)} beams")
    print(f"  (Slot allocation model: hot gets priority; "
          f"non-hot shares remainder)\n")

    f_hdr = ["Frame", "Slots", "K_b", "Capacity",
             "Hot slots/cell", "Non-hot slots/cell", "Hot %", "Non-hot %"]
    f_w   = [8, 6, 4, 9, 15, 19, 8, 10]
    f_fmt = "  " + "  ".join(f"{{:<{w}}}" for w in f_w)
    print(f_fmt.format(*f_hdr))
    print("  " + "  ".join("-" * w for w in f_w))

    for kb_try in range(1, min(kb_max_ok + 2, args.kb_max + 1)):
        for r in frame_layout(hot_cells, args.n_beams, kb_try, args.slot_ms, frame_ms):
            note = " ← RECOMMENDED" if (kb_try == kb_max_ok and r["frame_ms"] == max(frame_ms)) else ""
            print(f_fmt.format(
                f"{int(r['frame_ms'])} ms",
                str(r["n_slots"]),
                str(r["kb"]),
                str(r["capacity"]),
                str(r["hot_each"]),
                str(r["nonhot_each"]),
                f"{r['hot_pct']:.0f}%",
                f"{r['nonhot_pct']:.0f}%",
            ) + note)

    # ----------------------------------------------------------------
    hdr("Summary")
    print(f"\n  1. K_b @ 37° min elevation : max viable = {kb_max_ok}")
    print(f"     Above crit elevation, higher K_b may be used (see Part 1 table).")
    print(f"\n  2. Interference            : {'negligible' if max_omega_db < -20 else 'NON-NEGLIGIBLE'} "
          f"(worst pair = {max_omega_db:.1f} dB)")
    print(f"     K_b limit = POWER BUDGET only.")
    print(f"\n  3. Handover (C2a)          : see Part 3 output above.")
    print(f"\n  4. Frame recommendation    : see Part 4 — compare 70/80 ms above.")
    print(f"\n  [output] {out_dir}")


if __name__ == "__main__":
    main()
