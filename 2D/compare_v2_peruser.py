"""
compare_v2_peruser.py — Per-user SNR/SINR delta: C++ macro vs Python macro

Purpose:
    Feed the EXACT user positions from C++ (macro mode) back into the
    Python channel model (also in macro / no-fading mode), then compare
    per-user SNR and SINR.

    Reads from CSV files (no JSON dependency):
        user_positions.csv       — x_m, y_m, z_m per user
        channel_results.csv      — per-user snr_dB, sinr_dB (filtered by frame)
        satellite_positions.csv  — satellite x,y,z for the target frame

    Because both sides are deterministic (no Rician randomness), any
    remaining delta is purely from model differences (atmospheric loss
    approximation, floating-point accumulation, etc.).
    Target: |Δ SNR| 95th-percentile < 0.5 dB.

Prerequisites — run C++ in macro mode first (on Ubuntu SNS3):
    ./ns3 run "sat-multi-beam-simulation \
               --mode=macro \
               --frame=38537 \
               --out-dir=scratch/phase1" | tee scratch/phase1.log
    Then copy scratch/phase1/ to:
        2D/projection/output/phase1/

Usage (Windows, from repo root):
    python "2D/compare_v2_peruser.py"
    python "2D/compare_v2_peruser.py" --cpp-dir <path> [--frame 38537]
"""

import argparse
import csv
import os
import sys


import numpy as np

# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument(
        "--cpp-dir",
        default=r"c:\Users\wenj\Desktop\TriScale-LEO\2D\projection\output\phase1",
        help="Directory containing C++ macro-mode output CSVs",
    )
    p.add_argument("--frame", type=int, default=38537, help="Frame index to compare")
    p.add_argument(
        "--foot-km", type=int, default=100, help="Footprint radius in km (unused, kept for compatibility)"
    )
    return p.parse_args()


# ---------------------------------------------------------------------------
# Inline SNR/SINR calculation (mirrors simulation.py::calculate_simulation_result)
# avoids import-order issues; uses macro_channel (no Rician)
# ---------------------------------------------------------------------------

def calc_snr_sinr(macro_channel, beam_index, noise_w, ant_gain_db, beam_gain_matrix):
    """
    Parameters
    ----------
    macro_channel : (n_user, n_beams) complex, no fading
    beam_index    : (n_user,) int, assigned beam per user
    noise_w       : float, noise power in Watts
    ant_gain_db   : float, antenna array gain
    beam_gain_matrix : (n_user, n_beams) complex, raw beam gain before macro_loss

    Returns
    -------
    snr_db, sinr_db, center_bg_db : (n_user,) float arrays
    """
    import utils as _utils

    rec_power = np.abs(macro_channel) ** 2          # (n_user, n_beams)
    n_user    = macro_channel.shape[0]

    desired   = rec_power[np.arange(n_user), beam_index]          # (n_user,)
    total_rx  = rec_power.sum(axis=1)                              # (n_user,)
    interf    = total_rx - desired                                 # (n_user,)

    snr_db  = _utils.to_dB(desired / noise_w)
    sinr_db = _utils.to_dB(desired / (interf + noise_w))

    # center_beam_gain_dB — beam index 9 (central beam), matches Python definition
    center_bg_db = (
        _utils.to_dB(np.abs(beam_gain_matrix[:, 9]) ** 2) + ant_gain_db
    )
    return snr_db, sinr_db, center_bg_db


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    args = parse_args()

    sep = "=" * 68

    # -----------------------------------------------------------------------
    # Step 1 — Load C++ macro results from CSV files
    # -----------------------------------------------------------------------
    user_csv    = os.path.join(args.cpp_dir, "user_positions.csv")
    chan_csv    = os.path.join(args.cpp_dir, "channel_results.csv")
    sat_csv     = os.path.join(args.cpp_dir, "satellite_positions.csv")

    for p in [user_csv, chan_csv, sat_csv]:
        if not os.path.exists(p):
            print(f"[ERROR] File not found: {p}")
            print("  Run C++ in macro mode first (see script docstring).")
            sys.exit(1)

    # satellite position for the target frame
    cpp_sat_pos = None
    with open(sat_csv) as f:
        for row in csv.DictReader(f):
            if int(row["frame_id"]) == args.frame:
                cpp_sat_pos = np.array([float(row["x_m"]),
                                        float(row["y_m"]),
                                        float(row["z_m"])])
                break
    if cpp_sat_pos is None:
        print(f"[ERROR] Frame {args.frame} not found in {sat_csv}")
        sys.exit(1)

    # user positions: (3, n_user)
    user_xs, user_ys, user_zs = [], [], []
    with open(user_csv) as f:
        for row in csv.DictReader(f):
            user_xs.append(float(row["x_m"]))
            user_ys.append(float(row["y_m"]))
            user_zs.append(float(row["z_m"]))
    cpp_user_pos = np.array([user_xs, user_ys, user_zs])  # (3, n_user)

    # per-user results for the target frame
    cpp_snr, cpp_sinr, cpp_beam_idx, cpp_bg = [], [], [], []
    with open(chan_csv) as f:
        for row in csv.DictReader(f):
            if int(row["frame_id"]) == args.frame:
                cpp_snr.append(float(row["snr_dB"]))
                cpp_sinr.append(float(row["sinr_dB"]))
                cpp_beam_idx.append(int(row["beam_id"]))
                cpp_bg.append(float(row["beam_gain_dB"]))

    cpp_snr      = np.array(cpp_snr)
    cpp_sinr     = np.array(cpp_sinr)
    cpp_beam_idx = np.array(cpp_beam_idx)
    cpp_bg       = np.array(cpp_bg)
    n_user       = len(cpp_snr)

    if n_user == 0:
        print(f"[ERROR] No rows for frame {args.frame} in {chan_csv}")
        sys.exit(1)

    print(sep)
    print(f"Loaded C++ macro results  — frame={args.frame}, n_user={n_user}")
    print(f"  sat_pos = [{cpp_sat_pos[0]:.1f}, {cpp_sat_pos[1]:.1f}, {cpp_sat_pos[2]:.1f}] m")
    print(f"  C++ SNR  mean={cpp_snr.mean():.3f} dB  max={cpp_snr.max():.3f} dB")
    print(f"  C++ SINR mean={cpp_sinr.mean():.3f} dB  max={cpp_sinr.max():.3f} dB")

    # -----------------------------------------------------------------------
    # Step 2 — Run Python macro channel with the same user positions
    # -----------------------------------------------------------------------
    py_dir = os.path.join(
        os.path.dirname(__file__),
        "Multi-Beam LEO Communication Satellite Simulation Framework",
    )
    if not os.path.isdir(py_dir):
        print(f"[ERROR] Python framework directory not found:\n  {py_dir}")
        sys.exit(1)

    # Change CWD so params.py can write/read params.json from that directory
    orig_dir = os.getcwd()
    os.chdir(py_dir)
    if py_dir not in sys.path:
        sys.path.insert(0, py_dir)

    import params
    import channel
    import networkGeometry
    import utils

    # Init params with matching footprint
    params.update_param_file(args.foot_km * 1e3)
    config      = params.read_params()
    ant_gain_db = config["antenna_gain_dB"]
    noise_w     = params.get_noise_power()

    # Use the exact satellite position recorded by C++
    sat_pos = cpp_sat_pos  # (3,)

    # Beam centres (identical construction on both sides)
    beam_centers = networkGeometry.hex_grid_centers_two_rings()  # (3, 19)

    print()
    print("Running Python macro channel on C++ user positions...")

    # Path loss — use C++ values directly from the CSV.
    # Rationale: Python's itur library gives ~8-9 dB higher atmospheric loss for
    # these inputs than C++'s simplified 0.55/sin(ε) model.  The goal of this
    # script is to validate the BEAM-GAIN model (UPA Dirichlet kernel), not the
    # atmospheric loss model.  Using identical path-loss values on both sides
    # isolates any remaining delta to the beam model alone.
    print("  Loading C++ path_loss_dB values to use as shared input...")
    cpp_pl_list = []
    with open(chan_csv) as _f:
        for _row in csv.DictReader(_f):
            if int(_row["frame_id"]) == args.frame:
                cpp_pl_list.append(float(_row["path_loss_dB"]))
    loss_db = np.array(cpp_pl_list)   # (n_user,) — identical to C++ input
    print(f"  path_loss loaded: mean={loss_db.mean():.3f} dB  "
          f"min={loss_db.min():.3f}  max={loss_db.max():.3f} dB")

    # Analog precoder (fixed beam steering) — computed once
    precoder = channel.fixed_beam_steering(sat_pos, beam_centers)  # (n_ant_total, n_beams)

    # Batch processing — get_array_steering_vector allocates (n_user × 19456) complex128
    # which is ~291 MB per 1000 users. Process in chunks to stay under ~600 MB.
    BATCH = 2000
    py_snr_list  = []
    py_sinr_list = []
    py_bg_list   = []
    py_beam_list = []

    n_batches = (n_user + BATCH - 1) // BATCH
    for b in range(n_batches):
        lo = b * BATCH
        hi = min(lo + BATCH, n_user)
        batch_pos    = cpp_user_pos[:, lo:hi]   # (3, batch_size)
        batch_loss   = loss_db[lo:hi]            # (batch_size,)
        batch_n      = hi - lo

        print(f"  batch {b+1}/{n_batches}  users {lo}–{hi-1}", end="\r", flush=True)

        _, mc, bg = channel.get_effective_channel(
            batch_loss, precoder, sat_pos, batch_pos, batch_n, args.frame
        )  # mc: (batch_n, 19)

        fading = np.abs(mc) ** 2
        _, bi = np.where(np.transpose(np.transpose(fading) == fading.max(axis=1)))

        bsnr, bsinr, bbg = calc_snr_sinr(mc, bi, noise_w, ant_gain_db, bg)

        py_snr_list.append(bsnr)
        py_sinr_list.append(bsinr)
        py_bg_list.append(bbg)
        py_beam_list.append(bi)

    print()  # newline after \r progress

    py_snr      = np.concatenate(py_snr_list)
    py_sinr     = np.concatenate(py_sinr_list)
    py_bg       = np.concatenate(py_bg_list)
    py_beam_idx = np.concatenate(py_beam_list)

    os.chdir(orig_dir)

    # -----------------------------------------------------------------------
    # Step 3 — Per-user delta statistics
    # -----------------------------------------------------------------------
    delta_snr  = py_snr  - cpp_snr   # Python - C++  (n_user,)
    delta_sinr = py_sinr - cpp_sinr

    # Beam agreement: fraction of users assigned to the same beam
    beam_agree = np.mean(py_beam_idx == cpp_beam_idx)

    p50_snr  = np.percentile(np.abs(delta_snr),  50)
    p95_snr  = np.percentile(np.abs(delta_snr),  95)
    p99_snr  = np.percentile(np.abs(delta_snr),  99)
    p95_sinr = np.percentile(np.abs(delta_sinr), 95)

    print()
    print(sep)
    print("Per-user comparison: Python macro vs C++ macro")
    print("-" * 68)
    print(f"n_user                    : {n_user}")
    print(f"Beam agreement            : {beam_agree * 100:.1f}%  (same beam assigned)")
    print()
    print("                           Python       C++       delta_median delta_p95")
    print(f"  SNR  mean (dB)  : {py_snr.mean():>10.3f}  {cpp_snr.mean():>10.3f}")
    print(f"  SNR  max  (dB)  : {py_snr.max():>10.3f}  {cpp_snr.max():>10.3f}")
    print(f"  SINR mean (dB)  : {py_sinr.mean():>10.3f}  {cpp_sinr.mean():>10.3f}")
    print()
    print(f"  |Δ SNR|  p50    : {p50_snr:.4f} dB")
    print(f"  |Δ SNR|  p95    : {p95_snr:.4f} dB   {'✅ PASS' if p95_snr < 0.5 else '❌ FAIL (> 0.5 dB)'}")
    print(f"  |Δ SNR|  p99    : {p99_snr:.4f} dB")
    print(f"  |Δ SINR| p95    : {p95_sinr:.4f} dB   {'✅ PASS' if p95_sinr < 0.5 else '❌ FAIL (> 0.5 dB)'}")
    print()
    print(f"  beam_gain max   : Python={py_bg.max():.3f} dB  C++={cpp_bg.max():.3f} dB"
          f"  {'✅' if abs(py_bg.max() - cpp_bg.max()) < 0.01 else '⚠️'}")
    print(sep)

    # -----------------------------------------------------------------------
    # Step 4 — Breakdown: Δ SNR vs atmospheric loss model (path loss diff)
    # -----------------------------------------------------------------------
    # Rough atmospheric loss difference at 90° elevation:
    #   Python: full ITU-Rpy  ≈ 0.55 dB
    #   C++:    simplified     = 0.55 dB / sin(90°) = 0.55 dB
    #   → should be < 0.1 dB at high elevation; larger at low elevation
    print("Δ SNR histogram (all users, Python − C++, dB):")
    bins    = np.arange(-2.5, 2.6, 0.5)
    counts, edges = np.histogram(delta_snr, bins=bins)
    for lo, hi, cnt in zip(edges, edges[1:], counts):
        bar = "#" * (cnt * 40 // max(counts, default=1) if counts.max() > 0 else 0)
        print(f"  [{lo:+.1f}, {hi:+.1f}) : {cnt:6d}  {bar}")

    # Outlier report (|Δ SNR| > 0.5 dB)
    outliers = np.where(np.abs(delta_snr) > 0.5)[0]
    if len(outliers) > 0:
        print(f"\n  Outlier users (|ΔSNR|>0.5 dB): {len(outliers)} / {n_user} "
              f"({100*len(outliers)/n_user:.1f}%)")
        # Show 5 worst
        worst = outliers[np.argsort(np.abs(delta_snr[outliers]))[::-1][:5]]
        print("  Top-5 worst users:")
        print(f"  {'user_id':>8}  {'Δ SNR':>8}  {'Δ SINR':>8}  "
              f"{'py_beam':>7}  {'cpp_beam':>8}")
        for uid in worst:
            print(f"  {uid:8d}  {delta_snr[uid]:+8.3f}  {delta_sinr[uid]:+8.3f}  "
                  f"{py_beam_idx[uid]:7d}  {cpp_beam_idx[uid]:8d}")
    else:
        print("\n  No outliers (|Δ SNR| > 0.5 dB) — model equivalence confirmed ✅")

    print(sep)


if __name__ == "__main__":
    main()
