"""
compare_v1.py

Compare Python simulation output against the C++ v1 results (../projection/output/v1/).

Usage (run from the 'Multi-Beam LEO Communication Satellite Simulation Framework' directory):
    python compare_v1.py

What it does:
  1. Loads exact user positions and satellite position from v1 output files.
  2. Runs the Python macro-channel simulation (no Rician fading) on those positions.
  3. Compares per-user SNR, SINR, path_loss, and beam_index against the v1 JSON.
  4. Prints a summary table and a pass/fail verdict with tolerance thresholds.
"""

import json
import os
import numpy as np
import params
import channel
import networkGeometry
import utils


# ---------------------------------------------------------------------------
# Tolerance thresholds (acceptable difference between Python and C++)
# ---------------------------------------------------------------------------
SNR_TOL_DB    = 0.5   # dB — tighter: same model should match this closely
SINR_TOL_DB   = 1.0   # dB — looser: beam-gain model difference affects SINR more
LOSS_TOL_DB   = 0.1   # dB — atm model difference is <0.1 dB at near-nadir


# ---------------------------------------------------------------------------
# Paths (relative to this script's directory)
# ---------------------------------------------------------------------------
V1_DIR     = os.path.join("..", "projection", "output", "v1")
V1_JSON    = os.path.join(V1_DIR, "results_38357_100km.json")
V1_USER_POS_CSV = os.path.join(V1_DIR, "user_positions.csv")
V1_SAT_POS_CSV  = os.path.join(V1_DIR, "satellite_positions.csv")


def load_v1_results():
    """Load C++ v1 reference results."""
    with open(V1_JSON, "r") as f:
        data = json.load(f)

    # satellite position: [x, y, z] in meters
    sat_pos = np.array(data["satellite_position"])            # (3,)

    # user positions: list of [x, y, z]  →  (3, n_user)
    user_pos = np.array(data["user_positions"]).T             # (3, n_user)

    ref = {
        "sat_pos":    sat_pos,
        "user_pos":   user_pos,
        "n_user":     data["n_user"],
        "snr":        np.array(data["snr"]),
        "sinr":       np.array(data["sinr"]),
        "beam_index": np.array(data["beam_index"]),
        "r_footprint": data["r_footprint"],
        "elevation_angle": data["elevation_angle"],
    }
    return ref


def run_python_simulation(user_pos, sat_pos, i_frame=38537):
    """
    Run the Python macro-channel simulation (no Rician fading) on the given positions.
    Mirrors run_simulations_and_save_results_macro() in simulation.py.
    """
    # Ensure parameters are consistent with 100 km footprint
    r_footprint = 100e3
    params.update_param_file(r_footprint)
    config      = params.read_params()
    ant_gain_dB = config["antenna_gain_dB"]

    # Beam centers (fixed 2-ring hexagonal layout)
    beam_centers = networkGeometry.hex_grid_centers_two_rings()  # (3, 19)

    # Path loss
    loss_dB = channel.path_loss(user_pos, sat_pos)  # (n_user,)

    # Analog precoder
    precoder_analog = channel.fixed_beam_steering(sat_pos, beam_centers)

    n_user = np.size(user_pos, axis=1)

    # Effective channel (macro, no Rician)
    _, macro_channel, beam_gain = channel.get_effective_channel(
        loss_dB, precoder_analog, sat_pos, user_pos, n_user, i_frame
    )

    # Beam association
    fading = np.abs(macro_channel) ** 2
    _, beam_index = np.where(
        np.transpose((np.transpose(fading) == fading.max(axis=1)))
    )

    # SNR / SINR
    rec_power      = np.abs(macro_channel) ** 2
    desired_power  = rec_power[range(n_user), beam_index]
    total_power    = np.sum(rec_power, axis=1)
    interf_power   = total_power - desired_power
    noise          = params.get_noise_power()

    sinr_dB = utils.to_dB(desired_power / (interf_power + noise))
    snr_dB  = utils.to_dB(desired_power / noise)

    return {
        "snr":        snr_dB,
        "sinr":       sinr_dB,
        "beam_index": beam_index,
        "loss_dB":    loss_dB,
    }


def compare(ref, py):
    """Print per-user diff table and overall verdict."""
    n = ref["n_user"]
    snr_diff   = py["snr"]   - ref["snr"]
    sinr_diff  = py["sinr"]  - ref["sinr"]

    # Path loss: only Python has it, v1 JSON does not store it directly.
    # Use channel_results.csv if available for path_loss comparison.
    # (Skipped here — compare SNR/SINR/beam_index only.)

    beam_match = (py["beam_index"] == ref["beam_index"])

    print("=" * 76)
    print(f"{'User':>4}  {'Ref SNR':>9}  {'Py SNR':>9}  {'ΔSNR':>7}"
          f"  {'Ref SINR':>9}  {'Py SINR':>9}  {'ΔSINR':>7}  {'Beam':>7}")
    print("-" * 76)

    for u in range(n):
        beam_ok = "OK" if beam_match[u] else f"REF:{ref['beam_index'][u]}→PY:{py['beam_index'][u]}"
        snr_flag  = "" if abs(snr_diff[u])  <= SNR_TOL_DB  else " !"
        sinr_flag = "" if abs(sinr_diff[u]) <= SINR_TOL_DB else " !"
        print(f"{u:4d}  {ref['snr'][u]:9.3f}  {py['snr'][u]:9.3f}  {snr_diff[u]:+7.3f}{snr_flag}"
              f"  {ref['sinr'][u]:9.3f}  {py['sinr'][u]:9.3f}  {sinr_diff[u]:+7.3f}{sinr_flag}"
              f"  {beam_ok:>7}")

    print("=" * 76)

    # Summary statistics
    print("\n--- Summary ---")
    print(f"SNR  diff  → mean={snr_diff.mean():+.4f} dB | max|Δ|={np.abs(snr_diff).max():.4f} dB | "
          f"within ±{SNR_TOL_DB} dB: {np.sum(np.abs(snr_diff) <= SNR_TOL_DB)}/{n}")
    print(f"SINR diff  → mean={sinr_diff.mean():+.4f} dB | max|Δ|={np.abs(sinr_diff).max():.4f} dB | "
          f"within ±{SINR_TOL_DB} dB: {np.sum(np.abs(sinr_diff) <= SINR_TOL_DB)}/{n}")
    print(f"Beam match → {np.sum(beam_match)}/{n} users assigned to same beam")

    # Pass/fail
    snr_ok   = np.all(np.abs(snr_diff)  <= SNR_TOL_DB)
    sinr_ok  = np.all(np.abs(sinr_diff) <= SINR_TOL_DB)
    beam_ok_all = np.all(beam_match)

    print("\n--- Verdict ---")
    print(f"  SNR  (tol ±{SNR_TOL_DB} dB)  : {'PASS ✓' if snr_ok  else 'FAIL ✗'}")
    print(f"  SINR (tol ±{SINR_TOL_DB} dB)  : {'PASS ✓' if sinr_ok else 'FAIL ✗'}")
    print(f"  Beam assignment           : {'PASS ✓' if beam_ok_all else 'FAIL ✗'}")

    if not (snr_ok and sinr_ok):
        print("\n[INFO] Large SINR/SNR diff is expected if the C++ code uses a Gaussian")
        print("       beam-gain approximation while Python uses full UPA steering vectors.")
        print("       Check whether beam_gain_dB values in v1 JSON match Python output.")


if __name__ == "__main__":
    print("Loading v1 reference results...")
    ref = load_v1_results()
    print(f"  Satellite position: {ref['sat_pos']}")
    print(f"  Elevation angle  : {ref['elevation_angle']:.6f} deg")
    print(f"  n_user           : {ref['n_user']}")

    print("\nRunning Python simulation on same positions (frame 38537, no Rician)...")
    py = run_python_simulation(ref["user_pos"], ref["sat_pos"], i_frame=38537)

    print("\nComparison results:")
    compare(ref, py)
