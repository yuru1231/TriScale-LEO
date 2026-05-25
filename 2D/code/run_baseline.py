import json
import numpy as np

import channel
import networkGeometry
import params
import utils


def simulation_result(ch, n_user, beam_idx, beam_gain, ant_db):
    from utils import to_dB

    rec = np.abs(ch) ** 2
    des = rec[range(n_user), beam_idx]
    intfr = rec.sum(axis=1) - des
    noise = params.get_noise_power()
    sinr = to_dB(des / (intfr + noise))
    snr = to_dB(des / noise)
    bg_dB = to_dB(np.abs(beam_gain[range(n_user), 9]) ** 2) + ant_db
    return sinr, snr, bg_dB


params.update_param_file(100e3)
config = params.read_params()
ant_dB = config["antenna_gain_dB"]

sat_pos = networkGeometry.get_satellite_pos()
sat_pos[:, 38537] = np.array([0, 0, 600e3])

# Use 33333m spacing to match the ~37-user scale in C++ Phase 1.1.
user_pos = networkGeometry.get_grid_positions(33333)
n_user = np.size(user_pos, axis=1)
beam_centers = networkGeometry.hex_grid_centers_two_rings()

print(f"n_user = {n_user}")

for frame, label in [(38537, "90deg"), (33090, "55deg"), (23932, "25deg")]:
    i_sat = sat_pos[:, frame]
    elev = (
        utils.get_elevation_angle_from_center(
            np.array([i_sat[0]]), np.array([i_sat[2]])
        )[0]
        / np.pi
        * 180
    )

    loss_dB = channel.path_loss(user_pos, i_sat)
    precoder = channel.fixed_beam_steering(i_sat, beam_centers)
    _, macro, beam_gain = channel.get_effective_channel(
        loss_dB, precoder, i_sat, user_pos, n_user, frame
    )

    fading = np.abs(macro) ** 2
    _, beam_idx = np.where(np.transpose(np.transpose(fading) == fading.max(axis=1)))

    sinr, snr, bg_dB = simulation_result(macro, n_user, beam_idx, beam_gain, ant_dB)

    print(f"\n--- Frame {frame} ({label}, elev={elev:.3f} deg) ---")
    print(f"  sat_pos        : {i_sat.tolist()}")
    print(f"  user[0] beam   : {beam_idx[0]}")
    print(f"  user[0] snr    : {snr[0]:.4f} dB")
    print(f"  user[0] sinr   : {sinr[0]:.4f} dB")
    print(f"  user[0] ctr_bg : {bg_dB[0]:.4f} dB")
    print(f"  user[0] loss   : {loss_dB[0]:.4f} dB")
    print(f"  snr  mean/min/max : {snr.mean():.3f} / {snr.min():.3f} / {snr.max():.3f}")
    print(
        f"  sinr mean/min/max : "
        f"{sinr.mean():.3f} / {sinr.min():.3f} / {sinr.max():.3f}"
    )
