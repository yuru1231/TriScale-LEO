"""
analysis/link_budget.py
-----------------------
Pure-function module for satellite link budget calculations and MRC combining.

All parameters mirror SimConfig (sat-multi-beam-config.h) defaults.
No I/O — import and call functions directly.

Public API
----------
snr_at_elevation(elev_deg, cfg=None)      -> float (dB)
mrc_combine_snr_db(snr_db_list)           -> float (dB)
link_budget_table(elevs_deg, cfg=None)    -> list[dict]
critical_elevation(cfg=None)              -> float (deg)
"""

import math
from typing import List, Optional

# ---------------------------------------------------------------------------
# Default SimConfig parameters (mirrors sat-multi-beam-config.h)
# ---------------------------------------------------------------------------

DEFAULT_CFG = {
    "speed_of_light_ms":  299_792_458.0,   # m/s
    "boltzmann":          1.3806485e-23,   # J/K
    "h_satellite_m":      600e3,           # m   — LEO altitude
    "r_earth_m":          6_371e3,         # m   — mean Earth radius
    "center_freq_hz":     30e9,            # Hz  — Ka-band
    "bandwidth_hz":       25e6,            # Hz  — channel bandwidth
    "antenna_gain_db":    60.5,            # dBi — UPA peak gain
    "noise_figure_db":    7.0,             # dB  — receiver NF
    "transmit_power_w":   63.0,            # W
    "n_beams":            19,              # total beams (2-ring hex)
    "temperature_k":      300.0,           # K
}


def _merge_cfg(user_cfg):
    cfg = dict(DEFAULT_CFG)
    if user_cfg:
        cfg.update(user_cfg)
    return cfg


# ---------------------------------------------------------------------------
# Core calculations
# ---------------------------------------------------------------------------

def _slant_range_m(elev_deg: float, cfg: dict) -> float:
    """
    Slant range from ground observer to satellite (m).
    Derived from the law of cosines on the Earth-satellite triangle:
        range = R_E * (sqrt((R_orb/R_E)^2 - cos^2(elev)) - sin(elev))
    """
    r_e   = cfg["r_earth_m"]
    r_orb = r_e + cfg["h_satellite_m"]
    elev  = math.radians(elev_deg)
    cos_e = math.cos(elev)
    sin_e = math.sin(elev)
    return r_e * (math.sqrt((r_orb / r_e) ** 2 - cos_e ** 2) - sin_e)


def _fspl_db(slant_m: float, cfg: dict) -> float:
    """Free-space path loss (dB): 20*log10(4π*d*f/c)."""
    return 20.0 * math.log10(
        4.0 * math.pi * slant_m * cfg["center_freq_hz"] / cfg["speed_of_light_ms"]
    )


def _beam_gain_db(cfg: dict) -> float:
    """
    Effective beam gain at beam centre for the serving beam (dB).
    UPA array with N_beams divides peak gain evenly in the far-field:
        G_beam = 10*log10(1/N_beams) + G_peak_dBi
    This is the gain a user at the exact beam-centre point receives.
    """
    return 10.0 * math.log10(1.0 / cfg["n_beams"]) + cfg["antenna_gain_db"]


def _noise_power_dbw(cfg: dict) -> float:
    """Thermal noise power (dBW): k*T*B*10^(NF/10)."""
    noise_w = (cfg["boltzmann"] * cfg["temperature_k"]
               * cfg["bandwidth_hz"]
               * 10.0 ** (cfg["noise_figure_db"] / 10.0))
    return 10.0 * math.log10(noise_w)


def snr_at_elevation(elev_deg: float, cfg: Optional[dict] = None) -> float:
    """
    Analytical SNR (dB) for a user at the serving beam centre at the given
    satellite elevation angle.

    Parameters
    ----------
    elev_deg : satellite elevation angle above horizon (degrees, 0–90)
    cfg      : optional dict overriding any DEFAULT_CFG key

    Returns
    -------
    SNR in dB (can be negative at low elevation).
    """
    c = _merge_cfg(cfg)
    slant    = _slant_range_m(elev_deg, c)
    fspl     = _fspl_db(slant, c)
    g_beam   = _beam_gain_db(c)
    tx_dbw   = 10.0 * math.log10(c["transmit_power_w"])
    rx_dbw   = tx_dbw + g_beam - fspl
    noise_dbw = _noise_power_dbw(c)
    return rx_dbw - noise_dbw


def mrc_combine_snr_db(snr_db_list: list) -> float:
    """
    Maximum-Ratio Combining (MRC) of independent satellite paths.

    For N satellites with individual SNR γ_i (linear), MRC gives:
        γ_combined = Σ γ_i
    which in dB is: 10*log10(Σ 10^(γ_i / 10)).

    Parameters
    ----------
    snr_db_list : list of per-satellite SNR values (dB), non-empty

    Returns
    -------
    Combined SNR in dB, or -999.0 if the list is empty.
    """
    if not snr_db_list:
        return -999.0
    linear_sum = sum(10.0 ** (s / 10.0) for s in snr_db_list)
    return 10.0 * math.log10(linear_sum) if linear_sum > 0.0 else -999.0


# ---------------------------------------------------------------------------
# Convenience helpers
# ---------------------------------------------------------------------------

def link_budget_table(
    elevs_deg: Optional[List[float]] = None,
    cfg: Optional[dict] = None,
) -> list:
    """
    Compute SNR at a set of elevation angles.

    Parameters
    ----------
    elevs_deg : list of elevation angles (deg); defaults to [5,10,20,30,45,60,90]
    cfg       : optional SimConfig overrides

    Returns
    -------
    List of dicts: [{"elev_deg": ..., "slant_km": ..., "fspl_db": ..., "snr_db": ...}]
    """
    if elevs_deg is None:
        elevs_deg = [5, 10, 20, 30, 45, 60, 90]
    c = _merge_cfg(cfg)
    rows = []
    for e in elevs_deg:
        slant = _slant_range_m(e, c)
        rows.append({
            "elev_deg": e,
            "slant_km": slant / 1e3,
            "fspl_db":  _fspl_db(slant, c),
            "snr_db":   snr_at_elevation(e, cfg),
        })
    return rows


def critical_elevation(cfg: Optional[dict] = None, snr_thresh_db: float = 0.0) -> float:
    """
    Binary-search for the elevation angle at which SNR = snr_thresh_db.

    Returns the critical elevation in degrees (between 0 and 90).
    Returns 0.0 if even nadir (90°) is below threshold.
    Returns 90.0 if even the minimum elevation (5°) is above threshold.
    """
    lo, hi = 0.01, 90.0
    if snr_at_elevation(hi, cfg) < snr_thresh_db:
        return 90.0
    if snr_at_elevation(lo, cfg) >= snr_thresh_db:
        return 0.0
    for _ in range(60):
        mid = (lo + hi) / 2.0
        if snr_at_elevation(mid, cfg) < snr_thresh_db:
            lo = mid
        else:
            hi = mid
    return round((lo + hi) / 2.0, 2)
