"""
analysis/link_budget.py
-----------------------
Pure-function module for satellite link budget calculations and MRC combining.

All parameters mirror SimConfig (sat-multi-beam-config.h) with Iridium-NEXT
constellation defaults.

Path loss model:
  L_total = FSPL + L_atm
  FSPL    = 20·log10(4π·d·f/c)          (d from spherical Earth geometry)
  L_atm   = atm_loss_zenith_db / sin(ε)  (simple zenith-scaled model,
             3GPP TR 38.811 Ka-band clear sky ≈ 0.4 dB)

Beam gain model (5×5 elliptic grid):
  G_beam  = 10·log10(1/N_beams) + G_peak_dBi
  N_beams = 25   (5×5 along-track/cross-track elliptic grid)
  Peak at beam centre: |g|²_peak = 1/25 → G_beam = −14 + 60.5 = 46.5 dBi

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
    "speed_of_light_ms":    299_792_458.0,  # m/s
    "boltzmann":            1.3806485e-23,  # J/K
    "h_satellite_m":        634e3,          # m   — Iridium-NEXT TLE epoch ~1999 actual altitude
                                            #       (SGP4-derived from constellation_status.json: 634.2 km)
                                            #       overridden at runtime by check_coverage.py via JSON
    "r_earth_m":            6_371e3,        # m   — mean Earth radius
    "center_freq_hz":       30e9,           # Hz  — Ka-band
    "bandwidth_hz":         25e6,           # Hz  — channel bandwidth
    "antenna_gain_db":      60.5,           # dBi — UPA peak gain
    "noise_figure_db":      7.0,            # dB  — receiver noise figure
    "transmit_power_w":     63.0,           # W
    "n_beams":              25,             # total beams (5×5 elliptic grid)
    "temperature_k":        300.0,          # K
    "atm_loss_zenith_db":   0.4,            # dB  — Ka-band zenith atmospheric absorption
                                            #       (3GPP TR 38.811, clear sky ≈ 0.4 dB)
                                            #       L_atm(ε) = atm_loss_zenith / sin(ε)
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
    Spherical Earth law of cosines:
        d = R_E · (sqrt((R_orb/R_E)² − cos²(ε)) − sin(ε))
    """
    r_e   = cfg["r_earth_m"]
    r_orb = r_e + cfg["h_satellite_m"]
    elev  = math.radians(elev_deg)
    cos_e = math.cos(elev)
    sin_e = math.sin(elev)
    return r_e * (math.sqrt((r_orb / r_e) ** 2 - cos_e ** 2) - sin_e)


def _fspl_db(slant_m: float, cfg: dict) -> float:
    """Free-space path loss: L_FSPL = 20·log10(4π·d·f/c)  (dB)."""
    return 20.0 * math.log10(
        4.0 * math.pi * slant_m * cfg["center_freq_hz"] / cfg["speed_of_light_ms"]
    )


def _atmospheric_loss_db(elev_deg: float, cfg: dict) -> float:
    """
    Atmospheric attenuation scaled from zenith value:
        L_atm(ε) = L_zenith / sin(max(ε, 5°))
    Mirrors the approximation used by ComputeAtmosphericLoss_dB in C++
    (extracted from ThreeGppNTNDenseUrbanPropagationLossModel).
    """
    elev_eff = max(elev_deg, 5.0)
    return cfg["atm_loss_zenith_db"] / math.sin(math.radians(elev_eff))


def _beam_gain_db(cfg: dict) -> float:
    """
    Effective beam gain at beam centre for the serving beam (dBi).
    UPA Dirichlet-kernel peak:
        G_beam = 10·log10(1/N_beams) + G_peak_dBi
    N_beams=25: G_beam = −14.0 + 60.5 = 46.5 dBi
    """
    return 10.0 * math.log10(1.0 / cfg["n_beams"]) + cfg["antenna_gain_db"]


def _noise_power_dbw(cfg: dict) -> float:
    """Thermal noise power: N = k·T·B·10^(NF/10)  (dBW)."""
    noise_w = (cfg["boltzmann"] * cfg["temperature_k"]
               * cfg["bandwidth_hz"]
               * 10.0 ** (cfg["noise_figure_db"] / 10.0))
    return 10.0 * math.log10(noise_w)


def snr_at_elevation(elev_deg: float, cfg: Optional[dict] = None) -> float:
    """
    Analytical SNR (dB) for a user at the serving beam centre at the given
    satellite elevation angle.

    Formula:
        SNR = P_tx + G_beam − L_FSPL − L_atm − N_thermal

    Parameters
    ----------
    elev_deg : satellite elevation angle above horizon (degrees, 0–90)
    cfg      : optional dict overriding any DEFAULT_CFG key

    Returns
    -------
    SNR in dB (can be negative at low elevation).
    """
    c         = _merge_cfg(cfg)
    slant     = _slant_range_m(elev_deg, c)
    fspl      = _fspl_db(slant, c)
    l_atm     = _atmospheric_loss_db(elev_deg, c)
    g_beam    = _beam_gain_db(c)
    tx_dbw    = 10.0 * math.log10(c["transmit_power_w"])
    rx_dbw    = tx_dbw + g_beam - fspl - l_atm
    noise_dbw = _noise_power_dbw(c)
    return rx_dbw - noise_dbw


def mrc_combine_snr_db(snr_db_list: list) -> float:
    """
    Maximum-Ratio Combining (MRC) of independent satellite paths.

    γ_MRC = Σᵢ γᵢ_linear  →  SNR_MRC = 10·log10(Σᵢ 10^(γᵢ/10))

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
    Compute link budget metrics at a set of elevation angles.

    Parameters
    ----------
    elevs_deg : list of elevation angles (deg).
                Defaults to [5, 10, 15, 20, 25, 30, 45, 60, 75, 90].
    cfg       : optional SimConfig overrides

    Returns
    -------
    List of dicts with keys:
        elev_deg, slant_km, fspl_db, atm_loss_db, total_path_loss_db, snr_db
    """
    if elevs_deg is None:
        elevs_deg = [5, 10, 15, 20, 25, 30, 45, 60, 75, 90]
    c = _merge_cfg(cfg)
    rows = []
    for e in elevs_deg:
        slant    = _slant_range_m(e, c)
        fspl     = _fspl_db(slant, c)
        l_atm    = _atmospheric_loss_db(e, c)
        snr      = snr_at_elevation(e, cfg)
        rows.append({
            "elev_deg":          e,
            "slant_km":          slant / 1e3,
            "fspl_db":           fspl,
            "atm_loss_db":       l_atm,
            "total_path_loss_db": fspl + l_atm,
            "snr_db":            snr,
        })
    return rows


def critical_elevation(cfg: Optional[dict] = None, snr_thresh_db: float = 0.0) -> float:
    """
    Binary-search for the elevation angle at which SNR = snr_thresh_db.

    Returns the critical elevation in degrees (0–90).
    Returns  0.0 if SNR ≥ threshold even at the horizon.
    Returns 90.0 if SNR < threshold even at zenith.
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


# ---------------------------------------------------------------------------
# Standalone usage
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    c      = DEFAULT_CFG
    g_beam = _beam_gain_db(c)
    n_thw  = _noise_power_dbw(c)
    crit   = critical_elevation()

    print("=== Link Budget Parameters ===")
    print(f"  h_satellite   : {c['h_satellite_m']/1e3:.0f} km  (Iridium-NEXT)")
    print(f"  freq          : {c['center_freq_hz']/1e9:.0f} GHz  (Ka-band)")
    print(f"  n_beams       : {c['n_beams']}  (5×5 elliptic grid)")
    print(f"  G_beam (peak) : {g_beam:.2f} dBi  "
          f"= 10·log10(1/{c['n_beams']}) + {c['antenna_gain_db']} dBi")
    print(f"  P_tx          : {c['transmit_power_w']:.0f} W  "
          f"= {10*math.log10(c['transmit_power_w']):.1f} dBW")
    print(f"  N_thermal     : {n_thw:.2f} dBW  "
          f"(T={c['temperature_k']:.0f} K, B={c['bandwidth_hz']/1e6:.0f} MHz, "
          f"NF={c['noise_figure_db']:.0f} dB)")
    print(f"  L_atm zenith  : {c['atm_loss_zenith_db']:.1f} dB  "
          f"(Ka-band clear sky, 3GPP TR 38.811)")
    print()

    print("=== Link Budget vs Elevation ===")
    rows = link_budget_table()
    print(f"  {'Elev (°)':>8s}  {'Slant (km)':>10s}  "
          f"{'FSPL (dB)':>9s}  {'L_atm (dB)':>10s}  {'SNR (dB)':>8s}")
    print(f"  {'-'*8}  {'-'*10}  {'-'*9}  {'-'*10}  {'-'*8}")
    thresh = 0.0
    for r in rows:
        marker = " ← below threshold" if r["snr_db"] < thresh else ""
        print(f"  {r['elev_deg']:>8.0f}  {r['slant_km']:>10.1f}  "
              f"{r['fspl_db']:>9.1f}  {r['atm_loss_db']:>10.2f}  "
              f"{r['snr_db']:>8.1f}{marker}")
    print(f"\n  Critical elevation (SNR = 0 dB): {crit:.1f}°")
    print(f"  Satellites below {crit:.1f}° elevation cannot provide service")
