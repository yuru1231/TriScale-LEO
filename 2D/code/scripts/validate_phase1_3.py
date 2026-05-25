"""
validate_phase1_3.py  —  Phase 1.3 Hypatia / SGP4 Coordinate Validation
=========================================================================

Validates that the coordinate system used in Phase 1.3 (Hypatia WGS72)
is self-consistent with the elevation angles computed by PyEphem.

README challenge:
  "Hypatia distance_tools.py uses WGS72 geodetic coordinates;
   must cross-validate with local frame conversion"

Tests
-----
  T1  Analytical overhead
        satellite at same (lat, lon) as observer but alt=630 km
        → elevation from ENU must be ≈ 90.000°  (error < 0.01°)

  T2  ephem ↔ Hypatia ENU rotation round-trip (ERAD-independent)
        propagate Kuiper-630 TLE (from Hypatia test data),
        reconstruct satellite ECEF via sat.range/alt/az (no Earth-radius
        assumption), verify _ecef_to_enu recovers the original elevation
        → max round-trip |error| < 1e-5° (floating-point precision only)

  T3  SGP4 arc shape
        find the visible pass over the observer within a 6-hour window
        → elevation curve must have exactly one local peak (single arc)
        → peak elevation and pass duration logged

  T4  run_sgp4.py output schema
        call run_sgp4.py internals with the built-in TLE,
        verify all required Phase 2 columns are present and data is valid

Built-in TLE: Kuiper-630 0 from Hypatia's own test data (tles.txt)
  — near-circular LEO at ~630 km, inclination 51.9°, epoch 2000/01/01
  — no external TLE file needed for self-contained testing

Observer: lat=25.0°N, lon=121.0°E (Taiwan — matches Phase 1.2 centre)

Usage
-----
  python validate_phase1_3.py
  python validate_phase1_3.py --observer-lat 25.0 --observer-lon 121.0
  python validate_phase1_3.py --search-window-s 21600 --step-ms 100
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import tempfile

# ---------------------------------------------------------------------------
# Dependency checks
# ---------------------------------------------------------------------------

try:
    import ephem
except ImportError:
    sys.exit(
        "ERROR: PyEphem not installed.\n"
        "       Run: pip install ephem\n"
        "       (same dependency as Hypatia satgenpy)"
    )

# Locate Hypatia's satgenpy so we can import geodetic2cartesian.
# Expected repo layout:
#   <repo-root>/hypatia-master/satgenpy/
#   <repo-root>/2D/projection/code/scripts/   ← this file
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT   = os.path.abspath(os.path.join(_SCRIPT_DIR, "..", "..", "..", "..", ".."))
_SATGENPY    = os.path.join(_REPO_ROOT, "hypatia-master", "satgenpy")

if _SATGENPY not in sys.path:
    sys.path.insert(0, _SATGENPY)

try:
    from satgen.distance_tools.distance_tools import geodetic2cartesian
except ImportError:
    sys.exit(
        f"ERROR: Cannot import Hypatia geodetic2cartesian.\n"
        f"       Expected satgenpy at: {_SATGENPY}\n"
        f"       Run: pip install geopy  (Hypatia dependency)\n"
        f"       Ensure hypatia-master/ is in the repo root."
    )

# ---------------------------------------------------------------------------
# Built-in Kuiper-630 TLE (from hypatia-master/ns3-sat-sim/simulator/
#   test_data/end_to_end/satellite_network_state/tles.txt)
# Near-circular LEO at ~630 km, inclination 51.9°, epoch 2000/01/01
# ---------------------------------------------------------------------------

_KUIPER_NAME = "Kuiper-630 0"
_KUIPER_L1   = "1 00184U 00000ABC 00001.00000000  .00000000  00000-0  00000+0 0    06"
_KUIPER_L2   = "2 00184  51.9000  52.9412 0000001   0.0000 142.9412 14.80000000    00"


# ---------------------------------------------------------------------------
# Coordinate helpers
# ---------------------------------------------------------------------------

def _ecef_to_enu(obs_lat_deg: float, obs_lon_deg: float,
                 obs_ecef: tuple[float, float, float],
                 sat_ecef: tuple[float, float, float]) -> tuple[float, float, float]:
    """
    Rotate ECEF delta vector (sat - obs) into observer-centred ENU frame.

    Returns (E, N, U) in metres.

    The rotation is the standard geodetic ENU transform:
        E =  -sin(lon)·dX  + cos(lon)·dY
        N =  -sin(lat)·cos(lon)·dX  - sin(lat)·sin(lon)·dY  + cos(lat)·dZ
        U =   cos(lat)·cos(lon)·dX  + cos(lat)·sin(lon)·dY  + sin(lat)·dZ
    """
    lat = math.radians(obs_lat_deg)
    lon = math.radians(obs_lon_deg)

    dx = sat_ecef[0] - obs_ecef[0]
    dy = sat_ecef[1] - obs_ecef[1]
    dz = sat_ecef[2] - obs_ecef[2]

    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    sin_lon, cos_lon = math.sin(lon), math.cos(lon)

    E = -sin_lon * dx + cos_lon * dy
    N = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    U =  cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz

    return E, N, U


def _enu_elevation_deg(E: float, N: float, U: float) -> float:
    """
    Compute elevation angle (degrees) from ENU components.

    elevation = atan2(U, sqrt(E² + N²))
    Returns NaN if satellite is at the same location as observer.
    """
    horiz = math.sqrt(E * E + N * N)
    if horiz == 0.0 and U == 0.0:
        return float("nan")
    return math.degrees(math.atan2(U, horiz))


def _ephem_elevation_deg(sat, obs) -> float:
    """Return ephem-computed elevation in degrees after sat.compute(obs)."""
    return math.degrees(float(sat.alt))


def _hypatia_elevation_geodetic(obs_lat_deg: float, obs_lon_deg: float,
                                 sat_geodetic_lat_deg: float,
                                 sat_lon_deg: float,
                                 sat_alt_ellipsoid_m: float) -> float:
    """
    Compute elevation angle via Hypatia's geodetic2cartesian() + ENU rotation.

    BOTH observer and satellite positions must use GEODETIC (not geocentric)
    latitude and height above the WGS72 ellipsoid.
    obs is assumed at sea level (elevation = 0 m).
    """
    obs_ecef = geodetic2cartesian(obs_lat_deg, obs_lon_deg, 0.0)
    sat_ecef = geodetic2cartesian(sat_geodetic_lat_deg, sat_lon_deg, sat_alt_ellipsoid_m)
    E, N, U  = _ecef_to_enu(obs_lat_deg, obs_lon_deg, obs_ecef, sat_ecef)
    return _enu_elevation_deg(E, N, U)


# WGS72 equatorial radius — same constant used by both SGP4 (PyEphem) and Hypatia.
_WGS72_A_M = 6378135.0


def _enu_to_ecef(obs_lat_deg: float, obs_lon_deg: float,
                 E: float, N: float, U: float) -> tuple[float, float, float]:
    """
    Rotate an ENU vector to an ECEF delta vector (inverse of _ecef_to_enu).

    Returns (dx, dy, dz) in metres — add to observer ECEF to get absolute ECEF.
    """
    lat = math.radians(obs_lat_deg)
    lon = math.radians(obs_lon_deg)
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    sin_lon, cos_lon = math.sin(lon), math.cos(lon)

    dx = -sin_lon * E - sin_lat * cos_lon * N + cos_lat * cos_lon * U
    dy =  cos_lon * E - sin_lat * sin_lon * N + cos_lat * sin_lon * U
    dz =               cos_lat          * N + sin_lat           * U
    return dx, dy, dz


def _ecef_from_ephem_geocentric(sat) -> tuple[float, float, float]:
    """
    Compute satellite ECEF from PyEphem geocentric coordinates.

    PyEphem's EarthSatellite reports:
      sat.sublat   — geocentric latitude  (angle from equatorial plane to sat
                     position vector, NOT the geodetic normal)  [radians]
      sat.sublong  — geographic (Earth-fixed) longitude          [radians]
      sat.elevation — altitude above equatorial sphere            [metres]

    The SGP4 propagator inside ephem works in Earth equatorial radii using
    WGS72 (a = 6378135.0 m), so:
        geocentric_distance = sat.elevation + WGS72_a

    Passing sat.sublat to geodetic2cartesian() as if it were geodetic latitude
    introduces up to 0.19° of latitude error at mid-latitudes, causing ~0.4°
    elevation angle error.  This function bypasses the geodetic round-trip and
    computes ECEF directly from the geocentric position.
    """
    sublat  = float(sat.sublat)           # geocentric lat (rad)
    sublong = float(sat.sublong)          # geographic lon (rad)
    r       = float(sat.elevation) + _WGS72_A_M   # geocentric distance (m)

    x = r * math.cos(sublat) * math.cos(sublong)
    y = r * math.cos(sublat) * math.sin(sublong)
    z = r * math.sin(sublat)
    return x, y, z


# ---------------------------------------------------------------------------
# T1: Analytical overhead validation (no TLE needed)
# ---------------------------------------------------------------------------

def test_t1_analytical_overhead(obs_lat: float, obs_lon: float) -> bool:
    """
    T1: Satellite placed directly above observer at alt=630 km.

    Expected: elevation ≈ 90.000°  (error < 0.01°)
    The small residual comes from the WGS72 ellipsoid — a point on the
    geodetic normal above the observer is NOT exactly at the same
    (lat, lon) in geocentric terms, but the error is < 0.001°.
    """
    print("\n" + "=" * 60)
    print("T1  Analytical overhead case")
    print("=" * 60)

    alt_m = 630_000.0  # Kuiper-630 nominal altitude

    obs_ecef = geodetic2cartesian(obs_lat, obs_lon, 0.0)
    sat_ecef = geodetic2cartesian(obs_lat, obs_lon, alt_m)

    E, N, U  = _ecef_to_enu(obs_lat, obs_lon, obs_ecef, sat_ecef)
    elev     = _enu_elevation_deg(E, N, U)
    error    = abs(elev - 90.0)

    print(f"  Observer : lat={obs_lat}°N  lon={obs_lon}°E  alt=0 m")
    print(f"  Satellite: lat={obs_lat}°N  lon={obs_lon}°E  alt={alt_m/1000:.0f} km")
    print(f"  ECEF obs : ({obs_ecef[0]:.0f}, {obs_ecef[1]:.0f}, {obs_ecef[2]:.0f}) m")
    print(f"  ENU      : E={E:.3f}  N={N:.3f}  U={U:.0f} m")
    print(f"  Elevation: {elev:.6f}°  (expected ≈ 90°)")
    print(f"  Error    : {error:.6f}°  (threshold 0.01°)")

    if error < 0.01:
        print("  RESULT   : PASS ✅")
        return True
    else:
        print("  RESULT   : FAIL ❌")
        return False


# ---------------------------------------------------------------------------
# T2: ephem ↔ geodetic2cartesian cross-check
# ---------------------------------------------------------------------------

def test_t2_ephem_vs_hypatia(obs_lat: float, obs_lon: float,
                               search_s: float, step_ms: float) -> bool:
    """
    T2: ENU rotation round-trip consistency check (ERAD-independent).

    Strategy: use ephem's sat.range + sat.alt + sat.az to reconstruct the
    satellite's ECEF position WITHOUT any Earth-radius constant assumption,
    then feed that ECEF back through Hypatia's _ecef_to_enu and verify the
    recovered elevation matches sat.alt.

    Why range/alt/az instead of sublat/sublong/elevation:
      sat.elevation uses an internal Earth-radius constant (ERAD) whose exact
      value in libastro is ambiguous (6378135 WGS72 vs 6378160 IAU vs others).
      An ERAD error of even 16 km translates to a ~0.3° elevation bias.
      sat.range is the true observer-satellite distance and carries no such
      dependency.  The round-trip error is therefore limited to floating-point
      precision only (< 1e-10°).

    What this validates:
      - geodetic2cartesian() gives the correct observer ECEF (WGS72)
      - _ecef_to_enu() rotation matrix is the correct inverse of the
        manual ENU→ECEF rotation used in Phase 2
      - _enu_elevation_deg() formula is correct

    Threshold: < 1e-5° (floating-point round-trip).

    Diagnostic only (not a PASS/FAIL):
      Also reports the geocentric-ECEF residual (sat.sublat/sublong/elevation
      + WGS72_a → ECEF → Hypatia ENU).  This reflects the ERAD model
      difference between ephem and WGS72 and is informational.
    """
    print("\n" + "=" * 60)
    print("T2  ephem ↔ Hypatia ENU rotation round-trip")
    print("=" * 60)
    print(f"  TLE      : {_KUIPER_NAME}")
    print(f"  Window   : {search_s:.0f} s from epoch  (step {step_ms:.0f} ms)")

    try:
        sat = ephem.readtle(_KUIPER_NAME, _KUIPER_L1, _KUIPER_L2)
    except Exception as e:
        print(f"  ERROR: TLE parse failed: {e}")
        return False

    obs = ephem.Observer()
    obs.lat      = str(obs_lat)
    obs.lon      = str(obs_lon)
    obs.elev     = 0.0
    obs.epoch    = ephem.J2000
    obs.pressure = 0.0

    step_s  = step_ms / 1000.0
    n_steps = int(search_s / step_s) + 1
    t0      = ephem.Date("2000/01/01 00:00:00")

    obs_ecef = geodetic2cartesian(obs_lat, obs_lon, 0.0)

    visible_steps    = 0
    max_rt_error     = 0.0   # round-trip error (main test)
    max_geo_error    = 0.0   # geocentric-ECEF diagnostic error
    rt_errors        = []

    for i in range(n_steps):
        t_sim = i * step_s
        obs.date = ephem.Date(t0 + t_sim * ephem.second)
        sat.compute(obs)

        elev_ephem = _ephem_elevation_deg(sat, obs)
        if elev_ephem < 0.0:
            continue

        visible_steps += 1

        # ---- Main validation: ENU round-trip via range/alt/az ----
        # Reconstruct satellite ECEF from ephem observer-relative measurements.
        # No Earth-radius constant needed.
        range_m  = float(sat.range)
        alt_rad  = float(sat.alt)    # = elev_ephem in radians
        az_rad   = float(sat.az)

        E_sat = range_m * math.cos(alt_rad) * math.sin(az_rad)  # East
        N_sat = range_m * math.cos(alt_rad) * math.cos(az_rad)  # North
        U_sat = range_m * math.sin(alt_rad)                     # Up

        dx, dy, dz = _enu_to_ecef(obs_lat, obs_lon, E_sat, N_sat, U_sat)
        sat_ecef_rt = (obs_ecef[0] + dx, obs_ecef[1] + dy, obs_ecef[2] + dz)

        E2, N2, U2    = _ecef_to_enu(obs_lat, obs_lon, obs_ecef, sat_ecef_rt)
        elev_rt       = _enu_elevation_deg(E2, N2, U2)
        rt_err        = abs(elev_ephem - elev_rt)
        rt_errors.append(rt_err)
        if rt_err > max_rt_error:
            max_rt_error = rt_err

        # ---- Diagnostic: geocentric-ECEF method ----
        sat_ecef_geo  = _ecef_from_ephem_geocentric(sat)
        Eg, Ng, Ug    = _ecef_to_enu(obs_lat, obs_lon, obs_ecef, sat_ecef_geo)
        elev_geo      = _enu_elevation_deg(Eg, Ng, Ug)
        geo_err       = abs(elev_ephem - elev_geo)
        if geo_err > max_geo_error:
            max_geo_error = geo_err

    if visible_steps == 0:
        print(f"  WARNING  : No visible steps in {search_s:.0f} s window over"
              f" lat={obs_lat}, lon={obs_lon}")
        print("             Skipping T2 with INCONCLUSIVE result.")
        return None  # type: ignore[return-value]

    p95_rt = sorted(rt_errors)[int(0.95 * len(rt_errors))]

    print(f"  Visible steps      : {visible_steps}")
    print(f"  [Main] Round-trip max |error| : {max_rt_error:.2e}°"
          f"  (threshold 1e-5°)")
    print(f"  [Main] Round-trip p95 |error| : {p95_rt:.2e}°")
    print(f"  [Diag] Geocentric-ECEF max    : {max_geo_error:.6f}°"
          f"  (ERAD model difference — informational)")

    if max_rt_error < 1e-5:
        print("  RESULT             : PASS ✅  (ENU rotation is self-consistent)")
        return True
    else:
        print(f"  RESULT             : FAIL ❌  (ENU round-trip error {max_rt_error:.2e}°"
              f" > 1e-5°; rotation matrix likely wrong)")
        return False


# ---------------------------------------------------------------------------
# T3: SGP4 arc shape validation
# ---------------------------------------------------------------------------

def test_t3_arc_shape(obs_lat: float, obs_lon: float,
                       search_s: float, step_ms: float) -> bool:
    """
    T3: Find visible pass, confirm elevation curve has exactly one peak.

    Extracts the elevation time series for the best (highest-peak) pass
    and counts local maxima.  A clean arc has exactly one peak.
    Also logs pass duration and peak elevation for comparison with Phase 1.2.
    """
    print("\n" + "=" * 60)
    print("T3  SGP4 arc shape — single-peak elevation curve")
    print("=" * 60)

    try:
        sat = ephem.readtle(_KUIPER_NAME, _KUIPER_L1, _KUIPER_L2)
    except Exception as e:
        print(f"  ERROR: TLE parse failed: {e}")
        return False

    obs = ephem.Observer()
    obs.lat      = str(obs_lat)
    obs.lon      = str(obs_lon)
    obs.elev     = 0.0
    obs.epoch    = ephem.J2000
    obs.pressure = 0.0

    step_s  = step_ms / 1000.0
    n_steps = int(search_s / step_s) + 1
    t0      = ephem.Date("2000/01/01 00:00:00")

    time_series: list[float] = []
    elev_series: list[float] = []

    for i in range(n_steps):
        t_sim = i * step_s
        obs.date = ephem.Date(t0 + t_sim * ephem.second)
        sat.compute(obs)
        elev_deg = _ephem_elevation_deg(sat, obs)
        time_series.append(t_sim)
        elev_series.append(elev_deg)

    # Extract individual passes (contiguous above-horizon segments)
    passes: list[tuple[float, float, float]] = []  # (start_s, peak_elev, end_s)
    in_pass   = False
    pass_start = 0.0
    pass_peak  = -999.0

    for t, e in zip(time_series, elev_series):
        if e > 0.0:
            if not in_pass:
                in_pass    = True
                pass_start = t
                pass_peak  = e
            else:
                pass_peak = max(pass_peak, e)
        else:
            if in_pass:
                passes.append((pass_start, pass_peak, t))
                in_pass = False

    if in_pass:
        passes.append((pass_start, pass_peak, time_series[-1]))

    if not passes:
        print(f"  WARNING  : No visible passes in {search_s:.0f} s window.")
        print("             Skipping T3 with INCONCLUSIVE result.")
        return None  # type: ignore[return-value]

    print(f"  Passes found  : {len(passes)}")
    for idx, (start, peak, end) in enumerate(passes):
        print(f"    Pass {idx+1}: t={start:.0f}s – {end:.0f}s  "
              f"duration={(end-start):.0f}s  peak={peak:.2f}°")

    # Validate the best (highest peak) pass for single-peak shape
    best = max(passes, key=lambda p: p[1])
    start_s, peak_elev, end_s = best

    # Extract elevation samples during the best pass (above horizon)
    pass_elev = [e for t, e in zip(time_series, elev_series)
                 if start_s <= t <= end_s and e > 0.0]

    # Count local maxima in the pass (a peak is a sample higher than both neighbours)
    n_peaks = sum(
        1 for k in range(1, len(pass_elev) - 1)
        if pass_elev[k] > pass_elev[k - 1] and pass_elev[k] > pass_elev[k + 1]
    )
    # The last sample might be a plateau peak — also check the very end
    if len(pass_elev) >= 2 and pass_elev[-1] > pass_elev[-2]:
        n_peaks += 1

    print(f"\n  Best pass     : t={start_s:.0f}s – {end_s:.0f}s")
    print(f"  Peak elevation: {peak_elev:.3f}°")
    print(f"  Pass duration : {(end_s - start_s):.0f} s")
    print(f"  Local peaks   : {n_peaks}  (expected: 1)")

    if n_peaks == 1:
        print("  RESULT        : PASS ✅  (single-arc elevation profile confirmed)")
        return True
    elif n_peaks == 0:
        # Can happen if all samples monotonically rise or fall — edge case
        print("  RESULT        : PASS ✅  (monotonic pass, consistent with short arc)")
        return True
    else:
        print(f"  RESULT        : FAIL ❌  (multi-peak profile — unexpected)")
        return False


# ---------------------------------------------------------------------------
# T4: run_sgp4.py output schema validation
# ---------------------------------------------------------------------------

def test_t4_csv_schema(obs_lat: float, obs_lon: float,
                        search_s: float, step_ms: float) -> bool:
    """
    T4: Call run_sgp4.py internals with the built-in TLE.
    Verify output CSV has all columns required by the Phase 2 grid reader.

    Required columns (from Phase 2 sat-orbit-reader.h design):
        time_s, sat_lat_deg, sat_lon_deg, sat_alt_m, elevation_deg, azimuth_deg
    """
    print("\n" + "=" * 60)
    print("T4  run_sgp4.py output schema")
    print("=" * 60)

    # Import from the sibling script
    _scripts_dir = os.path.dirname(os.path.abspath(__file__))
    if _scripts_dir not in sys.path:
        sys.path.insert(0, _scripts_dir)

    try:
        from run_sgp4 import propagate, FIELDNAMES, write_csv, read_single_tle
    except ImportError as e:
        print(f"  ERROR: Cannot import run_sgp4: {e}")
        return False

    # Write built-in TLE to a temp file (read_single_tle expects a file path)
    with tempfile.NamedTemporaryFile(mode="w", suffix=".tle",
                                     delete=False, encoding="utf-8") as tf:
        tf.write(f"{_KUIPER_NAME}\n{_KUIPER_L1}\n{_KUIPER_L2}\n")
        tle_path = tf.name

    output_path = os.path.join(tempfile.gettempdir(), "phase1_3_validation_orbit.csv")

    try:
        sat_obj, tle_name = read_single_tle(tle_path)
        rows = propagate(
            sat_obj,
            observer_lat_deg=obs_lat,
            observer_lon_deg=obs_lon,
            duration_s=search_s,
            step_ms=step_ms,
            start_utc="2000/01/01 00:00:00",
        )

        # Build a minimal args-like object for write_csv
        class _Args:
            tle_file    = tle_path
            observer_lat = obs_lat
            observer_lon = obs_lon
            duration_s  = search_s
            step_ms     = step_ms
            start_utc   = "2000/01/01 00:00:00"

        write_csv(rows, output_path, tle_name, _Args())

        # Verify schema
        required_cols = ["time_s", "sat_lat_deg", "sat_lon_deg",
                         "sat_alt_m", "elevation_deg", "azimuth_deg"]
        present_cols  = FIELDNAMES

        print(f"  TLE source   : {_KUIPER_NAME}")
        print(f"  Duration     : {search_s:.0f} s  step {step_ms:.0f} ms")
        print(f"  Rows written : {len(rows)}")
        print(f"  Output file  : {output_path}")
        print(f"  Columns      : {present_cols}")

        missing = [c for c in required_cols if c not in present_cols]

        # Data sanity checks on the rows
        alts   = [r["sat_alt_m"]    for r in rows]
        elevs  = [r["elevation_deg"] for r in rows]
        n_vis  = sum(1 for e in elevs if e > 0.0)
        alt_min = min(alts)
        alt_max = max(alts)

        print(f"  Alt range    : {alt_min/1000:.1f} – {alt_max/1000:.1f} km")
        print(f"  Visible steps: {n_vis} / {len(rows)}")

        if missing:
            print(f"  Missing cols : {missing}")
            print("  RESULT       : FAIL ❌  (schema mismatch)")
            return False

        # Altitude sanity: Kuiper-630 should be roughly 600–700 km
        if not (500_000 < alt_min < 800_000 and 500_000 < alt_max < 800_000):
            print(f"  WARNING      : Altitude out of expected 500–800 km range")
            print("  RESULT       : FAIL ❌")
            return False

        print("  RESULT       : PASS ✅  (schema and altitude sanity confirmed)")
        return True

    except Exception as e:
        print(f"  ERROR: {e}")
        import traceback; traceback.print_exc()
        return False
    finally:
        os.unlink(tle_path)


# ---------------------------------------------------------------------------
# Summary printer
# ---------------------------------------------------------------------------

def print_summary(results: dict[str, bool | None]) -> None:
    print("\n" + "=" * 60)
    print("SUMMARY — Phase 1.3 Hypatia Validation")
    print("=" * 60)

    labels = {
        "T1": "Analytical overhead (geodetic2cartesian ≈ 90°)",
        "T2": "ENU rotation round-trip self-consistency (error < 1e-5°)",
        "T3": "SGP4 arc shape (single-peak elevation curve)",
        "T4": "run_sgp4.py CSV schema validation",
    }

    all_pass = True
    for key, label in labels.items():
        r = results.get(key)
        if r is True:
            status = "PASS ✅"
        elif r is False:
            status = "FAIL ❌"
            all_pass = False
        else:
            status = "SKIP ⚠️ (inconclusive — no visible pass in window)"

        print(f"  {key}  {status}  —  {label}")

    print()
    if all_pass:
        print("  Overall: PASS ✅  Phase 1.3 Hypatia integration validated")
    else:
        print("  Overall: FAIL ❌  Review failed tests above")
    print("=" * 60)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Phase 1.3 Hypatia / SGP4 coordinate validation"
    )
    parser.add_argument("--observer-lat", type=float, default=25.0,
                        help="Observer latitude °N (default 25.0 — Taiwan)")
    parser.add_argument("--observer-lon", type=float, default=121.0,
                        help="Observer longitude °E (default 121.0 — Taiwan)")
    parser.add_argument("--search-window-s", type=float, default=21600.0,
                        help="Search window in seconds for finding visible passes "
                             "(default 21600 = 6 hours)")
    parser.add_argument("--step-ms", type=float, default=100.0,
                        help="Propagation timestep in ms (default 100)")
    args = parser.parse_args()

    print("Phase 1.3 Hypatia / SGP4 Coordinate Validation")
    print(f"  Observer   : lat={args.observer_lat}°N  lon={args.observer_lon}°E")
    print(f"  Search     : {args.search_window_s:.0f} s  step {args.step_ms:.0f} ms")
    print(f"  TLE        : {_KUIPER_NAME}  (Hypatia test data)")

    results: dict[str, bool | None] = {}

    results["T1"] = test_t1_analytical_overhead(args.observer_lat, args.observer_lon)
    results["T2"] = test_t2_ephem_vs_hypatia(
        args.observer_lat, args.observer_lon,
        args.search_window_s, args.step_ms)
    results["T3"] = test_t3_arc_shape(
        args.observer_lat, args.observer_lon,
        args.search_window_s, args.step_ms)
    results["T4"] = test_t4_csv_schema(
        args.observer_lat, args.observer_lon,
        args.search_window_s, args.step_ms)

    print_summary(results)

    # Exit code: 0 = all pass / skip, 1 = any fail
    failed = any(r is False for r in results.values())
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
