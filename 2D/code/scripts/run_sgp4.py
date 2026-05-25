"""
run_sgp4.py — Phase 1.3 / Phase 2 SGP4 orbit pre-computation script.

Uses PyEphem (same library as Hypatia satgenpy) to propagate a satellite's
TLE over a specified duration and write per-timestep position data to CSV.

=== Modes ===

--mode single (default, Phase 1.3 / 2.0):
    Propagate ONE satellite from a TLE file for --duration-s seconds.
    Output: one CSV at --output.

    Usage:
        python run_sgp4.py \\
            --mode single \\
            --tle-file iridium.txt \\
            --observer-lat 35.676 \\
            --observer-lon 139.650 \\
            --duration-s 770 \\
            --step-ms 100 \\
            --start-utc "2000/01/01 00:00:00" \\
            --output satellite_orbit.csv

--mode sequence (Phase 2.1):
    Scan ALL satellites in a multi-TLE file for passes with
    peak elevation >= --min-peak-elev-deg.  Select the --pair-index-th
    qualifying pass (sat[i]) and the very next qualifying pass (sat[i+1])
    and propagate both over a shared time window.

    Outputs two CSVs with a COMMON 0-based time_s axis so that Phase 2.2's
    C++ dual-satellite loop can index them in sync.

    Usage:
        python run_sgp4.py \\
            --mode sequence \\
            --tle-file 2D/data/tle/iridium.txt \\
            --observer-lat 35.676 \\
            --observer-lon 139.650 \\
            --start-utc "2000/01/01 00:00:00" \\
            --search-window-s 7200 \\
            --scan-step-s 10 \\
            --min-peak-elev-deg 50 \\
            --pair-index 0 \\
            --margin-s 60 \\
            --step-ms 100 \\
            --output-sat-i orbit_sat_i.csv \\
            --output-sat-i1 orbit_sat_i1.csv

Output schema (all modes):
    time_s        — seconds since WINDOW START (0-indexed)
    sat_lat_deg   — satellite geodetic latitude (degrees N, Bowring-converted)
    sat_lon_deg   — satellite sub-point longitude (degrees E)
    sat_alt_m     — altitude above WGS72 ellipsoid (metres)
    elevation_deg — elevation angle seen from the observer (degrees)
    azimuth_deg   — azimuth angle from observer (degrees, N=0)

Dependencies:
    pip install ephem
"""

import argparse
import csv
import json
import math
import sys

# ---------------------------------------------------------------------------
# WGS72 constants — PyEphem's SGP4 uses WGS72 internally
# ---------------------------------------------------------------------------
_WGS72_A_M = 6378135.0
_WGS72_F   = 1.0 / 298.26
_WGS72_E2  = 2 * _WGS72_F - _WGS72_F * _WGS72_F   # first eccentricity squared


def _geocentric_to_geodetic(geocentric_lat_deg: float,
                             alt_above_equatorial_sphere_m: float
                             ) -> tuple:
    """
    Convert PyEphem geocentric satellite coordinates to WGS72 geodetic.

    PyEphem's EarthSatellite reports:
      sat.sublat   — geocentric latitude (angle from equatorial plane to the
                     satellite position vector, NOT the geodetic normal)
      sat.elevation — altitude above the WGS72 equatorial sphere
                      = geocentric_distance - WGS72_a

    Phase 2's C++ grid code sets up an ENU frame using geodetic lat/lon, so
    the CSV must carry geodetic coordinates for correct footprint projection.

    Returns: (geodetic_lat_deg, alt_above_WGS72_ellipsoid_m)
    """
    phi_c = math.radians(geocentric_lat_deg)
    r     = alt_above_equatorial_sphere_m + _WGS72_A_M   # geocentric distance

    # Satellite ECEF from geocentric spherical coordinates
    p = r * math.cos(phi_c)   # equatorial distance
    z = r * math.sin(phi_c)   # polar distance

    # Iterative Bowring's method: ECEF → geodetic latitude (converges in 2–3 steps)
    lat = math.atan2(z, p * (1.0 - _WGS72_E2))   # initial guess
    for _ in range(6):
        sin_lat = math.sin(lat)
        v       = _WGS72_A_M / math.sqrt(1.0 - _WGS72_E2 * sin_lat * sin_lat)
        lat_new = math.atan2(z + _WGS72_E2 * v * sin_lat, p)
        if abs(lat_new - lat) < 1e-13:
            lat = lat_new
            break
        lat = lat_new

    # Height above WGS72 ellipsoid
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    v = _WGS72_A_M / math.sqrt(1.0 - _WGS72_E2 * sin_lat * sin_lat)
    if abs(cos_lat) > 1e-6:
        alt_ellipsoid = p / cos_lat - v
    else:
        alt_ellipsoid = abs(z) / abs(sin_lat) - v * (1.0 - _WGS72_E2)

    return math.degrees(lat), alt_ellipsoid


try:
    import ephem
except ImportError:
    sys.exit(
        "ERROR: PyEphem not installed.  Run: pip install ephem\n"
        "Hypatia uses ephem; install it in the same virtualenv as Hypatia."
    )


# ---------------------------------------------------------------------------
# TLE reading
# ---------------------------------------------------------------------------

def read_single_tle(filename: str):
    """
    Read a standard 3-line TLE from a file (first satellite only).

    Returns: (ephem.EarthSatellite, name_str)
    """
    with open(filename, "r") as f:
        lines = [l.strip() for l in f.readlines() if l.strip()]

    if len(lines) < 3:
        raise ValueError(
            f"TLE file must have at least 3 non-empty lines "
            f"(name, line1, line2).  Got {len(lines)} lines."
        )

    name  = lines[0]
    line1 = lines[1]
    line2 = lines[2]

    if not line1.startswith("1 ") or not line2.startswith("2 "):
        raise ValueError(
            "Lines 1 and 2 must start with '1 ' and '2 ' respectively."
        )

    sat = ephem.readtle(name, line1, line2)
    return sat, name


def read_all_tles(filename: str) -> list:
    """
    Read ALL 3-line TLE groups from a multi-TLE file.

    Handles files where each group is: name line, TLE line 1, TLE line 2.
    Lines starting with '#' and blank lines are skipped.

    Returns: list of (ephem.EarthSatellite, name_str) tuples
    """
    with open(filename, "r") as f:
        lines = [l.strip() for l in f.readlines()
                 if l.strip() and not l.strip().startswith("#")]

    sats = []
    i = 0
    while i + 2 < len(lines):
        name  = lines[i]
        line1 = lines[i + 1]
        line2 = lines[i + 2]
        if line1.startswith("1 ") and line2.startswith("2 "):
            sat = ephem.readtle(name, line1, line2)
            sats.append((sat, name))
            i += 3
        else:
            # Line not in expected position; advance and retry
            i += 1

    return sats


# ---------------------------------------------------------------------------
# Pass scanning for sequence mode
# ---------------------------------------------------------------------------

def scan_passes(sat, observer_lat_deg: float, observer_lon_deg: float,
                search_window_s: float, scan_step_s: float,
                start_utc: str, min_visible_elev_deg: float = 0.0) -> list:
    """
    Scan a satellite's elevation over [0, search_window_s] at scan_step_s
    resolution and identify discrete passes above min_visible_elev_deg.

    A pass begins when elevation crosses above the threshold and ends when it
    drops back below.  The peak elevation within each pass is recorded.

    Returns: list of dicts, each with:
        t_enter_s   — first timestep above threshold (seconds from start_utc)
        t_peak_s    — timestep of maximum elevation in the pass
        t_exit_s    — last timestep above threshold
        peak_elev_deg — maximum elevation reached during the pass
    """
    obs = ephem.Observer()
    obs.lat      = str(observer_lat_deg)
    obs.lon      = str(observer_lon_deg)
    obs.elev     = 0.0
    obs.epoch    = ephem.J2000
    obs.pressure = 0.0   # disable atmospheric refraction

    t0     = ephem.Date(start_utc)
    passes = []

    in_pass    = False
    t_enter    = None
    t_peak     = None
    peak_elev  = -999.0

    t = 0.0
    while t <= search_window_s:
        obs.date = ephem.Date(t0 + t * ephem.second)
        sat.compute(obs)
        elev = math.degrees(float(sat.alt))

        if not in_pass and elev >= min_visible_elev_deg:
            # Pass entry
            in_pass   = True
            t_enter   = t
            peak_elev = elev
            t_peak    = t
        elif in_pass and elev >= min_visible_elev_deg:
            # Still in pass — update peak
            if elev > peak_elev:
                peak_elev = elev
                t_peak    = t
        elif in_pass and elev < min_visible_elev_deg:
            # Pass exit
            passes.append({
                "t_enter_s":    t_enter,
                "t_peak_s":     t_peak,
                "t_exit_s":     t - scan_step_s,   # last step above threshold
                "peak_elev_deg": peak_elev,
            })
            in_pass   = False
            t_enter   = None
            t_peak    = None
            peak_elev = -999.0

        t += scan_step_s

    # Close any pass still open at window boundary
    if in_pass:
        passes.append({
            "t_enter_s":    t_enter,
            "t_peak_s":     t_peak,
            "t_exit_s":     search_window_s,
            "peak_elev_deg": peak_elev,
        })

    return passes


def find_qualifying_passes(all_sats: list,
                           observer_lat_deg: float,
                           observer_lon_deg: float,
                           search_window_s: float,
                           scan_step_s: float,
                           start_utc: str,
                           min_peak_elev_deg: float,
                           min_visible_elev_deg: float = 0.0) -> list:
    """
    Scan all satellites and collect passes whose peak elevation meets the
    threshold.  Sort the result by t_peak_s to produce a chronological
    serving sequence.

    Returns: sorted list of dicts, each with sat, name, t_enter_s, t_peak_s,
             t_exit_s, peak_elev_deg fields.
    """
    all_passes = []
    total = len(all_sats)
    for idx, (sat, name) in enumerate(all_sats):
        print(f"\r  Scanning {idx+1}/{total}: {name:<30}", end="", flush=True)
        passes = scan_passes(sat, observer_lat_deg, observer_lon_deg,
                             search_window_s, scan_step_s, start_utc,
                             min_visible_elev_deg)
        for p in passes:
            if p["peak_elev_deg"] >= min_peak_elev_deg:
                p["sat"]  = sat
                p["name"] = name
                all_passes.append(p)

    print()   # newline after progress bar

    all_passes.sort(key=lambda x: x["t_peak_s"])
    return all_passes


# ---------------------------------------------------------------------------
# Window propagation for sequence mode
# ---------------------------------------------------------------------------

def propagate_window(sat,
                     observer_lat_deg: float,
                     observer_lon_deg: float,
                     t_abs_start_s: float,
                     t_abs_end_s: float,
                     step_ms: float,
                     start_utc: str) -> list:
    """
    Propagate satellite over an absolute time window [t_abs_start_s, t_abs_end_s].

    IMPORTANT: time_s in each output row is 0-BASED from t_abs_start_s so that
    two CSVs covering the same window share a common time axis for Phase 2.2's
    C++ dual-satellite loop.

    Returns: list of dicts with same schema as propagate().
    """
    obs = ephem.Observer()
    obs.lat      = str(observer_lat_deg)
    obs.lon      = str(observer_lon_deg)
    obs.elev     = 0.0
    obs.epoch    = ephem.J2000
    obs.pressure = 0.0

    step_s  = step_ms / 1000.0
    t0      = ephem.Date(start_utc)
    rows    = []

    t_rel = 0.0
    t_abs = t_abs_start_s
    while t_abs <= t_abs_end_s + 1e-9:
        obs.date = ephem.Date(t0 + t_abs * ephem.second)
        sat.compute(obs)

        sat_lat_deg, sat_alt_m = _geocentric_to_geodetic(
            math.degrees(float(sat.sublat)),
            float(sat.elevation),
        )
        sat_lon_deg   = math.degrees(float(sat.sublong))
        elevation_deg = math.degrees(float(sat.alt))
        azimuth_deg   = math.degrees(float(sat.az))

        rows.append({
            "time_s":        round(t_rel, 6),
            "sat_lat_deg":   round(sat_lat_deg, 9),
            "sat_lon_deg":   round(sat_lon_deg, 9),
            "sat_alt_m":     round(sat_alt_m, 3),
            "elevation_deg": round(elevation_deg, 6),
            "azimuth_deg":   round(azimuth_deg, 6),
        })

        t_rel += step_s
        t_abs += step_s

    return rows


# ---------------------------------------------------------------------------
# Single-satellite propagation (original, unchanged)
# ---------------------------------------------------------------------------

def propagate(sat,
              observer_lat_deg: float,
              observer_lon_deg: float,
              duration_s: float,
              step_ms: float,
              start_utc: str) -> list:
    """
    Propagate the satellite orbit and compute per-timestep position/elevation.

    time_s in each row is 0-based from start_utc.
    Returns a list of dicts, one per timestep.
    """
    obs = ephem.Observer()
    obs.lat      = str(observer_lat_deg)
    obs.lon      = str(observer_lon_deg)
    obs.elev     = 0.0
    obs.epoch    = ephem.J2000
    obs.pressure = 0.0

    step_s  = step_ms / 1000.0
    n_steps = int(duration_s / step_s) + 1
    t0      = ephem.Date(start_utc)

    rows = []
    for i in range(n_steps):
        t_sim_s = i * step_s
        obs.date = ephem.Date(t0 + t_sim_s * ephem.second)
        sat.compute(obs)

        sat_lat_deg, sat_alt_m = _geocentric_to_geodetic(
            math.degrees(float(sat.sublat)),
            float(sat.elevation),
        )
        sat_lon_deg   = math.degrees(float(sat.sublong))
        elevation_deg = math.degrees(float(sat.alt))
        azimuth_deg   = math.degrees(float(sat.az))

        rows.append({
            "time_s":        round(t_sim_s, 6),
            "sat_lat_deg":   round(sat_lat_deg, 9),
            "sat_lon_deg":   round(sat_lon_deg, 9),
            "sat_alt_m":     round(sat_alt_m, 3),
            "elevation_deg": round(elevation_deg, 6),
            "azimuth_deg":   round(azimuth_deg, 6),
        })

    return rows


# ---------------------------------------------------------------------------
# CSV output
# ---------------------------------------------------------------------------

FIELDNAMES = [
    "time_s", "sat_lat_deg", "sat_lon_deg",
    "sat_alt_m", "elevation_deg", "azimuth_deg",
]


def write_csv(rows: list, output_path: str, sat_name: str,
              args_or_meta) -> None:
    """
    Write propagation results to CSV with a self-documenting metadata header.

    args_or_meta can be either an argparse.Namespace (single mode, backward
    compatible) or a plain dict (sequence mode).
    """
    if isinstance(args_or_meta, dict):
        meta = args_or_meta
    else:
        # argparse.Namespace — convert to dict for uniform access
        meta = vars(args_or_meta)

    with open(output_path, "w", newline="") as f:
        f.write("# satellite_orbit.csv — generated by run_sgp4.py\n")
        f.write("# coord_note: sat_lat_deg is GEODETIC (WGS72), converted from ephem geocentric\n")
        f.write("# coord_note: sat_alt_m   is height above WGS72 ellipsoid\n")
        f.write("# coord_note: elevation_deg/azimuth_deg are from ephem observer (correct)\n")
        f.write(f"# satellite : {sat_name}\n")
        f.write(f"# tle_file  : {meta.get('tle_file', 'N/A')}\n")
        f.write(f"# start_utc : {meta.get('start_utc', 'N/A')}\n")
        f.write(f"# observer  : lat={meta.get('observer_lat', 'N/A')} "
                f"lon={meta.get('observer_lon', 'N/A')}\n")
        if "window_start_s" in meta:
            f.write(f"# window_start_s : {meta['window_start_s']}\n")
            f.write(f"# window_end_s   : {meta['window_end_s']}\n")
        else:
            f.write(f"# duration_s: {meta.get('duration_s', 'N/A')}\n")
        f.write(f"# step_ms   : {meta.get('step_ms', 'N/A')}\n")
        f.write(f"# rows      : {len(rows)}\n")

        writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)

    print(f"Wrote {len(rows)} rows → {output_path}")


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="SGP4 orbit pre-computation for Phase 2 d×d ROI simulation."
    )

    # ---- mode ---------------------------------------------------------------
    parser.add_argument(
        "--mode", choices=["single", "sequence"], default="single",
        help="'single': propagate one satellite (Phase 1.3/2.0). "
             "'sequence': find consecutive satellite pair from multi-TLE file (Phase 2.1)."
    )

    # ---- common args --------------------------------------------------------
    parser.add_argument(
        "--tle-file", required=True,
        help="Path to TLE file.  In 'single' mode: first 3 lines are used.  "
             "In 'sequence' mode: all satellite groups are scanned."
    )
    parser.add_argument(
        "--observer-lat", type=float, default=35.676,
        help="Observer latitude (degrees N, default: 35.676 — Tokyo)."
    )
    parser.add_argument(
        "--observer-lon", type=float, default=139.650,
        help="Observer longitude (degrees E, default: 139.650 — Tokyo)."
    )
    parser.add_argument(
        "--step-ms", type=float, default=100.0,
        help="Timestep size in milliseconds (default: 100 ms)."
    )
    parser.add_argument(
        "--start-utc", type=str, default="2000/01/01 00:00:00",
        help="UTC start time in ephem format 'YYYY/MM/DD HH:MM:SS' "
             "(default matches Iridium-NEXT TLE epoch 2000/01/01)."
    )

    # ---- single mode args ---------------------------------------------------
    parser.add_argument(
        "--duration-s", type=float, default=770.0,
        help="[single mode] Simulation duration in seconds (default: 770)."
    )
    parser.add_argument(
        "--output", type=str, default="satellite_orbit.csv",
        help="[single mode] Output CSV file path."
    )

    # ---- sequence mode args -------------------------------------------------
    parser.add_argument(
        "--search-window-s", type=float, default=7200.0,
        help="[sequence mode] How many seconds from start_utc to scan for passes "
             "(default: 7200 = 2 hours, covers ~1.2 Iridium orbital periods)."
    )
    parser.add_argument(
        "--scan-step-s", type=float, default=10.0,
        help="[sequence mode] Scan resolution in seconds (default: 10 s). "
             "Finer steps give more accurate t_enter/t_exit but take longer."
    )
    parser.add_argument(
        "--min-peak-elev-deg", type=float, default=50.0,
        help="[sequence mode] Minimum peak elevation for a pass to qualify "
             "(default: 50°; passes below this are too low for good SNR)."
    )
    parser.add_argument(
        "--pair-index", type=int, default=0,
        help="[sequence mode] Which qualifying pass to treat as sat[i] "
             "(0 = first, 1 = second, ...).  By default sat[i+1] is the "
             "immediately next qualifying pass.  Use --pair-i1-index to "
             "override and pick a non-adjacent pass."
    )
    parser.add_argument(
        "--pair-i1-index", type=int, default=-1,
        help="[sequence mode] Explicit index for sat[i+1] in the qualifying "
             "pass list.  If omitted (-1), defaults to pair-index + 1.  "
             "Use this to select same-plane consecutive satellites when they "
             "are not adjacent in the sorted list (e.g., --pair-index 1 "
             "--pair-i1-index 3 selects sat 45 and sat 44 from Iridium-66)."
    )
    parser.add_argument(
        "--margin-s", type=float, default=60.0,
        help="[sequence mode] Extra seconds added on each side of the combined "
             "window so that C++ can observe the satellites before/after the "
             "ROI coverage (default: 60 s)."
    )
    parser.add_argument(
        "--output-sat-i", type=str, default="orbit_sat_i.csv",
        help="[sequence mode] Output CSV for sat[i]."
    )
    parser.add_argument(
        "--output-sat-i1", type=str, default="orbit_sat_i1.csv",
        help="[sequence mode] Output CSV for sat[i+1]."
    )

    args = parser.parse_args()

    # =========================================================================
    # SINGLE MODE
    # =========================================================================
    if args.mode == "single":
        print(f"Loading TLE from: {args.tle_file}")
        sat, tle_name = read_single_tle(args.tle_file)
        print(f"  Satellite: {tle_name}")

        print(f"Propagating {args.duration_s}s at {args.step_ms}ms steps...")
        rows = propagate(
            sat,
            observer_lat_deg=args.observer_lat,
            observer_lon_deg=args.observer_lon,
            duration_s=args.duration_s,
            step_ms=args.step_ms,
            start_utc=args.start_utc,
        )

        elev_vals = [r["elevation_deg"] for r in rows]
        max_elev  = max(elev_vals)
        visible   = sum(1 for e in elev_vals if e > 0.0)
        print(f"  max elevation : {max_elev:.2f}°")
        print(f"  visible steps : {visible} / {len(rows)}"
              f" ({100*visible/len(rows):.1f}%)")

        write_csv(rows, args.output, tle_name, args)

    # =========================================================================
    # SEQUENCE MODE
    # =========================================================================
    else:
        print(f"=== Phase 2.1 Sequence Mode ===")
        print(f"Loading all TLEs from: {args.tle_file}")
        all_sats = read_all_tles(args.tle_file)
        print(f"  Loaded {len(all_sats)} satellites")

        print(f"\nScanning {args.search_window_s}s window at {args.scan_step_s}s steps "
              f"for passes with peak elevation >= {args.min_peak_elev_deg}°...")
        qualifying = find_qualifying_passes(
            all_sats,
            observer_lat_deg=args.observer_lat,
            observer_lon_deg=args.observer_lon,
            search_window_s=args.search_window_s,
            scan_step_s=args.scan_step_s,
            start_utc=args.start_utc,
            min_peak_elev_deg=args.min_peak_elev_deg,
            min_visible_elev_deg=0.0,
        )

        if len(qualifying) == 0:
            sys.exit(
                f"ERROR: No qualifying passes found with peak elevation "
                f">= {args.min_peak_elev_deg}° in {args.search_window_s}s window.\n"
                f"Try reducing --min-peak-elev-deg or increasing --search-window-s."
            )

        # Resolve sat[i+1] index
        i1_idx = args.pair_i1_index if args.pair_i1_index >= 0 else args.pair_index + 1

        # Print full list of qualifying passes
        print(f"\nFound {len(qualifying)} qualifying passes (sorted by peak time):")
        print(f"  {'#':>3}  {'Satellite':<30}  {'enter_s':>8}  {'peak_s':>8}  "
              f"{'exit_s':>8}  {'peak_elev':>10}")
        print("  " + "-" * 75)
        for idx, p in enumerate(qualifying):
            marker = " ← sat[i]" if idx == args.pair_index else \
                     " ← sat[i+1]" if idx == i1_idx else ""
            truncated = "  [peak before epoch]" if p["t_peak_s"] <= args.scan_step_s else ""
            print(f"  {idx:>3}  {p['name']:<30}  {p['t_enter_s']:>8.1f}  "
                  f"{p['t_peak_s']:>8.1f}  {p['t_exit_s']:>8.1f}  "
                  f"{p['peak_elev_deg']:>9.2f}°{marker}{truncated}")

        # Validate pair selection
        if args.pair_index >= len(qualifying):
            sys.exit(
                f"ERROR: --pair-index {args.pair_index} out of range "
                f"(found {len(qualifying)} qualifying passes, valid range 0–{len(qualifying)-1})."
            )
        if i1_idx >= len(qualifying):
            sys.exit(
                f"ERROR: sat[i+1] index {i1_idx} out of range "
                f"(found {len(qualifying)} qualifying passes, valid range 0–{len(qualifying)-1}).\n"
                f"Increase --search-window-s or adjust --pair-i1-index."
            )
        if i1_idx == args.pair_index:
            sys.exit("ERROR: --pair-index and --pair-i1-index must be different.")

        pass_i  = qualifying[args.pair_index]
        pass_i1 = qualifying[i1_idx]

        # Combined time window covering both satellites plus margins
        win_start_s = min(pass_i["t_enter_s"],  pass_i1["t_enter_s"])  - args.margin_s
        win_end_s   = max(pass_i["t_exit_s"],   pass_i1["t_exit_s"])   + args.margin_s
        win_start_s = max(win_start_s, 0.0)   # clamp to search window start

        print(f"\nSelected pair:")
        print(f"  sat[i]  : {pass_i['name']}  (peak at t={pass_i['t_peak_s']:.1f}s, "
              f"elev={pass_i['peak_elev_deg']:.2f}°)")
        print(f"  sat[i+1]: {pass_i1['name']}  (peak at t={pass_i1['t_peak_s']:.1f}s, "
              f"elev={pass_i1['peak_elev_deg']:.2f}°)")
        print(f"\nCombined window: [{win_start_s:.1f}s, {win_end_s:.1f}s] "
              f"(duration {win_end_s - win_start_s:.1f}s)")

        # Propagate sat[i] over combined window
        print(f"\nPropagating sat[i] ({pass_i['name']}) over combined window "
              f"at {args.step_ms}ms steps...")
        rows_i = propagate_window(
            pass_i["sat"],
            observer_lat_deg=args.observer_lat,
            observer_lon_deg=args.observer_lon,
            t_abs_start_s=win_start_s,
            t_abs_end_s=win_end_s,
            step_ms=args.step_ms,
            start_utc=args.start_utc,
        )

        # Propagate sat[i+1] over same combined window (same 0-based time axis)
        print(f"Propagating sat[i+1] ({pass_i1['name']}) over combined window "
              f"at {args.step_ms}ms steps...")
        rows_i1 = propagate_window(
            pass_i1["sat"],
            observer_lat_deg=args.observer_lat,
            observer_lon_deg=args.observer_lon,
            t_abs_start_s=win_start_s,
            t_abs_end_s=win_end_s,
            step_ms=args.step_ms,
            start_utc=args.start_utc,
        )

        # Write CSVs
        meta_i = {
            "tle_file":      args.tle_file,
            "start_utc":     args.start_utc,
            "observer_lat":  args.observer_lat,
            "observer_lon":  args.observer_lon,
            "window_start_s": win_start_s,
            "window_end_s":   win_end_s,
            "step_ms":       args.step_ms,
        }
        meta_i1 = dict(meta_i)

        write_csv(rows_i,  args.output_sat_i,  pass_i["name"],  meta_i)
        write_csv(rows_i1, args.output_sat_i1, pass_i1["name"], meta_i1)

        # Write sequence_summary.json for Phase 2.3 overlap detection
        summary = {
            "mode":              "sequence",
            "observer_lat_deg":  args.observer_lat,
            "observer_lon_deg":  args.observer_lon,
            "start_utc":         args.start_utc,
            "window_start_s":    win_start_s,
            "window_end_s":      win_end_s,
            "step_ms":           args.step_ms,
            "sat_i": {
                "name":          pass_i["name"],
                "t_enter_s":     pass_i["t_enter_s"],
                "t_peak_s":      pass_i["t_peak_s"],
                "t_exit_s":      pass_i["t_exit_s"],
                "peak_elev_deg": pass_i["peak_elev_deg"],
                # 0-based time within combined window
                "local_enter_s": pass_i["t_enter_s"] - win_start_s,
                "local_peak_s":  pass_i["t_peak_s"]  - win_start_s,
                "local_exit_s":  pass_i["t_exit_s"]  - win_start_s,
                "csv":           args.output_sat_i,
            },
            "sat_i1": {
                "name":          pass_i1["name"],
                "t_enter_s":     pass_i1["t_enter_s"],
                "t_peak_s":      pass_i1["t_peak_s"],
                "t_exit_s":      pass_i1["t_exit_s"],
                "peak_elev_deg": pass_i1["peak_elev_deg"],
                "local_enter_s": pass_i1["t_enter_s"] - win_start_s,
                "local_peak_s":  pass_i1["t_peak_s"]  - win_start_s,
                "local_exit_s":  pass_i1["t_exit_s"]  - win_start_s,
                "csv":           args.output_sat_i1,
            },
        }

        summary_path = "sequence_summary.json"
        with open(summary_path, "w") as f:
            json.dump(summary, f, indent=2)
        print(f"Wrote summary → {summary_path}")

        # Final summary printout
        print(f"\n=== Done ===")
        print(f"  sat[i]   CSV  : {args.output_sat_i}  ({len(rows_i)} rows)")
        print(f"  sat[i+1] CSV  : {args.output_sat_i1}  ({len(rows_i1)} rows)")
        print(f"  Summary JSON  : {summary_path}")
        print(f"\n  Both CSVs share a 0-based time_s axis.")
        print(f"  time_s=0 corresponds to absolute t={win_start_s:.1f}s from start_utc.")
        print(f"\n  Key local timestamps (0-based, for Phase 2.3 overlap detection):")
        print(f"    sat[i]   enters ROI at  time_s ≈ {pass_i['t_enter_s']  - win_start_s:.1f}s")
        print(f"    sat[i]   peaks      at  time_s ≈ {pass_i['t_peak_s']   - win_start_s:.1f}s")
        print(f"    sat[i]   exits  ROI at  time_s ≈ {pass_i['t_exit_s']   - win_start_s:.1f}s")
        print(f"    sat[i+1] enters ROI at  time_s ≈ {pass_i1['t_enter_s'] - win_start_s:.1f}s")
        print(f"    sat[i+1] peaks      at  time_s ≈ {pass_i1['t_peak_s']  - win_start_s:.1f}s")
        print(f"    sat[i+1] exits  ROI at  time_s ≈ {pass_i1['t_exit_s']  - win_start_s:.1f}s")


if __name__ == "__main__":
    main()
