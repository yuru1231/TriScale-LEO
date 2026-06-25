"""
analysis/check_coverage.py
--------------------------
Reads all sat_XXXXX_cells.csv in the results directory and reports:
  1. Full blackout   : no satellite has ANY row for this tick
  2. Low-SNR blackout: no geographic location has max SNR >= SNR_THRESHOLD
  3. Simultaneous visible satellite count distribution
  4. Link budget table: analytical SNR vs elevation
  5. MRC combining  : combined SNR at each geographic bin vs threshold

Geographic binning (GEO_BIN = 0.05°, ≈5 km):
  Cells from different satellites that point to the same ≈5 km area are
  grouped together and their SNRs are MRC-combined.  This fixes the
  cell_idx ambiguity in the old version: cell_idx is a satellite-relative
  index (aligned with along-track direction), so the same cell_idx from
  two different satellites can point to completely different geographic
  locations.  Using lat/lon bins ensures only physically co-located beams
  are combined.

  Requires CSVs produced by the updated sat-constellation-scanner.cc that
  writes cell_lat_deg and cell_lon_deg columns.  Falls back to cell_idx
  grouping (with a warning) for old-format CSVs that lack those columns.

Usage
-----
cd 2D/code/orbit-sgp4/analysis
python check_coverage.py [--out-dir ../ns_result] [--snr-thresh 3.0]
"""

import argparse
import csv
import json
import math
import os
import sys
from collections import Counter, defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from link_budget import (
    DEFAULT_CFG,
    link_budget_table,
    critical_elevation,
    mrc_combine_snr_db,
)

# Geographic bin size for grouping beam-centre positions across satellites.
# 0.05° ≈ 5.6 km (latitude) / 4.6 km (longitude at 35°N).
# This is finer than the nadir beam spacing (~13 km) so beams from the same
# satellite are rarely in the same bin, but close enough that beams from
# different satellites pointing to the same area will share a bin.
GEO_BIN = 0.05


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    default_results = os.path.normpath(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ns_result")
    )
    p = argparse.ArgumentParser(
        description="Coverage and gap analysis from constellation scan CSVs."
    )
    p.add_argument("--out-dir", default=default_results,
                   help=f"Directory containing sat_XXXXX_cells.csv files "
                        f"(default: {default_results})")
    p.add_argument("--snr-thresh", type=float, default=3.0,
                   help="SNR service threshold in dB (default 3.0 — 1st-order fading margin "
                        "above noise floor; 0 dB has no fade headroom and mis-classifies "
                        "borderline hard cells)")
    return p.parse_args()


# ---------------------------------------------------------------------------
# Load constellation_status.json
# ---------------------------------------------------------------------------

def load_status(out_dir: str) -> dict:
    path = os.path.join(out_dir, "constellation_status.json")
    if not os.path.isfile(path):
        return {}
    try:
        # Open in binary mode so Python's text-mode \r→\n translation does not
        # run first.  TLE satellite names written by C++ embed a raw \r byte
        # inside the JSON string literal (e.g. "STARLINK-1329   \r") which is
        # an invalid control character in JSON.  Strip \r bytes before decode.
        with open(path, "rb") as f:
            raw_bytes = f.read()
        raw = raw_bytes.replace(b"\r", b"").decode("utf-8")
        return json.loads(raw)
    except (json.JSONDecodeError, OSError, UnicodeDecodeError):
        return {}


def altitude_stats(status: dict):
    alts = [p["altitude_km"] for p in status.get("passes", [])
            if p.get("altitude_km", 0) > 0]
    if not alts:
        return None
    return sum(alts) / len(alts), min(alts), max(alts)


# ---------------------------------------------------------------------------
# Load CSV data
# ---------------------------------------------------------------------------

def _bin(v: float) -> float:
    """Round a coordinate to the nearest GEO_BIN multiple."""
    return round(round(v / GEO_BIN) * GEO_BIN, 5)


def load_data(out_dir: str):
    """
    Load all sat_XXXXX_cells.csv files.

    Returns
    -------
    data : dict
        data[time_s][geo_key] = list[snr_dB]  (all satellites at that tick)

        geo_key is (lat_bin, lon_bin)  when CSV has cell_lat_deg / cell_lon_deg
                   int cell_idx         otherwise (old format — MRC inaccurate)

    sat_at_tick : dict
        sat_at_tick[time_s] = set of CSV filenames visible at that tick

    has_geo : bool
        True if geographic columns were found in at least one CSV file
    """
    data        = defaultdict(lambda: defaultdict(list))
    sat_at_tick = defaultdict(set)
    n_files     = 0
    has_geo     = False

    for fname in sorted(os.listdir(out_dir)):
        if not fname.endswith("_cells.csv"):
            continue
        n_files += 1
        path = os.path.join(out_dir, fname)
        with open(path, newline="") as f:
            reader = csv.DictReader(f, skipinitialspace=True)
            if reader.fieldnames:
                reader.fieldnames = [name.strip() for name in reader.fieldnames]
            fields       = reader.fieldnames or []
            file_has_geo = ("cell_lat_deg" in fields and "cell_lon_deg" in fields)
            if file_has_geo:
                has_geo = True
            for row in reader:
                if row:
                    row = {
                        (k.strip() if isinstance(k, str) else k): v
                        for k, v in row.items()
                    }
                t   = round(float(row["time_s"]))
                snr = float(row["snr_dB"])
                # Skip degenerate first-tick rows (NaN SNR or NaN lat/lon).
                # These occur because the first callback has no valid prevSatEnu
                # to establish the along-track direction, producing NaN beam centres.
                if math.isnan(snr):
                    continue
                if file_has_geo:
                    lat = float(row["cell_lat_deg"])
                    lon = float(row["cell_lon_deg"])
                    if math.isnan(lat) or math.isnan(lon):
                        continue
                    key = (_bin(lat), _bin(lon))
                else:
                    key = int(row["cell_idx"])
                data[t][key].append(snr)
                sat_at_tick[t].add(fname)

    if not data:
        sys.exit(f"ERROR: no *_cells.csv files found in {out_dir}")

    print(f"  Loaded {n_files} satellite CSV files")
    if has_geo:
        print(f"  Geographic binning: {GEO_BIN} deg x {GEO_BIN} deg  "
              f"(~{GEO_BIN * 111:.0f} km per bin)  -> physically correct MRC")
    else:
        print(f"  WARNING: cell_lat_deg / cell_lon_deg columns absent -> "
              f"falling back to cell_idx grouping (MRC may over-estimate gain "
              f"because cell_idx is satellite-relative, not a fixed location)")
    return data, sat_at_tick, has_geo


# ---------------------------------------------------------------------------
# Gap detection helpers
# ---------------------------------------------------------------------------

def _gap_segments(ticks):
    """Convert a sorted list of gap ticks to (start, end, duration) segments."""
    if not ticks:
        return []
    segs, s, p = [], ticks[0], ticks[0]
    for t in ticks[1:]:
        if t == p + 1:
            p = t
        else:
            segs.append((s, p, p - s + 1))
            s = p = t
    segs.append((s, p, p - s + 1))
    return segs


def _print_gap_table(segs):
    print(f"  {'Start':>8s}  {'End':>8s}  {'Duration':>10s}")
    print(f"  {'-'*8}  {'-'*8}  {'-'*10}")
    for gs, ge, dur in segs:
        print(f"  {gs:8d}  {ge:8d}  {dur:8d} s")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    args    = parse_args()
    out_dir = os.path.abspath(args.out_dir)
    thresh  = args.snr_thresh

    print(f"=== Coverage Analysis ===")
    print(f"Data dir   : {out_dir}")
    print(f"SNR thresh : {thresh} dB")
    print()

    # Satellite altitude from constellation_status.json
    status    = load_status(out_dir)
    alt_stats_result = altitude_stats(status)

    if alt_stats_result:
        mean_km, min_km, max_km = alt_stats_result
        sim_cfg = {"h_satellite_m": mean_km * 1e3}
        spread  = max_km - min_km
        n_passes = len(status.get("passes", []))
        print(f"  h_satellite_km: mean={mean_km:.1f} km  "
              f"min={min_km:.1f} km  max={max_km:.1f} km  "
              f"(from {n_passes} passes, SGP4-derived)")
        if spread > 50:
            print(f"  WARNING: altitude spread {spread:.0f} km > 50 km — "
                  f"link budget table uses mean altitude only; "
                  f"per-satellite SNR in CSVs already correct.")
    else:
        h_km = status.get("h_satellite_km", None)
        if h_km and h_km > 0:
            sim_cfg = {"h_satellite_m": h_km * 1e3}
            print(f"  h_satellite_km: {h_km:.1f} km  (top-level field, no per-pass data)")
        else:
            sim_cfg = None
            print(f"  h_satellite_km: not found in JSON — using DEFAULT_CFG "
                  f"({DEFAULT_CFG['h_satellite_m']/1e3:.0f} km)")
    print()

    data, sat_at_tick, has_geo = load_data(out_dir)

    all_times = sorted(data.keys())
    all_keys  = sorted({k for tdict in data.values() for k in tdict}, key=repr)
    t_min, t_max = all_times[0], all_times[-1]

    loc_label = "geographic bins" if has_geo else "cell indices"
    print(f"Time range : {t_min} ~ {t_max} s")
    print(f"Locations  : {len(all_keys)} unique {loc_label}")
    print(f"Total ticks: {len(all_times)}")

    # ── Full blackout ──────────────────────────────────────────────────────
    expected_ticks = set(range(t_min, t_max + 1))
    missing_ticks  = sorted(expected_ticks - set(all_times))

    print(f"\n=== Full Blackout Ticks (no satellite data) ===")
    if missing_ticks:
        full_segs = _gap_segments(missing_ticks)
        for gs, ge, dur in full_segs:
            print(f"  [{gs} ~ {ge}]  duration={dur}s")
        print(f"  Total: {len(missing_ticks)} s")
    else:
        print("  None — every tick has at least one satellite row")

    # ── Low-SNR blackout (Greedy-Max) ─────────────────────────────────────
    # A tick is a gap when NO location has any satellite providing SNR >= thresh.
    # With geographic binning each key is a fixed ≈5 km area, so max(data[t][k])
    # is the best SNR achievable at that area from any single satellite.
    print(f"\n=== Low-SNR Blackout (all locations SNR < {thresh} dB) ===")
    low_snr_ticks = [
        t for t in sorted(all_times)
        if not any(
            k in data[t] and max(data[t][k]) >= thresh
            for k in all_keys
        )
    ]
    print(f"  Ticks where ALL locations have SNR < {thresh} dB: {len(low_snr_ticks)}")

    if low_snr_ticks:
        greedy_segs = _gap_segments(low_snr_ticks)
        durs        = [d for _, _, d in greedy_segs]
        tot         = sum(durs)
        win         = t_max - t_min + 1
        print(f"\n  Gap breakdown ({len(greedy_segs)} segments):")
        _print_gap_table(greedy_segs)
        print(f"\n  Min gap : {min(durs)} s")
        print(f"  Max gap : {max(durs)} s")
        print(f"  Mean gap: {tot / len(durs):.1f} s")
        print(f"  Total   : {tot} s  ({tot / win * 100:.1f}% of {win} s window)")

    # ── Simultaneous satellite count ───────────────────────────────────────
    print(f"\n=== Simultaneous Satellite Count Distribution ===")
    count_dist = Counter(len(v) for v in sat_at_tick.values())
    for n in sorted(count_dist):
        print(f"  {n:>2d} sat(s): {count_dist[n]:>6d} ticks")

    # ── Link budget table ──────────────────────────────────────────────────
    print(f"\n=== Link Budget vs Elevation ===")
    if sim_cfg:
        print(f"  (h = {sim_cfg['h_satellite_m']/1e3:.1f} km from SGP4, "
              f"n_beams = {DEFAULT_CFG['n_beams']})")
    rows = link_budget_table(
        elevs_deg=[5, 10, 15, 20, 25, 30, 45, 60, 75, 90],
        cfg=sim_cfg,
    )
    crit = critical_elevation(cfg=sim_cfg, snr_thresh_db=thresh)
    print(f"  {'Elev (deg)':>10s}  {'Slant (km)':>10s}  "
          f"{'FSPL (dB)':>9s}  {'L_atm (dB)':>10s}  {'SNR (dB)':>8s}")
    print(f"  {'-'*10}  {'-'*10}  {'-'*9}  {'-'*10}  {'-'*8}")
    for r in rows:
        marker = " <- below threshold" if r["snr_db"] < thresh else ""
        print(f"  {r['elev_deg']:>10.0f}  {r['slant_km']:>10.1f}  "
              f"{r['fspl_db']:>9.1f}  {r['atm_loss_db']:>10.2f}  "
              f"{r['snr_db']:>8.1f}{marker}")
    print(f"\n  Critical elevation (SNR = {thresh} dB): {crit:.1f} deg")
    print(f"  Satellites below {crit:.1f} deg elevation cannot provide service")

    # ── MRC combining ──────────────────────────────────────────────────────
    # For each geographic bin, MRC-combine SNRs from all satellites visible
    # at that bin at tick t.  A tick is covered if ANY bin's combined SNR
    # exceeds the threshold.
    #
    # With geographic binning (has_geo=True) this is physically correct:
    # only beams pointing to the same ≈5 km area are combined.
    #
    # With cell_idx fallback (has_geo=False) the old behaviour is preserved
    # with a warning: cell_idx is satellite-relative so the same index from
    # two satellites can point to different locations, making MRC over-optimistic.
    print(f"\n=== MRC Combining vs Greedy-Max ===")
    if has_geo:
        print(f"  Grouping by {GEO_BIN} deg geographic bin "
              f"(~{GEO_BIN * 111:.0f} km) -> only co-located beams combined")
    else:
        print(f"  WARNING: cell_idx grouping -> satellites' beams at the same "
              f"index are NOT necessarily at the same location; "
              f"MRC result may over-estimate improvement")

    low_snr_mrc = [
        t for t in sorted(all_times)
        if all(
            k not in data[t] or mrc_combine_snr_db(data[t][k]) < thresh
            for k in all_keys
        )
    ]
    window_s   = t_max - t_min + 1
    greedy_tot = len(low_snr_ticks)
    mrc_tot    = len(low_snr_mrc)
    print(f"  Greedy-max gaps : {greedy_tot:4d} s  ({greedy_tot / window_s * 100:.1f}%)")
    print(f"  MRC gaps        : {mrc_tot:4d} s  ({mrc_tot / window_s * 100:.1f}%)")
    if greedy_tot > 0:
        impr = greedy_tot - mrc_tot
        print(f"  Improvement     : {impr:+d} s  ({impr / greedy_tot * 100:.1f}% reduction)")

    if not low_snr_mrc:
        print("\n  MRC combining eliminates ALL service gaps.")
    else:
        mrc_segs = _gap_segments(low_snr_mrc)
        mrc_durs = [d for _, _, d in mrc_segs]
        tot      = sum(mrc_durs)
        print(f"\n  MRC gap breakdown ({len(mrc_segs)} segments):")
        _print_gap_table(mrc_segs)
        print(f"\n  Min gap : {min(mrc_durs)} s")
        print(f"  Max gap : {max(mrc_durs)} s")
        print(f"  Mean gap: {tot / len(mrc_durs):.1f} s")
        print(f"  Total   : {tot} s  ({tot / window_s * 100:.1f}% of {window_s} s window)")


if __name__ == "__main__":
    main()
