"""
analysis/check_coverage.py
--------------------------
Reads all sat_XXXXX_cells.csv in the results directory and reports:
  1. Full blackout   : no satellite has ANY row for this tick
  2. Per-cell blackout: specific cell_idx has no row at this tick
  3. Low-SNR blackout: all rows at this tick have max snr_dB < SNR_THRESHOLD
  4. Simultaneous visible satellite count distribution
  5. Link budget table: analytical SNR vs elevation
  6. MRC combining  : combined SNR across simultaneous satellites vs threshold

Usage
-----
cd 2D/code/orbit-sgp4/analysis
python check_coverage.py [--out-dir ../results] [--snr-thresh 0.0]

# Or run from results/ directory (no --out-dir needed):
cd 2D/code/orbit-sgp4/results
python ../analysis/check_coverage.py
"""

import argparse
import csv
import math
import os
import sys
from collections import Counter, defaultdict

# link_budget.py is in the same analysis/ directory
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from link_budget import (
    link_budget_table,
    critical_elevation,
    mrc_combine_snr_db,
)

# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    # Default: results/ sibling of analysis/
    default_results = os.path.normpath(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ns_result")
    )
    p = argparse.ArgumentParser(
        description="Coverage and gap analysis from constellation scan CSVs."
    )
    p.add_argument("--out-dir",    default=default_results,
                   help=f"Directory containing sat_XXXXX_cells.csv files "
                        f"(default: {default_results})")
    p.add_argument("--snr-thresh", type=float, default=0.0,
                   help="SNR service threshold in dB (default 0.0)")
    return p.parse_args()

# ---------------------------------------------------------------------------
# Load data
# ---------------------------------------------------------------------------

def load_data(out_dir: str, snr_thresh: float):
    """data[time_s][cell_idx] = list[snr_dB] across all satellites."""
    data = defaultdict(lambda: defaultdict(list))
    n_files = 0
    for fname in os.listdir(out_dir):
        if not fname.endswith("_cells.csv"):
            continue
        n_files += 1
        path = os.path.join(out_dir, fname)
        with open(path, newline="") as f:
            for row in csv.DictReader(f):
                t   = round(float(row["time_s"]))
                ci  = int(row["cell_idx"])
                snr = float(row["snr_dB"])
                data[t][ci].append(snr)
    if not data:
        sys.exit(f"ERROR: no *_cells.csv files found in {out_dir}")
    print(f"  Loaded {n_files} satellite CSV files")
    return data

# ---------------------------------------------------------------------------
# Main analysis
# ---------------------------------------------------------------------------

def main():
    args    = parse_args()
    out_dir = os.path.abspath(args.out_dir)
    thresh  = args.snr_thresh

    print(f"=== Coverage Analysis ===")
    print(f"Data dir   : {out_dir}")
    print(f"SNR thresh : {thresh} dB")
    print()

    data = load_data(out_dir, thresh)

    all_times = sorted(data.keys())
    all_cells = sorted({ci for tdict in data.values() for ci in tdict})
    t_min, t_max = all_times[0], all_times[-1]

    print(f"Time range : {t_min} ~ {t_max} s")
    print(f"Cells      : {len(all_cells)}  ({all_cells[0]} ~ {all_cells[-1]})")
    print(f"Total ticks: {len(all_times)}")

    # ── Full blackout ──────────────────────────────────────────────────────
    expected_ticks = set(range(t_min, t_max + 1))
    missing_ticks  = sorted(expected_ticks - set(all_times))

    print(f"\n=== Full Blackout Ticks (no satellite data) ===")
    if missing_ticks:
        groups, s, p = [], missing_ticks[0], missing_ticks[0]
        for t in missing_ticks[1:]:
            if t == p + 1: p = t
            else: groups.append((s, p)); s = p = t
        groups.append((s, p))
        for gs, ge in groups:
            print(f"  [{gs} ~ {ge}]  duration={ge-gs+1}s")
        print(f"  Total: {len(missing_ticks)} s")
    else:
        print("  None — every tick has at least one satellite row")

    # ── Per-cell blackout ──────────────────────────────────────────────────
    print(f"\n=== Per-Cell Coverage Summary ===")
    cell_blackout = defaultdict(int)
    for t in all_times:
        for ci in all_cells:
            if ci not in data[t]:
                cell_blackout[ci] += 1
    for ci in all_cells:
        pct = cell_blackout[ci] / len(all_times) * 100
        print(f"  cell {ci:2d}: blackout {cell_blackout[ci]:5d} ticks ({pct:5.1f}%)")

    # ── Low-SNR blackout ───────────────────────────────────────────────────
    print(f"\n=== Low-SNR Blackout (all sats snr < {thresh} dB) ===")
    low_snr_ticks = [
        t for t in sorted(all_times)
        if not any(
            ci in data[t] and max(data[t][ci]) >= thresh
            for ci in all_cells
        )
    ]
    print(f"  Ticks where ALL cells have SNR < {thresh} dB: {len(low_snr_ticks)}")

    if low_snr_ticks:
        gaps, s, p = [], low_snr_ticks[0], low_snr_ticks[0]
        for t in low_snr_ticks[1:]:
            if t == p + 1: p = t
            else: gaps.append((s, p, p - s + 1)); s = p = t
        gaps.append((s, p, p - s + 1))

        print(f"\n  Gap breakdown ({len(gaps)} segments):")
        print(f"  {'Start':>8s}  {'End':>8s}  {'Duration':>10s}")
        print(f"  {'-'*8}  {'-'*8}  {'-'*10}")
        for gs, ge, dur in gaps:
            print(f"  {gs:8d}  {ge:8d}  {dur:8d} s")
        durs = [d for _, _, d in gaps]
        print(f"\n  Min gap : {min(durs)} s")
        print(f"  Max gap : {max(durs)} s")
        print(f"  Mean gap: {sum(durs)/len(durs):.1f} s")
        total_gap = sum(durs)
        window_s  = t_max - t_min + 1
        print(f"  Total   : {total_gap} s  ({total_gap/window_s*100:.1f}% of {window_s} s window)")

    # ── Simultaneous satellite count ───────────────────────────────────────
    print(f"\n=== Simultaneous Satellite Count Distribution ===")
    sat_counts = defaultdict(set)
    for fname in os.listdir(out_dir):
        if not fname.endswith("_cells.csv"):
            continue
        with open(os.path.join(out_dir, fname), newline="") as f:
            for row in csv.DictReader(f):
                sat_counts[round(float(row["time_s"]))].add(fname)
    count_dist = Counter(len(v) for v in sat_counts.values())
    for n in sorted(count_dist):
        print(f"  {n:>2d} sat(s): {count_dist[n]:>6d} ticks")

    # ── Link budget table ──────────────────────────────────────────────────
    print(f"\n=== Link Budget vs Elevation ===")
    rows = link_budget_table()
    crit = critical_elevation(snr_thresh_db=thresh)
    print(f"  {'Elev (°)':>8s}  {'Slant (km)':>10s}  {'FSPL (dB)':>9s}  {'SNR (dB)':>8s}")
    print(f"  {'-'*8}  {'-'*10}  {'-'*9}  {'-'*8}")
    for r in rows:
        marker = " ← below threshold" if r["snr_db"] < thresh else ""
        print(f"  {r['elev_deg']:>8.0f}  {r['slant_km']:>10.1f}  "
              f"{r['fspl_db']:>9.1f}  {r['snr_db']:>8.1f}{marker}")
    print(f"\n  Critical elevation (SNR = {thresh} dB): {crit:.1f}°")
    print(f"  Satellites below {crit:.1f}° elevation cannot provide service")

    # ── MRC combining ──────────────────────────────────────────────────────
    print(f"\n=== MRC Combining vs Greedy-Max ===")
    low_snr_mrc = [
        t for t in sorted(all_times)
        if all(
            ci not in data[t] or mrc_combine_snr_db(data[t][ci]) < thresh
            for ci in all_cells
        )
    ]
    window_s    = t_max - t_min + 1
    greedy_tot  = len(low_snr_ticks)
    mrc_tot     = len(low_snr_mrc)
    print(f"  Greedy-max gaps : {greedy_tot:4d} s  ({greedy_tot/window_s*100:.1f}%)")
    print(f"  MRC gaps        : {mrc_tot:4d} s  ({mrc_tot/window_s*100:.1f}%)")
    if greedy_tot > 0:
        impr = greedy_tot - mrc_tot
        print(f"  Improvement     : {impr:+d} s  ({impr/greedy_tot*100:.1f}% reduction)")
    if not low_snr_mrc:
        print("\n  MRC combining eliminates ALL service gaps.")

if __name__ == "__main__":
    main()
