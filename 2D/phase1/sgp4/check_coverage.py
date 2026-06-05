"""
check_coverage.py
-----------------
Reads all sat_XXXXX_cells.csv in the out/ directory and
reports, for each 1-second tick, how many satellites are
actively providing data for each cell_idx.

Identifies:
  1. True blackout: no satellite has ANY row for this tick
  2. Per-cell blackout: specific cell_idx has no row at this tick
  3. Low-SNR (greedy): all rows at this tick have max snr_dB < SNR_THRESHOLD
  4. Link budget table: analytical SNR vs elevation (SimConfig defaults)
  5. MRC combining: combined SNR across simultaneous satellites vs threshold
"""

import csv
import math
import os
import sys
from collections import defaultdict

# Allow importing link budget helpers regardless of working directory.
SCRIPT_DIR = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(SCRIPT_DIR, "..", "orbit-sgp4"))
from link_budget import (
    link_budget_table,
    critical_elevation,
    mrc_combine_snr_db,
)

OUT_DIR    = SCRIPT_DIR
SNR_THRESH = 0.0  # dB — rows below this are considered unusable

# ----------------------------------------------------------------
# Load all CSV files
# ----------------------------------------------------------------
# data[time_s][cell_idx] = list of snr_dB values across all sats
data = defaultdict(lambda: defaultdict(list))

for fname in os.listdir(OUT_DIR):
    if not fname.endswith("_cells.csv"):
        continue
    path = os.path.join(OUT_DIR, fname)
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t   = round(float(row["time_s"]))
            ci  = int(row["cell_idx"])
            snr = float(row["snr_dB"])
            data[t][ci].append(snr)

if not data:
    sys.exit(f"ERROR: no *_cells.csv files found in {OUT_DIR}")

all_times = sorted(data.keys())
all_cells = sorted({ci for tdict in data.values() for ci in tdict})

print(f"Time range : {all_times[0]} ~ {all_times[-1]} s")
print(f"Cells      : {len(all_cells)}  ({all_cells[0]} ~ {all_cells[-1]})")
print(f"Total ticks: {len(all_times)}")

# ----------------------------------------------------------------
# Check full blackout (no satellite data at all for this tick)
# ----------------------------------------------------------------
# Build expected tick set (every integer second in window)
t_min, t_max = all_times[0], all_times[-1]
expected_ticks = set(range(t_min, t_max + 1))
actual_ticks   = set(all_times)
missing_ticks  = sorted(expected_ticks - actual_ticks)

print(f"\n=== Full Blackout Ticks (no satellite data at all) ===")
if missing_ticks:
    # Group consecutive
    groups = []
    start = missing_ticks[0]
    prev  = missing_ticks[0]
    for t in missing_ticks[1:]:
        if t == prev + 1:
            prev = t
        else:
            groups.append((start, prev))
            start = prev = t
    groups.append((start, prev))
    for gs, ge in groups:
        print(f"  [{gs} ~ {ge}]  duration={ge-gs+1}s")
    print(f"  Total blackout ticks: {len(missing_ticks)}s")
else:
    print("  None — every tick has at least one satellite row")

# ----------------------------------------------------------------
# Per-cell blackout: how many cells are un-served at each tick
# ----------------------------------------------------------------
print(f"\n=== Per-Cell Coverage Summary ===")
cell_blackout_count = defaultdict(int)
for t in actual_ticks:
    for ci in all_cells:
        if ci not in data[t]:
            cell_blackout_count[ci] += 1

for ci in all_cells:
    pct = cell_blackout_count[ci] / len(actual_ticks) * 100
    print(f"  cell {ci:2d}: blackout {cell_blackout_count[ci]:5d} ticks "
          f"({pct:5.1f}% of time)")

# ----------------------------------------------------------------
# Low-SNR blackout: all sats at this tick for this cell < threshold
# ----------------------------------------------------------------
print(f"\n=== Low-SNR Blackout (all sats snr < {SNR_THRESH} dB) ===")
low_snr_ticks = []
for t in sorted(actual_ticks):
    cells_with_any_good = sum(
        1 for ci in all_cells
        if ci in data[t] and max(data[t][ci]) >= SNR_THRESH
    )
    if cells_with_any_good == 0:
        low_snr_ticks.append(t)

print(f"  Ticks where ALL cells have SNR < {SNR_THRESH} dB: {len(low_snr_ticks)}")

# Group consecutive low-SNR ticks into contiguous gaps
if low_snr_ticks:
    gaps = []
    g_start = low_snr_ticks[0]
    g_prev  = low_snr_ticks[0]
    for t in low_snr_ticks[1:]:
        if t == g_prev + 1:
            g_prev = t
        else:
            gaps.append((g_start, g_prev, g_prev - g_start + 1))
            g_start = g_prev = t
    gaps.append((g_start, g_prev, g_prev - g_start + 1))

    print(f"\n  Gap breakdown ({len(gaps)} segments):")
    print(f"  {'Start':>8s}  {'End':>8s}  {'Duration':>10s}")
    print(f"  {'-'*8}  {'-'*8}  {'-'*10}")
    for gs, ge, dur in gaps:
        print(f"  {gs:8d}  {ge:8d}  {dur:8d} s")
    durs = [d for _, _, d in gaps]
    print(f"\n  Min gap : {min(durs)} s")
    print(f"  Max gap : {max(durs)} s")
    print(f"  Mean gap: {sum(durs)/len(durs):.1f} s")
    print(f"  Total   : {sum(durs)} s  ({sum(durs)/3600*100:.1f}% of 3600 s window)")

# ----------------------------------------------------------------
# How many satellites simultaneously visible at each tick
# ----------------------------------------------------------------
print(f"\n=== Simultaneous Satellite Count Distribution ===")
from collections import Counter
sat_counts = {}
for fname in os.listdir(OUT_DIR):
    if not fname.endswith("_cells.csv"):
        continue
    path = os.path.join(OUT_DIR, fname)
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t = round(float(row["time_s"]))
            if t not in sat_counts:
                sat_counts[t] = set()
            sat_counts[t].add(fname)

count_dist = Counter(len(v) for v in sat_counts.values())
print(f"  {'Num sats':>10s}  {'Ticks':>8s}")
for n in sorted(count_dist):
    print(f"  {n:>10d}  {count_dist[n]:>8d}")

# ----------------------------------------------------------------
# Link budget summary: analytical SNR vs elevation angle
# ----------------------------------------------------------------
print(f"\n=== Link Budget vs Elevation ===")
rows = link_budget_table()
crit = critical_elevation(snr_thresh_db=SNR_THRESH)
print(f"  {'Elev (°)':>8s}  {'Slant (km)':>10s}  {'FSPL (dB)':>9s}  {'SNR (dB)':>8s}")
print(f"  {'-'*8}  {'-'*10}  {'-'*9}  {'-'*8}")
for r in rows:
    marker = " ← below threshold" if r["snr_db"] < SNR_THRESH else ""
    print(f"  {r['elev_deg']:>8.0f}  {r['slant_km']:>10.1f}  "
          f"{r['fspl_db']:>9.1f}  {r['snr_db']:>8.1f}{marker}")
if crit <= 0.0:
    print(f"\n  SNR >= {SNR_THRESH} dB at ALL elevations (threshold very low)")
elif crit >= 90.0:
    print(f"\n  SNR < {SNR_THRESH} dB even at nadir — link budget insufficient")
else:
    print(f"\n  Critical elevation (SNR = {SNR_THRESH} dB): {crit:.1f}°")
    print(f"  Satellites below {crit:.1f}° elevation cannot provide service")

# ----------------------------------------------------------------
# MRC combining: compare greedy-max vs MRC per tick
# ----------------------------------------------------------------
print(f"\n=== MRC Combining vs Greedy-Max ===")
print(f"  Threshold      : {SNR_THRESH} dB")

low_snr_ticks_mrc = []
for t in sorted(actual_ticks):
    # A tick is a gap under MRC only if EVERY cell's MRC-combined SNR is below threshold.
    all_cells_below = all(
        ci not in data[t] or mrc_combine_snr_db(data[t][ci]) < SNR_THRESH
        for ci in all_cells
    )
    if all_cells_below:
        low_snr_ticks_mrc.append(t)

greedy_total = len(low_snr_ticks)
mrc_total    = len(low_snr_ticks_mrc)
window_s     = t_max - t_min + 1

print(f"  Greedy-max gaps : {greedy_total:4d} s  ({greedy_total/window_s*100:.1f}%)")
print(f"  MRC gaps        : {mrc_total:4d} s  ({mrc_total/window_s*100:.1f}%)")
improvement = greedy_total - mrc_total
if greedy_total > 0:
    print(f"  Improvement     : {improvement:+d} s  "
          f"({improvement/greedy_total*100:.1f}% reduction)")
else:
    print("  No greedy gaps to compare against.")

if low_snr_ticks_mrc:
    mrc_gaps = []
    g_start = low_snr_ticks_mrc[0]
    g_prev  = low_snr_ticks_mrc[0]
    for t in low_snr_ticks_mrc[1:]:
        if t == g_prev + 1:
            g_prev = t
        else:
            mrc_gaps.append((g_start, g_prev, g_prev - g_start + 1))
            g_start = g_prev = t
    mrc_gaps.append((g_start, g_prev, g_prev - g_start + 1))

    print(f"\n  MRC gap breakdown ({len(mrc_gaps)} segments):")
    print(f"  {'Start':>8s}  {'End':>8s}  {'Duration':>10s}")
    print(f"  {'-'*8}  {'-'*8}  {'-'*10}")
    for gs, ge, dur in mrc_gaps:
        print(f"  {gs:8d}  {ge:8d}  {dur:8d} s")
    durs = [d for _, _, d in mrc_gaps]
    print(f"\n  Min gap : {min(durs)} s")
    print(f"  Max gap : {max(durs)} s")
    print(f"  Mean gap: {sum(durs)/len(durs):.1f} s")
    print(f"  Total   : {sum(durs)} s  ({sum(durs)/window_s*100:.1f}% of {window_s} s window)")
else:
    print("\n  MRC combining eliminates ALL service gaps.")

