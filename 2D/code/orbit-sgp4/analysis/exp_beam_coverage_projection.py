"""
analysis/exp_beam_coverage_projection.py
-----------------------------------------
Experiment: Beam Footprint Coverage Projection onto ROI 25-Cell Grid

Evaluates how different beam radius types project onto a fixed geographic ROI
under varying satellite elevation angles (θ) and azimuth directions (φ).

Key metric:
  Cov(θ, φ, T, n, m) = intersection_area(beam_n_ellipse, cell_m) / cell_m_area
  where beam_n is aimed at cell n centre; n=m → self-coverage, n≠m → spillover.

Three analysis phases (run separately via --phase flag):
  --phase roi-study        Phase 1: Compare nadir vs θ=37° beam pattern extents
                                    to decide a suitable ROI size.
  --phase beam-cell-match  Phase 2: Table & plot of r_beam vs cell_size per beam type.
  --phase coverage         Phase 3: Full 25×25 Cov matrix sweep (11 elevations × 4
                                    azimuths × 5 beam types = 220 combinations).

Usage (run from project root)
------------------------------
  # Phase 1 - ROI study (decide ROI size)
  python 2D/code/orbit-sgp4/analysis/exp_beam_coverage_projection.py ^
      --phase roi-study ^
      --altitude-km 550 ^
      --output-dir 2D/code/orbit-sgp4/data/processed/beam_coverage_projection

  # Phase 2 - beam vs cell size matching
  python 2D/code/orbit-sgp4/analysis/exp_beam_coverage_projection.py ^
      --phase beam-cell-match ^
      --altitude-km 550 --roi-km 192 ^
      --output-dir 2D/code/orbit-sgp4/data/processed/beam_coverage_projection

  # Phase 3 - full coverage sweep
  python 2D/code/orbit-sgp4/analysis/exp_beam_coverage_projection.py ^
      --phase coverage ^
      --altitude-km 550 --roi-km 192 ^
      --elev 37 45 50 55 60 65 70 75 80 85 90 ^
      --azim 0 45 90 135 ^
      --beam-types 1.0 1.5 2.0 2.5 3.0 ^
      --sp-threshold 0.20 ^
      --sc-threshold 0.80 ^
      --output-dir 2D/code/orbit-sgp4/data/processed/beam_coverage_projection
"""

import argparse
import math
import os
import sys
from itertools import product

import matplotlib
matplotlib.use("Agg")          # headless backend; no display required
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import pandas as pd

# ---------------------------------------------------------------------------
# Optional Shapely import (required only for Phase 3 exact intersection)
# ---------------------------------------------------------------------------
try:
    from shapely.geometry import Point, box as shapely_box
    from shapely.affinity import scale as shapely_scale, rotate as shapely_rotate
    HAS_SHAPELY = True
except ImportError:
    HAS_SHAPELY = False

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
D         = 5          # 5×5 grid
N_CELLS   = D * D      # 25 cells
LAT_C_DEG = 35.676     # Tokyo ROI centre (latitude)
LON_C_DEG = 139.650    # Tokyo ROI centre (longitude)

BEAM_LABELS = {1.0: "NARROW", 1.5: "SLIGHT", 2.0: "MIDDLE",
               2.5: "BROAD",  3.0: "WIDE"}
BEAM_COLORS = {1.0: "#1f77b4", 1.5: "#ff7f0e", 2.0: "#2ca02c",
               2.5: "#d62728", 3.0: "#9467bd"}

# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def r_beam_km(half_angle_deg: float, altitude_km: float) -> float:
    """Nadir footprint radius for a beam with given half-angle (km)."""
    return altitude_km * math.tan(math.radians(half_angle_deg))


def ellipse_axes(r_km: float, elev_deg: float):
    """
    Projected ellipse semi-axes on the ground (km).
    Returns (a_along, b_cross):
      a_along  = r / sin²(θ)  -- along-track (larger, more stretched)
      b_cross  = r / sin(θ)   -- cross-track
    Reference: sat-multi-beam-geometry.h, GetEllipticBeamCenters()
    """
    sin_e = math.sin(math.radians(elev_deg))
    return r_km / (sin_e ** 2), r_km / sin_e


def along_cross_unit(azim_deg: float):
    """
    Unit vectors for along-track and cross-track directions (East, North).
    azim_deg: satellite heading azimuth, measured from North clockwise (°).
    along  = (sin φ, cos φ)   [East, North]
    cross  = (cos φ, -sin φ)  [perpendicular, rightward]
    """
    phi   = math.radians(azim_deg)
    along = np.array([math.sin(phi),  math.cos(phi)])
    cross = np.array([math.cos(phi), -math.sin(phi)])
    return along, cross


# ---------------------------------------------------------------------------
# ROI grid (fixed geographic frame, km ENU with origin at ROI centre)
# ---------------------------------------------------------------------------

def make_grid_cells(roi_km: float, d: int = D):
    """
    Returns list of 25 dicts, each with keys:
      idx, row, col, cx, cy, x0, y0, x1, y1  (all in km, ENU)
    Row 0 = southernmost, col 0 = westernmost.
    Cell (row,col): centre at cx = (col+0.5 - d/2)*cell, cy = (row+0.5 - d/2)*cell
    """
    cell = roi_km / d
    cells = []
    for row in range(d):
        for col in range(d):
            cx = (col + 0.5 - d / 2) * cell
            cy = (row + 0.5 - d / 2) * cell
            cells.append({
                "idx": row * d + col,
                "row": row, "col": col,
                "cx": cx, "cy": cy,
                "x0": cx - cell / 2, "y0": cy - cell / 2,
                "x1": cx + cell / 2, "y1": cy + cell / 2,
            })
    return cells, cell   # (list_of_cells, cell_size_km)


# ---------------------------------------------------------------------------
# Beam centre computation for the full 25-beam UPA pattern
# (used in Phase 1 visualisation)
# ---------------------------------------------------------------------------

def compute_upa_beam_centers(elev_deg: float, azim_deg: float,
                              r_km: float, d: int = D):
    """
    Compute 25 beam centre positions in ENU km (ground plane, z=0).
    The central beam (d//2, d//2) is aimed at ROI centre (0, 0).
    Adjacent beam pitch:
      cross-track: 2 × b_cross
      along-track: 2 × a_along
    """
    a, b      = ellipse_axes(r_km, elev_deg)
    along, cross = along_cross_unit(azim_deg)

    centers = []
    half = d // 2                          # 2 for d=5
    for row in range(d):
        for col in range(d):
            offset = (col - half) * 2 * b * cross + (row - half) * 2 * a * along
            centers.append(offset)         # shape (2,)
    return centers, a, b                   # list[(E_km, N_km)], a_km, b_km


# ---------------------------------------------------------------------------
# Shapely ellipse helper
# ---------------------------------------------------------------------------

def _shapely_ellipse(cx_km: float, cy_km: float,
                     a_along: float, b_cross: float,
                     azim_deg: float):
    """
    Create a Shapely Polygon representing the projected beam ellipse.
    Major axis (a_along) aligned with along-track direction (azim_deg from North).
    Rotation convention: start with major axis = North (y axis), then rotate CW
    by azim_deg  →  rotate CCW by -azim_deg in Shapely.
    """
    unit = Point(cx_km, cy_km).buffer(1.0, resolution=128)
    # scale: x = cross-track, y = along-track (before rotation)
    ell = shapely_scale(unit, xfact=b_cross, yfact=a_along, origin=(cx_km, cy_km))
    ell = shapely_rotate(ell, -azim_deg, origin=(cx_km, cy_km))
    return ell


# ---------------------------------------------------------------------------
# Coverage matrix  (Phase 3 core computation)
# ---------------------------------------------------------------------------

def compute_cov_matrix(elev_deg: float, azim_deg: float,
                        half_angle_deg: float, altitude_km: float,
                        grid_cells: list, cell_km: float) -> np.ndarray:
    """
    Returns 25×25 numpy array Cov[n, m]:
      n = beam index (beam aimed at cell n centre)
      m = cell index being intersected
      Cov[n,m] = intersection_area / cell_m_area
    Requires Shapely.
    """
    if not HAS_SHAPELY:
        raise RuntimeError("Shapely is required for Phase 3. Install with: pip install shapely")

    r   = r_beam_km(half_angle_deg, altitude_km)
    a, b = ellipse_axes(r, elev_deg)
    cell_area = cell_km ** 2              # km²

    Cov = np.zeros((N_CELLS, N_CELLS))
    for n, cn in enumerate(grid_cells):
        # beam aimed at cell n centre
        ell = _shapely_ellipse(cn["cx"], cn["cy"], a, b, azim_deg)
        for m, cm in enumerate(grid_cells):
            cell_rect = shapely_box(cm["x0"], cm["y0"], cm["x1"], cm["y1"])
            inter_area = ell.intersection(cell_rect).area
            Cov[n, m]  = inter_area / cell_area

    return Cov


def cov_metrics(Cov: np.ndarray):
    """Extract per-beam self-coverage and max spillover from 25×25 Cov matrix."""
    sc = np.diag(Cov)                     # self-coverage,  shape (25,)
    sp = np.array([
        Cov[j, :][np.arange(N_CELLS) != j].max()
        for j in range(N_CELLS)
    ])                                    # max spillover,  shape (25,)
    isolation = sc / np.maximum(sc + sp.sum(axis=0), 1e-9)
    return sc, sp, isolation


# ---------------------------------------------------------------------------
# Plot helpers
# ---------------------------------------------------------------------------

def _draw_grid(ax, grid_cells: list, cell_km: float,
               label_cells=True, linewidth=0.8, color="gray"):
    """Draw the 5×5 rectangular grid on ax."""
    for c in grid_cells:
        rect = plt.Rectangle((c["x0"], c["y0"]), cell_km, cell_km,
                              linewidth=linewidth, edgecolor=color,
                              facecolor="none")
        ax.add_patch(rect)
        if label_cells:
            ax.text(c["cx"], c["cy"], str(c["idx"]),
                    ha="center", va="center", fontsize=6, color="gray")


def _draw_ellipse_patch(ax, cx, cy, a_along, b_cross, azim_deg,
                         color, alpha=0.3, label=None):
    """Draw a rotated ellipse patch on ax (units: km)."""
    from matplotlib.patches import Ellipse
    angle_mpl = 90 - azim_deg     # matplotlib angle = CCW from East
    ell = Ellipse((cx, cy), width=2 * b_cross, height=2 * a_along,
                  angle=angle_mpl,
                  linewidth=0.6, edgecolor=color,
                  facecolor=color, alpha=alpha, label=label)
    ax.add_patch(ell)


# ---------------------------------------------------------------------------
# Phase 1 — ROI study
# ---------------------------------------------------------------------------

def phase1_roi_study(altitude_km: float, output_dir: str):
    """
    Visualise the full 25-beam UPA footprint pattern at θ=90° (nadir) and
    θ=37° (extreme), for MIDDLE beam (2.0°), overlaid on two candidate ROIs:
      ROI-MIDDLE (192 km) and ROI-WIDE (288 km).
    Helps the user decide which ROI size to adopt.
    """
    os.makedirs(os.path.join(output_dir, "phase1_roi"), exist_ok=True)

    elevations   = [90, 37]
    azim_deg     = 0.0               # satellite heading = North
    half_angle   = 2.0               # MIDDLE beam for reference
    r_km         = r_beam_km(half_angle, altitude_km)

    # Candidate ROI sizes (km): nadir natural size for each beam type
    roi_candidates = {T: 5 * 2 * r_beam_km(T, altitude_km)
                      for T in [1.0, 1.5, 2.0, 2.5, 3.0]}

    # ── Print summary table ──────────────────────────────────────────────
    print("\n=== Phase 1: ROI Size Candidates (Starlink h={:.0f} km) ===".format(altitude_km))
    print(f"{'Beam Type':<12} {'Half-angle':>10} {'r_beam (km)':>12} "
          f"{'Cell size (km)':>15} {'ROI size (km)':>14}")
    print("-" * 65)
    for T, roi in roi_candidates.items():
        r  = r_beam_km(T, altitude_km)
        print(f"{BEAM_LABELS[T]:<12} {T:>9.1f}°   {r:>11.1f}   "
              f"{2*r:>13.1f}   {roi:>12.1f}")

    # ── Ellipse parameter table at θ=37° ────────────────────────────────
    print("\n=== Ellipse axes at θ=37° (MIDDLE beam) ===")
    a37, b37 = ellipse_axes(r_km, 37)
    print(f"  r_beam      = {r_km:.1f} km  (nadir radius)")
    print(f"  a_along     = {a37:.1f} km  (along-track semi-axis, ×{a37/r_km:.2f} nadir)")
    print(f"  b_cross     = {b37:.1f} km  (cross-track semi-axis, ×{b37/r_km:.2f} nadir)")
    print(f"  Beam pitch along-track  = {2*a37:.1f} km  (vs nadir cell pitch {2*r_km:.1f} km)")
    print(f"  Beam pitch cross-track  = {2*b37:.1f} km  (vs nadir cell pitch {2*r_km:.1f} km)")

    # ── Figure: nadir vs 37° for MIDDLE beam ────────────────────────────
    fig, axes = plt.subplots(1, 2, figsize=(14, 7))
    roi_middle = roi_candidates[2.0]       # 192 km for h=550

    for ax_idx, (theta, ax) in enumerate(zip(elevations, axes)):
        a, b = ellipse_axes(r_km, theta)
        centers, _, _ = compute_upa_beam_centers(theta, azim_deg, r_km)

        grid_cells, cell_km = make_grid_cells(roi_middle)
        _draw_grid(ax, grid_cells, cell_km, label_cells=(theta == 90))

        for j, (cx, cy) in enumerate([(c[0], c[1]) for c in centers]):
            _draw_ellipse_patch(ax, cx, cy, a, b, azim_deg,
                                color=BEAM_COLORS[2.0], alpha=0.2)
            ax.plot(cx, cy, ".", markersize=3, color=BEAM_COLORS[2.0])

        # ROI boundary
        half_r = roi_middle / 2
        ax.add_patch(plt.Rectangle((-half_r, -half_r), roi_middle, roi_middle,
                                   linewidth=2, edgecolor="red",
                                   facecolor="none", label="ROI boundary"))
        ax.set_xlim(-200, 200)
        ax.set_ylim(-200, 200)
        ax.set_aspect("equal")
        ax.set_xlabel("East (km)")
        ax.set_ylabel("North (km)")
        ax.set_title(f"MIDDLE beam (2.0°), θ={theta}°, φ={azim_deg:.0f}°\n"
                     f"Ellipse: a={a:.1f} km, b={b:.1f} km | "
                     f"ROI={roi_middle:.0f}×{roi_middle:.0f} km")
        ax.legend(loc="upper right", fontsize=7)
        ax.grid(True, alpha=0.3)

    plt.suptitle(f"Phase 1 — Full 25-beam Pattern Projection (h={altitude_km:.0f} km)",
                 fontsize=12, fontweight="bold")
    plt.tight_layout()
    out_path = os.path.join(output_dir, "phase1_roi", "nadir_vs_elev37_footprint.png")
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"\n[Phase 1] Saved: {out_path}")

    # ── Figure: all 5 beam types at nadir ───────────────────────────────
    fig, ax = plt.subplots(figsize=(10, 10))
    for T, roi in roi_candidates.items():
        r  = r_beam_km(T, altitude_km)
        a, b = ellipse_axes(r, 90)         # nadir → circle
        centers, _, _ = compute_upa_beam_centers(90, 0, r)
        for j, (cx, cy) in enumerate([(c[0], c[1]) for c in centers]):
            _draw_ellipse_patch(ax, cx, cy, a, b, 0,
                                color=BEAM_COLORS[T], alpha=0.15,
                                label=f"{BEAM_LABELS[T]} ({T}°)" if j == 0 else None)

    # Draw MIDDLE ROI
    half_r = roi_candidates[2.0] / 2
    ax.add_patch(plt.Rectangle((-half_r, -half_r), roi_candidates[2.0], roi_candidates[2.0],
                               linewidth=2, edgecolor="black", linestyle="--",
                               facecolor="none", label="MIDDLE ROI (192 km)"))
    ax.set_xlim(-250, 250)
    ax.set_ylim(-250, 250)
    ax.set_aspect("equal")
    ax.set_xlabel("East (km)")
    ax.set_ylabel("North (km)")
    ax.set_title(f"Phase 1 — All 5 Beam Types at Nadir (h={altitude_km:.0f} km)")
    handles, labels = ax.get_legend_handles_labels()
    seen = set()
    unique = [(h, l) for h, l in zip(handles, labels)
              if l not in seen and not seen.add(l)]
    ax.legend(*zip(*unique), loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    out_path2 = os.path.join(output_dir, "phase1_roi", "all_beamtypes_nadir.png")
    plt.savefig(out_path2, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"[Phase 1] Saved: {out_path2}")

    # ── CSV summary ──────────────────────────────────────────────────────
    rows = []
    for T in [1.0, 1.5, 2.0, 2.5, 3.0]:
        r  = r_beam_km(T, altitude_km)
        a37, b37 = ellipse_axes(r, 37)
        rows.append({
            "beam_type": T,
            "label": BEAM_LABELS[T],
            "r_beam_km": round(r, 2),
            "nadir_cell_size_km": round(2 * r, 2),
            "nadir_roi_km": round(5 * 2 * r, 2),
            "a_along_37deg": round(a37, 2),
            "b_cross_37deg": round(b37, 2),
            "along_pitch_37deg": round(2 * a37, 2),
            "cross_pitch_37deg": round(2 * b37, 2),
        })
    csv_path = os.path.join(output_dir, "phase1_roi", "roi_size_candidates.csv")
    pd.DataFrame(rows).to_csv(csv_path, index=False)
    print(f"[Phase 1] Saved: {csv_path}")


# ---------------------------------------------------------------------------
# Phase 2 — Beam-cell size matching
# ---------------------------------------------------------------------------

def phase2_beam_cell_match(altitude_km: float, roi_km: float, output_dir: str):
    """
    For the chosen ROI, compute cell_size = roi_km/5 and compare with r_beam
    for each beam type. Print a table and save a visualisation.
    """
    os.makedirs(os.path.join(output_dir, "phase2_beam_cell_match"), exist_ok=True)
    grid_cells, cell_km = make_grid_cells(roi_km)

    print(f"\n=== Phase 2: Beam-Cell Size Matching (ROI={roi_km:.0f} km, "
          f"cell={cell_km:.1f} km, h={altitude_km:.0f} km) ===")
    print(f"{'Beam Type':<12} {'r_beam':>8} {'r/cell':>8} {'a@37°':>8} "
          f"{'b@37°':>8} {'Coverage@nadir':>16} {'Assessment':>12}")
    print("-" * 78)

    rows = []
    for T in [1.0, 1.5, 2.0, 2.5, 3.0]:
        r     = r_beam_km(T, altitude_km)
        ratio = r / cell_km
        a37, b37 = ellipse_axes(r, 37)
        cov_nadir = min(math.pi * r ** 2 / cell_km ** 2, 1.0)  # π r² / A_cell
        if ratio < 0.35:
            note = "too small"
        elif ratio > 0.65:
            note = "spills over"
        else:
            note = "good match"
        print(f"{BEAM_LABELS[T]:<12} {r:>7.1f}  {ratio:>7.3f}  {a37:>7.1f}  "
              f"{b37:>7.1f}  {cov_nadir:>14.3f}   {note}")
        rows.append({
            "beam_type": T, "label": BEAM_LABELS[T],
            "r_beam_km": round(r, 2), "cell_km": round(cell_km, 2),
            "r_over_cell": round(ratio, 4),
            "a_along_37": round(a37, 2), "b_cross_37": round(b37, 2),
            "cov_nadir": round(cov_nadir, 4), "assessment": note,
        })

    csv_path = os.path.join(output_dir, "phase2_beam_cell_match",
                            "beam_vs_cell_size.csv")
    pd.DataFrame(rows).to_csv(csv_path, index=False)
    print(f"\n[Phase 2] Saved: {csv_path}")

    # ── Figure: 5 beam types overlaid on fixed ROI grid (nadir) ─────────
    fig, ax = plt.subplots(figsize=(10, 10))
    _draw_grid(ax, grid_cells, cell_km, label_cells=True, linewidth=1.0)
    for T in [1.0, 1.5, 2.0, 2.5, 3.0]:
        r = r_beam_km(T, altitude_km)
        for c in grid_cells:
            _draw_ellipse_patch(ax, c["cx"], c["cy"], r, r, 0,
                                color=BEAM_COLORS[T], alpha=0.12,
                                label=f"{BEAM_LABELS[T]} r={r:.1f} km"
                                      if c["idx"] == 0 else None)

    handles, labels = ax.get_legend_handles_labels()
    seen = set()
    unique = [(h, l) for h, l in zip(handles, labels)
              if l not in seen and not seen.add(l)]
    ax.legend(*zip(*unique), loc="upper right", fontsize=8)
    ax.set_xlim(-roi_km * 0.6, roi_km * 0.6)
    ax.set_ylim(-roi_km * 0.6, roi_km * 0.6)
    ax.set_aspect("equal")
    ax.set_xlabel("East (km)")
    ax.set_ylabel("North (km)")
    ax.set_title(f"Phase 2 — 5 Beam Types on Fixed ROI Grid\n"
                 f"ROI={roi_km:.0f} km, cell={cell_km:.1f} km, "
                 f"h={altitude_km:.0f} km, Nadir (θ=90°)")
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    out_path = os.path.join(output_dir, "phase2_beam_cell_match",
                            "nadir_overlay_5beamtypes.png")
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"[Phase 2] Saved: {out_path}")


# ---------------------------------------------------------------------------
# Phase 3 — Full Coverage Sweep
# ---------------------------------------------------------------------------

def phase3_coverage(elevations: list, azimuths: list, beam_types: list,
                    altitude_km: float, roi_km: float,
                    sc_thresh: float, sp_thresh: float,
                    output_dir: str):
    """
    Sweep all (θ, φ, T) combinations, compute 25×25 Cov matrices, and
    produce heatmaps, spillover curves, and a beam-type recommendation table.
    """
    if not HAS_SHAPELY:
        print("ERROR: Shapely is required for Phase 3.")
        print("Install with:  pip install shapely")
        sys.exit(1)

    cov_dir = os.path.join(output_dir, "coverage_matrices")
    fig_dir  = os.path.join(output_dir, "figures")
    os.makedirs(cov_dir, exist_ok=True)
    os.makedirs(fig_dir,  exist_ok=True)

    grid_cells, cell_km = make_grid_cells(roi_km)
    total = len(elevations) * len(azimuths) * len(beam_types)
    print(f"\n=== Phase 3: Coverage Sweep — {total} combinations ===")
    print(f"    ROI={roi_km:.0f} km, cell={cell_km:.1f} km, h={altitude_km:.0f} km")

    # ── Main sweep ───────────────────────────────────────────────────────
    summary_rows = []
    # store per (T, phi): list of (theta, mean_sc, mean_sp)
    sweep_data = {}

    combo_idx = 0
    for T, phi in product(beam_types, azimuths):
        key = (T, phi)
        sweep_data[key] = {"theta": [], "mean_sc": [], "mean_sp": []}
        for theta in elevations:
            combo_idx += 1
            if combo_idx % 20 == 1:
                print(f"  [{combo_idx}/{total}] θ={theta}° φ={phi}° T={T}°")

            Cov = compute_cov_matrix(theta, phi, T, altitude_km,
                                     grid_cells, cell_km)
            sc, sp, iso = cov_metrics(Cov)

            # Save CSV
            theta_tag = int(round(theta))
            phi_tag = int(round(phi))
            tag = f"theta{theta_tag:02d}_phi{phi_tag:03d}_T{T:.1f}"
            csv_path = os.path.join(cov_dir, f"cov_{tag}.csv")
            df_cov = pd.DataFrame(Cov,
                                  index=[f"beam{j}" for j in range(N_CELLS)],
                                  columns=[f"cell{m}" for m in range(N_CELLS)])
            df_cov.to_csv(csv_path)

            # Save heatmap
            _plot_cov_heatmap(Cov, theta, phi, T, fig_dir)

            mean_sc = float(sc.mean())
            mean_sp = float(sp.mean())
            summary_rows.append({
                "theta": theta, "phi": phi, "beam_type": T,
                "label": BEAM_LABELS[T],
                "mean_sc": round(mean_sc, 4),
                "mean_sp": round(mean_sp, 4),
                "mean_iso": round(float(iso.mean()), 4),
                "pct_cells_sc_ok": round(
                    float((sc >= sc_thresh).mean()) * 100, 1),
                "pct_cells_sp_ok": round(
                    float((sp <= sp_thresh).mean()) * 100, 1),
            })
            sweep_data[key]["theta"].append(theta)
            sweep_data[key]["mean_sc"].append(mean_sc)
            sweep_data[key]["mean_sp"].append(mean_sp)

    # ── Summary CSV ──────────────────────────────────────────────────────
    df_summary = pd.DataFrame(summary_rows)
    summary_path = os.path.join(output_dir, "summary_metrics.csv")
    df_summary.to_csv(summary_path, index=False)
    print(f"\n[Phase 3] Summary saved: {summary_path}")

    # ── Aggregate plots (φ=0°, all beam types) ──────────────────────────
    _plot_sc_vs_elevation(sweep_data, beam_types, azimuths, elevations,
                          fig_dir, sc_thresh)
    _plot_sp_vs_elevation(sweep_data, beam_types, azimuths, elevations,
                          fig_dir, sp_thresh)
    _plot_recommendation_table(df_summary, elevations, beam_types,
                               sc_thresh, sp_thresh, fig_dir)
    print("[Phase 3] All figures saved to:", fig_dir)


# ---------------------------------------------------------------------------
# Phase 3 sub-plots
# ---------------------------------------------------------------------------

def _plot_cov_heatmap(Cov: np.ndarray, theta: float, phi: float,
                       T: float, fig_dir: str):
    """Save a 25×25 Cov heatmap."""
    fig, ax = plt.subplots(figsize=(7, 6))
    im = ax.imshow(Cov, vmin=0, vmax=1, cmap="YlOrRd", aspect="auto")
    plt.colorbar(im, ax=ax, label="Coverage fraction")
    ax.set_xlabel("Cell index (m)")
    ax.set_ylabel("Beam index (n = beam aimed at cell n)")
    ax.set_title(f"Cov[n,m] — θ={theta}° φ={phi}° T={T}° ({BEAM_LABELS[T]})\n"
                 f"Diagonal=self-coverage, off-diag=spillover")
    ax.set_xticks(range(0, N_CELLS, 5))
    ax.set_yticks(range(0, N_CELLS, 5))
    plt.tight_layout()
    theta_tag = int(round(theta))
    phi_tag = int(round(phi))
    tag = f"theta{theta_tag:02d}_phi{phi_tag:03d}_T{T:.1f}"
    plt.savefig(os.path.join(fig_dir, f"heatmap_{tag}.png"),
                dpi=120, bbox_inches="tight")
    plt.close()


def _plot_sc_vs_elevation(sweep_data: dict, beam_types: list, azimuths: list,
                           elevations: list, fig_dir: str, sc_thresh: float):
    """Self-coverage vs. elevation for each beam type (averaged over azimuths)."""
    fig, ax = plt.subplots(figsize=(9, 6))
    for T in beam_types:
        sc_by_theta = {theta: [] for theta in elevations}
        for phi in azimuths:
            key = (T, phi)
            for theta, sc in zip(sweep_data[key]["theta"],
                                  sweep_data[key]["mean_sc"]):
                sc_by_theta[theta].append(sc)
        thetas = sorted(sc_by_theta.keys())
        mean_sc = [np.mean(sc_by_theta[t]) for t in thetas]
        ax.plot(thetas, mean_sc, "-o", color=BEAM_COLORS[T],
                label=f"{BEAM_LABELS[T]} ({T}°)", linewidth=1.8, markersize=5)

    ax.axhline(sc_thresh, color="black", linestyle="--", linewidth=1.2,
               label=f"SC threshold = {sc_thresh:.2f}")
    ax.set_xlabel("Elevation angle θ (°)")
    ax.set_ylabel("Mean self-coverage Sc (avg over 25 cells & 4 azimuths)")
    ax.set_title("Self-Coverage vs Elevation — All Beam Types")
    ax.set_xticks(elevations)
    ax.set_ylim(0, 1.05)
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(fig_dir, "self_coverage_vs_elevation.png"),
                dpi=150, bbox_inches="tight")
    plt.close()


def _plot_sp_vs_elevation(sweep_data: dict, beam_types: list, azimuths: list,
                           elevations: list, fig_dir: str, sp_thresh: float):
    """Max spillover vs. elevation for each beam type (averaged over azimuths)."""
    fig, ax = plt.subplots(figsize=(9, 6))
    for T in beam_types:
        sp_by_theta = {theta: [] for theta in elevations}
        for phi in azimuths:
            key = (T, phi)
            for theta, sp in zip(sweep_data[key]["theta"],
                                  sweep_data[key]["mean_sp"]):
                sp_by_theta[theta].append(sp)
        thetas = sorted(sp_by_theta.keys())
        mean_sp = [np.mean(sp_by_theta[t]) for t in thetas]
        ax.plot(thetas, mean_sp, "-s", color=BEAM_COLORS[T],
                label=f"{BEAM_LABELS[T]} ({T}°)", linewidth=1.8, markersize=5)

    ax.axhline(sp_thresh, color="black", linestyle="--", linewidth=1.2,
               label=f"SP threshold = {sp_thresh:.2f}")
    ax.set_xlabel("Elevation angle θ (°)")
    ax.set_ylabel("Mean max spillover Sp (avg over 25 beams & 4 azimuths)")
    ax.set_title("Max Spillover vs Elevation — All Beam Types")
    ax.set_xticks(elevations)
    ax.set_ylim(0, 1.05)
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(fig_dir, "spillover_vs_elevation.png"),
                dpi=150, bbox_inches="tight")
    plt.close()


def _plot_recommendation_table(df: pd.DataFrame, elevations: list,
                                beam_types: list, sc_thresh: float,
                                sp_thresh: float, fig_dir: str):
    """
    Build a recommendation matrix: for each (elevation, beam_type) combo,
    show mean_sc as a heatmap with recommended beam type indicated.
    """
    # Average over phi for each (theta, T)
    avg = df.groupby(["theta", "beam_type"])["mean_sc"].mean().reset_index()
    pivot = avg.pivot(index="theta", columns="beam_type", values="mean_sc")
    pivot = pivot.reindex(index=sorted(elevations, reverse=True),
                          columns=sorted(beam_types))

    fig, ax = plt.subplots(figsize=(8, 6))
    im = ax.imshow(pivot.values, vmin=0, vmax=1, cmap="RdYlGn", aspect="auto")
    plt.colorbar(im, ax=ax, label="Mean self-coverage Sc")
    ax.set_xticks(range(len(beam_types)))
    ax.set_xticklabels([f"{T}°\n({BEAM_LABELS[T]})" for T in sorted(beam_types)],
                       fontsize=8)
    ax.set_yticks(range(len(elevations)))
    ax.set_yticklabels([f"{t}°" for t in sorted(elevations, reverse=True)])
    ax.set_xlabel("Beam Type (half-angle)")
    ax.set_ylabel("Elevation θ")
    ax.set_title(f"Mean Self-Coverage Sc per (θ, Beam Type)\n"
                 f"SC threshold={sc_thresh:.2f}, SP threshold={sp_thresh:.2f}")

    # Annotate cells
    for i, theta in enumerate(sorted(elevations, reverse=True)):
        for j, T in enumerate(sorted(beam_types)):
            val = pivot.loc[theta, T] if (theta in pivot.index and
                                           T in pivot.columns) else float("nan")
            if not math.isnan(val):
                ax.text(j, i, f"{val:.2f}", ha="center", va="center",
                        fontsize=7, color="black" if val < 0.7 else "white")

    plt.tight_layout()
    plt.savefig(os.path.join(fig_dir, "recommendation_table.png"),
                dpi=150, bbox_inches="tight")
    plt.close()

    # Print recommended beam type per elevation (highest Sc)
    print("\n=== Recommended Beam Type per Elevation (highest mean Sc) ===")
    print(f"{'Elevation':>10} {'Best T':>8} {'Label':>10} {'mean Sc':>10}")
    print("-" * 45)
    for theta in sorted(elevations):
        row = avg[avg["theta"] == theta]
        best = row.loc[row["mean_sc"].idxmax()]
        print(f"{theta:>9}°  {best['beam_type']:>7.1f}°  "
              f"{BEAM_LABELS[best['beam_type']]:>10}  {best['mean_sc']:>9.3f}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(
        description="Beam Footprint Coverage Projection Experiment (Starlink / ROI 25-cell)"
    )
    p.add_argument("--phase", choices=["roi-study", "beam-cell-match", "coverage"],
                   required=True, help="Which analysis phase to run")
    p.add_argument("--altitude-km",  type=float, default=550.0,
                   help="Satellite altitude km [Starlink default=550]")
    p.add_argument("--roi-km",       type=float, default=192.0,
                   help="ROI side length km (Phase 2 & 3); default=192 (MIDDLE nadir)")
    p.add_argument("--elev",         type=float, nargs="+",
                   default=[37, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90],
                   help="Elevation angles to sweep (Phase 3)")
    p.add_argument("--azim",         type=float, nargs="+",
                   default=[0, 45, 90, 135],
                   help="Azimuth angles to sweep (Phase 3)")
    p.add_argument("--beam-types",   type=float, nargs="+",
                   default=[1.0, 1.5, 2.0, 2.5, 3.0],
                   help="Beam half-angles in degrees (Phase 2 & 3)")
    p.add_argument("--sc-threshold", type=float, default=0.80,
                   help="Self-coverage threshold for recommendation (Phase 3)")
    p.add_argument("--sp-threshold", type=float, default=0.20,
                   help="Max spillover threshold for recommendation (Phase 3)")
    p.add_argument("--output-dir",   type=str,
                   default="2D/code/orbit-sgp4/data/processed/beam_coverage_projection",
                   help="Root output directory")
    return p.parse_args()


def main():
    args = parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    if args.phase == "roi-study":
        phase1_roi_study(args.altitude_km, args.output_dir)

    elif args.phase == "beam-cell-match":
        phase2_beam_cell_match(args.altitude_km, args.roi_km, args.output_dir)

    elif args.phase == "coverage":
        phase3_coverage(
            elevations  = sorted(args.elev),
            azimuths    = sorted(args.azim),
            beam_types  = sorted(args.beam_types),
            altitude_km = args.altitude_km,
            roi_km      = args.roi_km,
            sc_thresh   = args.sc_threshold,
            sp_thresh   = args.sp_threshold,
            output_dir  = args.output_dir,
        )


if __name__ == "__main__":
    main()
