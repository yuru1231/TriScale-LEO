/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * sat-constellation-params.h
 *
 * ConstellationParams — shared altitude-based beam geometry for TriScale-LEO.
 *
 * Purpose:
 *   Single source of truth for LEO beam geometry calculations.  Both the BH
 *   scheduler (interference model) and the 2D footprint tool (UT grid placement)
 *   derive their geometric constants from this struct, eliminating hard-coded
 *   values that are implicitly tied to a specific constellation altitude.
 *
 * Integration points:
 *   sat-bh-scheduler.cc  — InitBeamPositions() and ComputeInterferenceFactor()
 *   sat-bh-helper.cc     — SetupScheduler() injects grid positions into scheduler
 *   sat-bh-example.cc    — starlink25 scenario: kCellLat/kCellLon derived here
 *   sat-bh-2d-footprint.cc — rFootprint_m derived from RFootprintM()
 *
 * Beam half-angle convention (matches BeamRadiusType in sat-bh-time-plan.h):
 *   The "beamwidth" values in BeamRadiusType are the HALF-angles (not the full
 *   3-dB beamwidth).  Example: MIDDLE = 2.0° half-angle → 4.0° full beamwidth
 *   → r ≈ 19.2 km at Starlink 550 km altitude.
 *
 * Grid coordinate system:
 *   x = East  [km], positive = East
 *   y = North [km], positive = North
 *   Origin   = grid centre (roiLat, roiLon)
 *   Index i  = row × Nx + col
 *   row 0 = South, row (Ny-1) = North
 *   col 0 = West,  col (Nx-1) = East
 */

#ifndef SAT_CONSTELLATION_PARAMS_H
#define SAT_CONSTELLATION_PARAMS_H

#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

namespace ns3
{

// ── ConstellationParams ───────────────────────────────────────────────────────

struct ConstellationParams
{
    double altitudeKm{550.0};  ///< Orbital altitude above sea level [km]
    double minElevDeg{25.0};   ///< Minimum elevation angle for link establishment [deg]

    // ── Beam geometry ─────────────────────────────────────────────────────────

    /// Ground radius [km] of a spot beam from the altitude and beam half-angle.
    ///
    /// @param halfAngleDeg  Half-power HALF-angle of the spot beam [deg].
    ///                      This is HALF the 3-dB beamwidth.
    ///                      Example: MIDDLE = 2.0° → r ≈ 19.2 km at 550 km.
    ///
    /// Formula: r = altitudeKm × tan(halfAngleDeg × π/180)
    ///
    double
    BeamRadiusKm(double halfAngleDeg) const
    {
        const double kPi = std::acos(-1.0);
        return altitudeKm * std::tan(halfAngleDeg * kPi / 180.0);
    }

    /// Inscribed-circle radius [m] of the rectangular Nx×Ny beam grid.
    ///
    /// Grid design: cell width = 2 × r_beam (non-overlapping adjacent beams).
    /// The inscribed circle of the (max(Nx,Ny) × cellWidth) square bounds the ROI.
    ///
    /// Derivation:
    ///   cellWidth = 2 × r_beam
    ///   L = max(Nx,Ny) × cellWidth          (square side)
    ///   r_footprint = L / sqrt(2)           (inscribed circle)
    ///
    double
    RFootprintM(uint32_t Nx, uint32_t Ny, double halfAngleDeg) const
    {
        double r_beam_km = BeamRadiusKm(halfAngleDeg);
        double L_km      = static_cast<double>(std::max(Nx, Ny)) * 2.0 * r_beam_km;
        return (L_km / std::sqrt(2.0)) * 1000.0;  // convert km → m
    }

    /// Maximum coverage footprint radius [km] using spherical Earth geometry.
    ///
    /// Uses the Earth-central-angle formula for a satellite at altitudeKm:
    ///   sin(η) = R_earth × cos(ε) / (R_earth + h)
    ///   λ = π/2 − η − ε     [Earth central angle from sub-satellite point]
    ///   r = R_earth × λ
    ///
    /// Example: Starlink 550 km, minElev 25° → coverage radius ≈ 945 km
    ///
    double
    CoverageRadiusKm() const
    {
        const double kPi   = std::acos(-1.0);
        const double R     = 6371.0;           // Earth radius [km]
        const double h     = altitudeKm;
        const double eps   = minElevDeg * kPi / 180.0;
        const double sinEta = R * std::cos(eps) / (R + h);
        const double eta   = std::asin(sinEta);
        const double lambda = kPi / 2.0 - eta - eps;
        return R * lambda;
    }

    // ── Rectangular grid positions ────────────────────────────────────────────

    /// Beam centre positions in a local Nx×Ny rectangular grid [km].
    ///
    /// Indexing: i = row × Nx + col
    ///   row 0 = South (y most negative), row (Ny-1) = North
    ///   col 0 = West  (x most negative), col (Nx-1) = East
    ///
    /// Cell width = 2 × r_beam (non-overlapping beams).
    ///
    /// To convert positions to geographic coordinates (centre at latC, lonC):
    ///   deltaLat_deg = y_km / R_earth × (180/π)
    ///   deltaLon_deg = x_km / (R_earth × cos(latC_rad)) × (180/π)
    ///   lat_i = latC + deltaLat_deg
    ///   lon_i = lonC + deltaLon_deg
    ///
    std::vector<std::pair<double, double>>
    GridPositions(uint32_t Nx, uint32_t Ny, double halfAngleDeg) const
    {
        const double r_km  = BeamRadiusKm(halfAngleDeg);
        const double cellW = 2.0 * r_km;
        const double Lx    = static_cast<double>(Nx) * cellW;
        const double Ly    = static_cast<double>(Ny) * cellW;

        std::vector<std::pair<double, double>> pos;
        pos.reserve(Nx * Ny);
        for (uint32_t row = 0; row < Ny; ++row)
        {
            for (uint32_t col = 0; col < Nx; ++col)
            {
                double x = -Lx / 2.0 + (col + 0.5) * cellW;  // East  [km]
                double y = -Ly / 2.0 + (row + 0.5) * cellW;  // North [km]
                pos.push_back({x, y});
            }
        }
        return pos;
    }
};

// ── Predefined constellation parameters ─────────────────────────────────────

/// Starlink v1 Gen1 Shell 1 (1584 satellites)
///   altitude 550 km, inclination 53°, minimum user elevation 25°
///   MIDDLE beam (2° half-angle): r ≈ 19.2 km, cell spacing ≈ 38.4 km
inline ConstellationParams
StarlinkParams()
{
    return {550.0, 25.0};
}

/// Iridium NEXT (66 satellites)
///   altitude 636.5 km, inclination 86.4°, minimum user elevation 8.2°
///   MIDDLE beam (2° half-angle): r ≈ 22.2 km, cell spacing ≈ 44.4 km
inline ConstellationParams
IridiumParams()
{
    return {636.5, 8.2};
}

} // namespace ns3

#endif // SAT_CONSTELLATION_PARAMS_H
