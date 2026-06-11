/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * sat-roi-grid.h
 *
 * d×d rectangular ROI grid geometry.
 *
 * Converts the circular satellite footprint to the largest inscribed square
 * and divides it into a d×d uniform grid of observation cells.  Each cell is
 * treated as an independent user position for SNR/SINR computation via
 * ComputeFrameResults().
 *
 * Nadir approximation inscribed square (elevation = 90°):
 *   L = W = r_footprint × √2
 *
 * Elliptic footprint correction (orbit-sgp4):
 *   semi-major a = r_footprint / sin(θ),  semi-minor b = r_footprint
 *   max inscribed rect: W_along = a/√2,  H_cross = b/√2
 *   Cell positions computed by GetEllipticBeamCenters() in sat-multi-beam-geometry.h.
 *   EllipticFootprint is used here for diagnostic output only.
 *
 * Coordinate frame: local ENU centred at the fixed observation point.
 *   x → East,  y → North,  z → Up (altitude offset, zero at Earth surface)
 *
 * This ENU frame is used consistently with EcefOffsetToEnu() so that
 * the satellite position Vec3, beam centres, and cell positions all share
 * the same coordinate system.
 */

#ifndef SAT_ROI_GRID_H
#define SAT_ROI_GRID_H

#include "sat-multi-beam-geometry.h"

#include <vector>

namespace ns3
{

/**
 * EllipticFootprint — elliptic footprint parameters at a given elevation.
 *
 * Computed by ComputeEllipticFootprint(); used for diagnostic CSV output.
 */
struct EllipticFootprint
{
    double elevDeg{90.0};         // elevation angle (degrees)
    double aSemiMajorM{0.0};      // along-track semi-major axis = r_foot / sin(elev) (m)
    double bSemiMinorM{0.0};      // cross-track semi-minor axis = r_foot (m)
    double rectAlongHalfM{0.0};   // max inscribed rect along-track half-width = a/√2 (m)
    double rectCrossHalfM{0.0};   // max inscribed rect cross-track half-height = b/√2 (m)
};

/**
 * ComputeEllipticFootprint
 *
 * Compute elliptic footprint parameters from elevation angle and footprint radius.
 *
 * @param elevDeg      Elevation angle (degrees) from ROI centre to satellite.
 * @param rFootprintM  Footprint radius (m).
 * @return             EllipticFootprint with semi-axes and rectangle dimensions.
 */
EllipticFootprint ComputeEllipticFootprint(double elevDeg, double rFootprintM);

/**
 * RoiCell — one rectangular cell of the d×d grid.
 *
 * cx_m / cy_m are the cell-centre coordinates in the local ENU frame
 * (East and North offsets from the ROI observation point, in metres).
 * A cell is valid for channel computation only if inFootprint == true,
 * i.e., its centre lies within the circular footprint.
 */
struct RoiCell
{
    int    row{0};             // row index (0 = top row, d-1 = bottom row)
    int    col{0};             // column index (0 = left, d-1 = right)
    double cx_m{0.0};          // East offset of cell centre (m)
    double cy_m{0.0};          // North offset of cell centre (m)
    double width_m{0.0};       // cell width = L_m / d (same as height for square grid)
    bool   inFootprint{false}; // true if cx_m² + cy_m² ≤ r_footprint²
};

/**
 * RoiGrid — the complete d×d rectangular grid over the footprint.
 *
 * Cells are stored row-major: cells[row * d + col].
 * Only in-footprint cells are passed to ComputeFrameResults.
 */
struct RoiGrid
{
    int    d{5};       // grid dimension (d×d cells total)
    double L_m{0.0};   // rectangle length along East axis (m)
    double W_m{0.0};   // rectangle width along North axis (m)
    double cellL{0.0}; // cell width  = L_m / d  (East direction)
    double cellW{0.0}; // cell height = W_m / d  (North direction)
    std::vector<RoiCell> cells; // d*d cells, row-major order
};

/**
 * GenerateRoiGrid
 *
 * Create a d×d RoiGrid inscribed in the circular footprint.
 *
 * Nadir-approximation inscribed rectangle (square, elevation = 90°):
 *   L_m = W_m = rFootprintM × √2
 *   Each cell: width = L_m / d,  height = W_m / d
 *
 * Cell centre in ENU:
 *   cx = −L/2 + (col + 0.5) × cellL      (East, positive = right)
 *   cy = −W/2 + (row + 0.5) × cellW      (North, positive = up)
 *
 * inFootprint criterion:
 *   cx² + cy² ≤ rFootprintM²
 *
 * @param d            Grid dimension (cells per side, d ≥ 1).
 * @param rFootprintM  Footprint radius (m).
 * @return             Fully populated RoiGrid.
 */
RoiGrid GenerateRoiGrid(int d, double rFootprintM);

/**
 * GetRoiCellPositions
 *
 * Return in-footprint cell centres as Vec3 positions for ComputeFrameResults.
 *
 * Only cells with inFootprint == true are included.
 * Vec3 = {cx_m, cy_m, 0.0}  (ENU: East, North, zero altitude offset).
 * The ordering matches the indices returned by GetRoiCellIndexMap().
 *
 * @param grid  RoiGrid produced by GenerateRoiGrid.
 * @return      Vec3 vector for in-footprint cells (used as userPos in ComputeFrameResults).
 */
std::vector<Vec3> GetRoiCellPositions(const RoiGrid& grid);

/**
 * GetRoiCellIndexMap
 *
 * Return the grid.cells[] index for each in-footprint cell, parallel to
 * GetRoiCellPositions() output.  Used to map ComputeFrameResults results
 * back to (row, col) for CSV output.
 *
 * @param grid  RoiGrid produced by GenerateRoiGrid.
 * @return      Vector of indices into grid.cells (one per in-footprint cell).
 */
std::vector<size_t> GetRoiCellIndexMap(const RoiGrid& grid);

} // namespace ns3

#endif // SAT_ROI_GRID_H
