/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * sat-roi-grid.cc
 *
 * Phase 2.0 — d×d rectangular ROI grid geometry implementation.
 * See sat-roi-grid.h for full API documentation.
 */

#include "sat-roi-grid.h"

#include <cmath>

namespace ns3
{

// ---------------------------------------------------------------------------
// GenerateRoiGrid
// ---------------------------------------------------------------------------

RoiGrid
GenerateRoiGrid(int d, double rFootprintM)
{
    // Phase 2.0: largest inscribed square inside the circular footprint.
    // A square of side s is inscribed in a circle of radius r when
    // s = r * sqrt(2)  (half-diagonal = r).
    const double sideM = rFootprintM * std::sqrt(2.0);

    RoiGrid grid;
    grid.d     = d;
    grid.L_m   = sideM;           // East extent
    grid.W_m   = sideM;           // North extent (square grid)
    grid.cellL = sideM / static_cast<double>(d); // cell width  (East)
    grid.cellW = sideM / static_cast<double>(d); // cell height (North)
    grid.cells.reserve(d * d);

    const double halfL = grid.L_m / 2.0;
    const double halfW = grid.W_m / 2.0;
    const double rSq   = rFootprintM * rFootprintM;

    for (int row = 0; row < d; ++row)
    {
        for (int col = 0; col < d; ++col)
        {
            // Cell centre in ENU: cx = East offset, cy = North offset
            const double cx = -halfL + (col + 0.5) * grid.cellL;
            const double cy = -halfW + (row + 0.5) * grid.cellW;

            RoiCell cell;
            cell.row         = row;
            cell.col         = col;
            cell.cx_m        = cx;
            cell.cy_m        = cy;
            cell.width_m     = grid.cellL;            // square cells
            cell.inFootprint = (cx * cx + cy * cy <= rSq);

            grid.cells.push_back(cell);
        }
    }

    return grid;
}

// ---------------------------------------------------------------------------
// GetRoiCellPositions
// ---------------------------------------------------------------------------

std::vector<Vec3>
GetRoiCellPositions(const RoiGrid& grid)
{
    std::vector<Vec3> positions;
    for (const auto& cell : grid.cells)
    {
        if (cell.inFootprint)
        {
            // ENU position: East=cx_m, North=cy_m, Up=0 (Earth surface)
            positions.push_back({cell.cx_m, cell.cy_m, 0.0});
        }
    }
    return positions;
}

// ---------------------------------------------------------------------------
// GetRoiCellIndexMap
// ---------------------------------------------------------------------------

std::vector<size_t>
GetRoiCellIndexMap(const RoiGrid& grid)
{
    std::vector<size_t> idxMap;
    for (size_t i = 0; i < grid.cells.size(); ++i)
    {
        if (grid.cells[i].inFootprint)
        {
            idxMap.push_back(i);
        }
    }
    return idxMap;
}

} // namespace ns3
