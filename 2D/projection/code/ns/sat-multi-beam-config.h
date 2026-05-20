/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * sat-multi-beam-config.h
 *
 * System parameter configuration for Multi-Beam LEO satellite simulation.
 * Mirrors params.py from the Python framework:
 *   2D/Multi-Beam LEO Communication Satellite Simulation Framework/params.py
 *
 * All physical constants and system parameters are defined here as a single
 * SimConfig struct, which is populated from CommandLine in the main script.
 */

#ifndef SAT_MULTI_BEAM_CONFIG_H
#define SAT_MULTI_BEAM_CONFIG_H

#include "ns3/satellite-utils.h"

#include <cmath>
#include <string>

namespace ns3
{

/**
 * SimConfig — mirrors the JSON produced by params.update_param_file().
 *
 * Defaults match the Python framework exactly (h=600 km, f=30 GHz,
 * 32×32 UPA per beam, 19 beams in 2-ring hexagonal, Rician K=10).
 */
struct SimConfig
{
    // ---------- physical constants ----------
    double speedOfLight{299792458.0};   // m/s — speed of light in vacuum
    double boltzmann{1.3806485e-23};    // J/K — Boltzmann constant

    // ---------- satellite orbit ----------
    double hSatelliteM{600e3};          // m   — LEO altitude (600 km)
    double vSatelliteMs{7.56e3};        // m/s — orbital speed
    double rEarthM{6371e3};             // m   — mean Earth radius

    // ---------- RF link ----------
    double centerFreqHz{30e9};          // Hz  — Ka-band centre frequency
    double bandwidthHz{25e6};           // Hz  — channel bandwidth
    double antennaGainDb{60.5};         // dBi — peak antenna array gain
    double noiseFigureDb{7.0};          // dB  — receiver noise figure (3GPP TR 38.821)
    double transmitPowerW{63.0};        // W   — total transmit power

    // ---------- UPA antenna array ----------
    int nAntennaX{32};                  // antenna elements in x per beam
    int nAntennaY{32};                  // antenna elements in y per beam
    int nBeamsX{5};                     // beams in x direction of array
    int nBeamsY{4};                     // beams in y direction of array
    int nBeams{19};                     // total beams (2-ring hexagonal layout)

    // ---------- fading ----------
    double ricianK{10.0};               // Rician K-factor (LoS-dominant)

    // ---------- simulation timing ----------
    double tFrameS{10e-3};             // s — time between two satellite positions

    // ---------- service area ----------
    double latitudeCenterDeg{35.67619};  // deg N — Tokyo
    double longitudeCenterDeg{139.65031}; // deg E — Tokyo
    double rFootprintM{100e3};           // m — footprint radius (configurable)

    // ---------- thermal noise ----------
    double temperatureK{300.0};         // K — UE receiver temperature

    // ---------- user distribution ----------
    int nUser{1000};                    // number of simulated users

    // ---------- atmospheric model inputs ----------
    double antennaD{0.0};              // m  — antenna diameter (0 = point)
    double unavailabilityPct{1.0};     // %  — rain unavailability parameter (p=1)

    // -----------------------------------------------------------------------
    // Derived quantities (mirrors params.py helper functions)
    // -----------------------------------------------------------------------

    /**
     * GetNoisePower — mirrors params.get_noise_power()
     * Returns thermal noise power including noise figure: k*T*B*F (Watts).
     */
    double GetNoisePower() const
    {
        // noise_power_W = k_B * T * B * 10^(NF_dB/10)
        return boltzmann * temperatureK * bandwidthHz *
               SatUtils::DbToLinear(noiseFigureDb);
    }

    /**
     * GetAntennaSpacing — mirrors params.get_antenna_spacing()
     * Returns element spacing = lambda/2 (metres).
     */
    double GetAntennaSpacing() const
    {
        return speedOfLight / centerFreqHz / 2.0;
    }

    /**
     * GetHpbwRad — half-power beamwidth of the UPA sub-array (radians).
     * Formula: HPBW ≈ 1.772 / N for a broadside ULA with N elements and
     * lambda/2 spacing.  Used by the Gaussian beam gain model in channel.cc.
     * We use the larger HPBW (smaller N axis) for the circular approximation.
     */
    double GetHpbwRad() const
    {
        int nMin = (nAntennaX < nAntennaY) ? nAntennaX : nAntennaY;
        return 1.772 / static_cast<double>(nMin);
    }

    /**
     * GetSatelliteAltitude — orbital radius (m).
     */
    double GetOrbitalRadius() const
    {
        return rEarthM + hSatelliteM;
    }

    /**
     * GetMinElevRad — minimum elevation angle at which the satellite is
     * visible from the footprint centre (radians).
     * Derived from: eps_zero = pi/2 - arccos(r_earth / altitude).
     * At this angle z_sat = 0 (satellite on the geometric horizon).
     */
    double GetMinElevRad() const
    {
        return M_PI / 2.0 - std::acos(rEarthM / GetOrbitalRadius());
    }

    /**
     * GetFrameAngleStepRad — angular increment per frame (radians).
     * delt_eps = v_sat * t_frame / altitude.
     */
    double GetFrameAngleStepRad() const
    {
        return vSatelliteMs * tFrameS / GetOrbitalRadius();
    }

    /**
     * GetTotalFrames — total simulation frames for one complete pass.
     * The arc runs from eps_zero to pi - eps_zero.
     */
    int GetTotalFrames() const
    {
        double range = M_PI - 2.0 * GetMinElevRad();
        return static_cast<int>(std::ceil(range / GetFrameAngleStepRad()));
    }
};

} // namespace ns3

#endif // SAT_MULTI_BEAM_CONFIG_H
