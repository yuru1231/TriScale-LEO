/* ============================================================
 * satellite-bstp-controller-patch.cc
 *
 * PATCH FILE — Apply these changes to:
 *   contrib/satellite/model/satellite-bstp-controller.cc
 *   (in your VMware SNS3 environment)
 *
 * Date: 2026-06-17
 * Tag:  [DemandAwareBHTP 2026-06-17]
 *
 * Purpose:
 *   1. Add SetDynamicBstpProvider() implementation.
 *   2. In DoBstpConfiguration(), replace the static GetNextConf() call with
 *      a conditional that first checks m_dynamicBstp; falls back to m_staticBstp
 *      if null.  All downstream ToggleCallback / SatMac::Enable/Disable logic
 *      is untouched — only the beam-set source changes.
 *
 * How to apply:
 *   1. Open contrib/satellite/model/satellite-bstp-controller.cc in VMware.
 *   2. Follow the two numbered sections below IN ORDER.
 *   3. Save and rebuild ns-3.
 * ============================================================ */


/* ── CHANGE 1: Add SetDynamicBstpProvider() implementation ─────────────────
 *
 * Find the end of the existing method implementations in the .cc file.
 * A good place is DIRECTLY BEFORE DoDispose() or at the very end of the file
 * (before the closing namespace brace if any).
 * Insert this complete method:
 *
 * ─── INSERT BEGIN ──────────────────────────────────────────────────────────

void
SatBstpController::SetDynamicBstpProvider(Ptr<SatDynamicBstpProvider> provider)
{
    // [DemandAwareBHTP 2026-06-17]
    // Store the provider pointer.  DoBstpConfiguration() checks this on every
    // invocation and uses it when non-null.  Passing nullptr restores the
    // original static-file (m_staticBstp) path with no further side effects.
    m_dynamicBstp = provider;

    NS_LOG_INFO("SatBstpController::SetDynamicBstpProvider: "
                << (provider ? "dynamic provider installed" : "reverted to static BSTP"));
}

 * ─── INSERT END ────────────────────────────────────────────────────────────
 * ─────────────────────────────────────────────────────────────────────────── */


/* ── CHANGE 2: Replace the static GetNextConf() call in DoBstpConfiguration ─
 *
 * Find DoBstpConfiguration() in the .cc file.
 * Inside it, locate the line:
 *
 *   std::vector<uint32_t> nextConf = m_staticBstp->GetNextConf();
 *
 * REPLACE that SINGLE LINE with the following block:
 *
 * ─── REPLACE BEGIN ─────────────────────────────────────────────────────────

    // [DemandAwareBHTP 2026-06-17]
    // If a dynamic BSTP provider has been registered via SetDynamicBstpProvider(),
    // use it to compute the next active beam set.  The provider returns a Conf with
    // the same semantics as one row of the static BSTP file:
    //   {validityInSuperframes, beamId1, beamId2, ...}
    // When no dynamic provider is set (m_dynamicBstp == nullptr), fall back to the
    // original static BSTP file playback — no change in behaviour from the
    // unmodified SNS3 source.
    //
    // The rest of DoBstpConfiguration() (ToggleCallback loop, SatMac Enable/Disable)
    // is intentionally unchanged so the same physical beam-switching path is used
    // regardless of whether the beam set came from a file or the dynamic provider.
    std::vector<uint32_t> nextConf;
    if (m_dynamicBstp)
    {
        SatDynamicBstpProvider::Conf dynConf =
            m_dynamicBstp->GetNextConf(Simulator::Now());

        // Pack into the same vector format the downstream logic already consumes:
        //   nextConf[0]       = validityInSuperframes
        //   nextConf[1..]     = active beamIds
        nextConf.push_back(dynConf.validityInSuperframes);
        nextConf.insert(nextConf.end(),
                        dynConf.activeBeams.begin(),
                        dynConf.activeBeams.end());

        NS_LOG_DEBUG("SatBstpController::DoBstpConfiguration [DemandAwareBHTP]:"
                     " validity=" << dynConf.validityInSuperframes
                     << " beams=" << dynConf.activeBeams.size()
                     << " t=" << Simulator::Now().GetSeconds() << "s");
    }
    else
    {
        // [ORIGINAL] Static file playback — unmodified behaviour.
        nextConf = m_staticBstp->GetNextConf();
    }

 * ─── REPLACE END ───────────────────────────────────────────────────────────
 *
 * IMPORTANT: Only the single line
 *   std::vector<uint32_t> nextConf = m_staticBstp->GetNextConf();
 * is replaced.  Everything after it (the ToggleCallback loop) stays exactly
 * as it was in the original SNS3 source.
 * ─────────────────────────────────────────────────────────────────────────── */


/* ── CHANGE 3: Add include for Simulator::Now() if not already present ──────
 *
 * The replacement code above calls Simulator::Now().
 * If the file does not already include the simulator header, add:
 *
 *   #include "ns3/simulator.h"
 *
 * near the top of satellite-bstp-controller.cc, alongside the other ns3 includes.
 * (Most SNS3 .cc files already include this; check before adding.)
 * ─────────────────────────────────────────────────────────────────────────── */


/* ── HOW TO WIRE IN YOUR EXAMPLE / SIMULATION SCRIPT ───────────────────────
 *
 * After applying the patch, wire the provider from your simulation script
 * (or from SatBhHelper::SetupDynamicBstp() once that path is integrated):
 *
 *   // 1. Create the greedy provider
 *   Ptr<SatGreedyBstpProvider> provider = CreateObject<SatGreedyBstpProvider>();
 *   provider->SetAttribute("MaxActiveBeams", UintegerValue(4));
 *   provider->SetAttribute("BacklogWeight",  DoubleValue(1.0));
 *   provider->SetAttribute("FairnessWeight", DoubleValue(0.5));
 *
 *   // 2. Register beams (must match fwdConf.txt entries for your scenario)
 *   provider->AddEnabledBeamInfo(beamId, userFreqId, feederFreqId, gwId);
 *   // ... repeat for each beam ...
 *
 *   // 3. Get the SatBstpController from SatBeamHelper
 *   Ptr<SatBstpController> bstpCtrl =
 *       simHelper->GetSatelliteHelper()->GetBeamHelper()->GetBstpController();
 *
 *   // 4. Install the provider (must be before Simulator::Run())
 *   bstpCtrl->SetDynamicBstpProvider(provider);
 *
 * After this, every DoBstpConfiguration() call uses the greedy scorer instead
 * of the static file, while SatMac::Enable/Disable still fires normally.
 * ─────────────────────────────────────────────────────────────────────────── */
