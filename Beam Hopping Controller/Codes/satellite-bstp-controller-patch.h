/* ============================================================
 * satellite-bstp-controller-patch.h
 *
 * PATCH FILE — Apply these changes to:
 *   contrib/satellite/model/satellite-bstp-controller.h
 *   (in your VMware SNS3 environment)
 *
 * Date: 2026-06-17
 * Tag:  [DemandAwareBHTP 2026-06-17]
 *
 * Purpose:
 *   Adds a SetDynamicBstpProvider() setter and m_dynamicBstp private member
 *   so that DoBstpConfiguration() can call GetNextConf() on a demand-aware
 *   provider instead of the static file playback in m_staticBstp.
 *
 * How to apply:
 *   1. Open contrib/satellite/model/satellite-bstp-controller.h in VMware.
 *   2. Follow the three numbered sections below IN ORDER.
 *   3. Save and rebuild ns-3.
 * ============================================================ */


/* ── CHANGE 1: Add #include near the top of the file ───────────────────────
 *
 * Find the existing includes section (usually after the #pragma once / header guard).
 * Add this line AFTER the last local include (e.g., after #include "satellite-static-bstp.h"):
 *
 *   // [DemandAwareBHTP 2026-06-17]
 *   // Abstract provider that can replace static BSTP file playback.
 *   // Included here so Ptr<SatDynamicBstpProvider> is a complete type in the
 *   // member declaration below.  The header lives in our custom BH module;
 *   // the build system must include Beam Hopping Controller/Codes/ in its path.
 *   #include "sat-dynamic-bstp-provider.h"
 *
 * ─────────────────────────────────────────────────────────────────────────── */


/* ── CHANGE 2: Add public setter declaration ────────────────────────────────
 *
 * Find the public section of the SatBstpController class declaration.
 * It will have methods like DoBstpConfiguration(), AddBeamInfo(), etc.
 * Add this block AFTER the last existing public method declaration:
 *
 *   // [DemandAwareBHTP 2026-06-17]
 *   // Install a demand-aware dynamic BSTP provider to replace static file
 *   // scheduling.  When non-null, DoBstpConfiguration() calls
 *   //   m_dynamicBstp->GetNextConf(Simulator::Now())
 *   // instead of m_staticBstp->GetNextConf(), then emits the same
 *   // ToggleCallback sequence so SatMac::Enable/Disable still fires.
 *   //
 *   // Must be called BEFORE simulation start (after AddEnabledBeamInfo()).
 *   // Passing nullptr reverts to static-file mode.
 *   void SetDynamicBstpProvider (Ptr<SatDynamicBstpProvider> provider);
 *
 * ─────────────────────────────────────────────────────────────────────────── */


/* ── CHANGE 3: Add private member declaration ───────────────────────────────
 *
 * Find the private section of the SatBstpController class.
 * It will contain members like:
 *   SatStaticBstp*    m_staticBstp;
 *   ...
 * Add this block DIRECTLY AFTER the m_staticBstp declaration:
 *
 *   // [DemandAwareBHTP 2026-06-17]
 *   // When non-null, DoBstpConfiguration() uses this provider instead of
 *   // m_staticBstp.  Null by default → original static-file behaviour.
 *   Ptr<SatDynamicBstpProvider> m_dynamicBstp;
 *
 * ─────────────────────────────────────────────────────────────────────────── */
