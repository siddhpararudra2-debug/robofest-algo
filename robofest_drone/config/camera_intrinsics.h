#pragma once

// ============================================================================
// *** UNCALIBRATED PLACEHOLDER — DO NOT USE FOR PRODUCTION FLIGHTS ***
// ============================================================================
// These intrinsics are PLACEHOLDER values. CAM_INTRINSICS_VALID = false
// causes undistort() to degrade to an identity pass-through. Position
// projection will use the pinhole approximation from H_FOV/V_FOV instead.
//
// To calibrate with the real camera:
//   python sim/calibrate_camera.py --camera 0 --rows 6 --cols 9
//
// The script will output calibrated values to paste here. After updating,
// set CAM_INTRINSICS_VALID = true.
//
// Also verify: thresholds.h → HSV_THRESHOLDS_VERIFIED
//              thresholds.h → RGB565_BYTE_ORDER_VERIFIED
// (REQ-DER-120, item 20)
// ============================================================================

#include <stdint.h>

namespace RobofestDrone {
namespace Config {

constexpr bool   CAM_INTRINSICS_VALID   = false; // invalid until real calibration
constexpr uint16_t CAM_CALIB_WIDTH      = 320u;
constexpr uint16_t CAM_CALIB_HEIGHT     = 240u;

constexpr float  CAM_FX_PX              = 277.1281f;
constexpr float  CAM_FY_PX              = 289.6890f;
constexpr float  CAM_CX_PX              = 160.0000f;
constexpr float  CAM_CY_PX              = 120.0000f;

constexpr float  CAM_K1                 = 0.0f;
constexpr float  CAM_K2                 = 0.0f;
constexpr float  CAM_P1                 = 0.0f;
constexpr float  CAM_P2                 = 0.0f;

} // namespace Config
} // namespace RobofestDrone
