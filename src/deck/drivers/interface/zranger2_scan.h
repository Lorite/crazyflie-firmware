/**
 * zranger2_scan.h: multizone ROI scan logic of the Flow deck v2 range sensor
 * (Lorite #171). Pure functions, no driver calls, so that they are unit tested.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define ZSCAN_MAX_ZONES 16

// A VL53L1X ROI in the ST API convention: SPAD columns x and rows y in 0..15,
// the top-left corner has the larger y.
typedef struct {
  uint8_t topLeftX;
  uint8_t topLeftY;
  uint8_t botRightX;
  uint8_t botRightY;
} zscanRoi_t;

typedef struct {
  bool full;     // the full 16 x 16 array, the range that feeds the EKF
  uint8_t zone;  // zone index when !full, row-major from the ROI origin
} zscanSlot_t;

/**
 * The ROI of zone `zone` in an n x n grid (n = 2, 3 or 4): 8 x 8, 6 x 6 or
 * 4 x 4 SPADs, spread evenly over the array (the 6 x 6 zones overlap by one SPAD).
 */
void zscanZoneRoi(uint8_t grid, uint8_t zone, zscanRoi_t *roi);

/**
 * The ROI that the range after `measured` uses.
 *
 * @param grid        effective grid, 0 = scan off
 * @param interleave  true: a full-array range after every zone range
 * @param nextZone    in/out: the next zone of the scan
 */
zscanSlot_t zscanChooseNext(zscanSlot_t measured, uint8_t grid, bool interleave, uint8_t *nextZone);
