/**
 * zranger2_scan.c: multizone ROI scan logic of the Flow deck v2 range sensor
 * (Lorite #171). See zranger2.c for how the task uses it.
 */
#include "zranger2_scan.h"

void zscanZoneRoi(uint8_t grid, uint8_t zone, zscanRoi_t *roi)
{
  const uint8_t size = (grid == 2) ? 8 : (grid == 3) ? 6 : 4;
  const uint8_t col = zone % grid;
  const uint8_t row = zone / grid;
  const uint8_t x0 = (uint8_t)(col * (16 - size) / (grid - 1));
  const uint8_t y0 = (uint8_t)(row * (16 - size) / (grid - 1));

  roi->topLeftX = x0;
  roi->botRightX = (uint8_t)(x0 + size - 1);
  roi->topLeftY = (uint8_t)(15 - y0);
  roi->botRightY = (uint8_t)(15 - y0 - (size - 1));
}

zscanSlot_t zscanChooseNext(zscanSlot_t measured, uint8_t grid, bool interleave, uint8_t *nextZone)
{
  zscanSlot_t next = {.full = true, .zone = 0};
  if (grid == 0) {
    return next;
  }
  if (interleave && !measured.full) {
    return next;
  }

  const uint8_t nZones = (uint8_t)(grid * grid);
  if (*nextZone >= nZones) {
    *nextZone = 0;
  }
  next.full = false;
  next.zone = *nextZone;
  *nextZone = (uint8_t)((*nextZone + 1) % nZones);
  return next;
}
