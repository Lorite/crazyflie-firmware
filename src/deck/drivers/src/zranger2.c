/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 BitCraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * vl53l0x.c: Time-of-flight distance sensor driver
 */

#define DEBUG_MODULE "ZR2"

#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "deck.h"
#include "system.h"
#include "debug.h"
#include "log.h"
#include "param.h"
#include "range.h"
#include "static_mem.h"

#include "i2cdev.h"
#include "zranger2.h"
#include "vl53l1x.h"
#include "zranger2_scan.h"

#include "cf_math.h"
#include "supervisor.h"

// Measurement noise model
static const float expPointA = 2.5f;
static const float expStdA = 0.0025f; // STD at elevation expPointA [m]
static const float expPointB = 4.0f;
static const float expStdB = 0.2f;    // STD at elevation expPointB [m]
static float expCoeff;

#define RANGE_OUTLIER_LIMIT 5000 // the measured range is in [mm]

static uint16_t range_last = 0;

static bool isInit;

NO_DMA_CCM_SAFE_ZERO_INIT static VL53L1_Dev_t dev;

/*
 * Multizone ROI scan (Lorite #171), off by default.
 *
 * The VL53L1X ranges one region of interest (ROI) of its 16 x 16 SPAD array
 * at a time (datasheet DocID031281 Rev 3, section 2.5: from 4 x 4 SPADs to the
 * full 16 x 16). With zscan.grid = n > 1 the task steps the ROI through an
 * n x n grid of zones, one zone per range: n = 2 gives four 8 x 8 zones,
 * n = 3 nine 6 x 6 zones (overlapping by one SPAD), n = 4 sixteen 4 x 4 zones.
 * The new ROI is set between StopMeasurement and StartMeasurement, so it is
 * used by the next range. With zscan.interleave = 1 (default) every other
 * range uses the full array, and only those full-array ranges go to
 * range.zrange and the EKF (at half the rate, 20 Hz instead of 40 Hz).
 * Zone ranges never reach the EKF. The scan is refused while the drone is
 * armed or flying unless zscan.allowFlying is set, and interleave = 0 (no
 * height for the EKF at all) is only used on the ground.
 *
 * Zone i is row-major from the ROI origin (TopLeftX = 0, TopLeftY = 15) of the
 * ST API. Which zone points forward on the drone depends on the deck mounting
 * and is not verified yet.
 */

static uint8_t zscanGrid = 0;         // param: 0 or 1 off, 2, 3 or 4
static uint8_t zscanInterleave = 1;   // param
static uint8_t zscanAllowFlying = 0;  // param

static uint16_t zscanRange[ZSCAN_MAX_ZONES];  // log [mm]
static uint16_t zscanValid = 0;       // log: bit i set when zone i had RangeStatus 0
static uint16_t zscanSeq = 0;         // log: completed scans
static uint16_t zscanPeriodMs = 0;    // log: time of the last complete scan
static uint8_t zscanActive = 0;       // log: grid in use (0 = off or refused)
static uint16_t zscanRefused = 0;     // log

static const VL53L1_UserRoi_t fullRoi = {.TopLeftX = 0, .TopLeftY = 15, .BotRightX = 15, .BotRightY = 0};

static uint8_t zscanEffectiveGrid(void)
{
  const uint8_t grid = zscanGrid;
  if (grid < 2 || grid > 4) {
    return 0;
  }
  const bool airborne = supervisorIsArmed() || supervisorIsFlying();
  if (airborne && (!zscanAllowFlying || !zscanInterleave)) {
    zscanRefused++;
    return 0;
  }
  return grid;
}

static uint16_t zRanger2GetMeasurementAndRestart(VL53L1_Dev_t *dev, const VL53L1_UserRoi_t *nextRoi,
                                                 uint8_t *rangeStatus)
{
    VL53L1_Error status = VL53L1_ERROR_NONE;
    VL53L1_RangingMeasurementData_t rangingData;
    uint8_t dataReady = 0;
    uint16_t range;

    while (dataReady == 0)
    {
        status = VL53L1_GetMeasurementDataReady(dev, &dataReady);
        vTaskDelay(M2T(1));
    }

    status = VL53L1_GetRangingMeasurementData(dev, &rangingData);
    range = rangingData.RangeMilliMeter;
    if (rangeStatus) {
      *rangeStatus = rangingData.RangeStatus;
    }

    VL53L1_StopMeasurement(dev);
    if (nextRoi) {
      // Applied by the next StartMeasurement (VL53L1_set_user_zone only stores it)
      VL53L1_SetUserROI(dev, (VL53L1_UserRoi_t*)nextRoi);
    }
    status = VL53L1_StartMeasurement(dev);
    status = status;

    return range;
}

void zRanger2Init(DeckInfo* info)
{
  if (isInit)
    return;

  if (vl53l1xInit(&dev, I2C1_DEV))
  {
      DEBUG_PRINT("Z-down sensor [OK]\n");
  }
  else
  {
    DEBUG_PRINT("Z-down sensor [FAIL]\n");
    return;
  }

  xTaskCreate(zRanger2Task, ZRANGER2_TASK_NAME, ZRANGER2_TASK_STACKSIZE, NULL, ZRANGER2_TASK_PRI, NULL);

  // pre-compute constant in the measurement noise model for kalman
  expCoeff = logf(expStdB / expStdA) / (expPointB - expPointA);

  isInit = true;
}

bool zRanger2Test(void)
{
  if (!isInit)
    return false;

  return true;
}

void zRanger2Task(void* arg)
{
  TickType_t lastWakeTime;

  systemWaitStart();

  // Restart sensor
  VL53L1_StopMeasurement(&dev);
  VL53L1_SetDistanceMode(&dev, VL53L1_DISTANCEMODE_MEDIUM);
  VL53L1_SetMeasurementTimingBudgetMicroSeconds(&dev, 25000);

  VL53L1_StartMeasurement(&dev);

  lastWakeTime = xTaskGetTickCount();

  zscanSlot_t inProgress = {.full = true, .zone = 0};
  uint8_t nextZone = 0;
  uint32_t scanStartMs = T2M(lastWakeTime);

  while (1) {
    vTaskDelayUntil(&lastWakeTime, M2T(25));

    // Choose the ROI of the next range before this one is read and restarted
    const zscanSlot_t measured = inProgress;
    const uint8_t grid = zscanEffectiveGrid();
    zscanActive = grid;
    const zscanSlot_t next = zscanChooseNext(measured, grid, zscanInterleave != 0, &nextZone);

    VL53L1_UserRoi_t nextRoi;
    const VL53L1_UserRoi_t *nextRoiPtr = 0;
    if (!next.full) {
      zscanRoi_t zone;
      zscanZoneRoi(grid, next.zone, &zone);
      nextRoi.TopLeftX = zone.topLeftX;
      nextRoi.TopLeftY = zone.topLeftY;
      nextRoi.BotRightX = zone.botRightX;
      nextRoi.BotRightY = zone.botRightY;
      nextRoiPtr = &nextRoi;
    } else if (!measured.full) {
      nextRoiPtr = &fullRoi;
    }

    uint8_t rangeStatus = 0;
    const uint16_t range = zRanger2GetMeasurementAndRestart(&dev, nextRoiPtr, &rangeStatus);
    inProgress = next;

    if (!measured.full) {
      zscanRange[measured.zone] = range;
      if (rangeStatus == 0) {
        zscanValid |= (uint16_t)(1u << measured.zone);
      } else {
        zscanValid &= (uint16_t)~(1u << measured.zone);
      }
      if (grid > 0 && measured.zone == grid * grid - 1) {
        const uint32_t nowMs = T2M(xTaskGetTickCount());
        zscanPeriodMs = (uint16_t)(nowMs - scanStartMs);
        scanStartMs = nowMs;
        zscanSeq++;
      }
      continue;
    }

    range_last = range;
    rangeSet(rangeDown, range_last / 1000.0f);

    // check if range is feasible and push into the estimator
    // the sensor should not be able to measure >5 [m], and outliers typically
    // occur as >8 [m] measurements
    if (range_last < RANGE_OUTLIER_LIMIT) {
      float distance = (float)range_last * 0.001f; // Scale from [mm] to [m]
      float stdDev = expStdA * (1.0f  + expf( expCoeff * (distance - expPointA)));
      rangeEnqueueDownRangeInEstimator(distance, stdDev, xTaskGetTickCount());
    }
  }
}

static const DeckDriver zranger2_deck = {
  .vid = 0xBC,
  .pid = 0x0E,
  .name = "bcZRanger2",
  .usedGpio = 0,
  .usedPeriph = DECK_USING_I2C,

  .init = zRanger2Init,
  .test = zRanger2Test,
};

DECK_DRIVER(zranger2_deck);

PARAM_GROUP_START(deck)

/**
 * @brief Nonzero if [Z-ranger deck v2](%https://store.bitcraze.io/collections/decks/products/z-ranger-deck-v2) is attached
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcZRanger2, &isInit)

PARAM_GROUP_STOP(deck)

/**
 * Multizone ROI scan of the Flow deck v2 range sensor (Lorite #171), off by default.
 */
PARAM_GROUP_START(zscan)
/**
 * @brief Zones per side: 0 = off (default), 2 = 4 zones of 8x8, 3 = 9 of 6x6, 4 = 16 of 4x4 SPADs
 */
PARAM_ADD(PARAM_UINT8, grid, &zscanGrid)
/**
 * @brief 1 (default): every other range uses the full array and feeds range.zrange and the EKF
 */
PARAM_ADD(PARAM_UINT8, interleave, &zscanInterleave)
/**
 * @brief 1 allows the interleaved scan while armed or flying
 */
PARAM_ADD(PARAM_UINT8, allowFlying, &zscanAllowFlying)
PARAM_GROUP_STOP(zscan)

/**
 * Zone ranges of the multizone ROI scan [mm], zone i row-major from the ROI origin.
 */
LOG_GROUP_START(zscan)
LOG_ADD(LOG_UINT16, z0, &zscanRange[0])
LOG_ADD(LOG_UINT16, z1, &zscanRange[1])
LOG_ADD(LOG_UINT16, z2, &zscanRange[2])
LOG_ADD(LOG_UINT16, z3, &zscanRange[3])
LOG_ADD(LOG_UINT16, z4, &zscanRange[4])
LOG_ADD(LOG_UINT16, z5, &zscanRange[5])
LOG_ADD(LOG_UINT16, z6, &zscanRange[6])
LOG_ADD(LOG_UINT16, z7, &zscanRange[7])
LOG_ADD(LOG_UINT16, z8, &zscanRange[8])
LOG_ADD(LOG_UINT16, z9, &zscanRange[9])
LOG_ADD(LOG_UINT16, z10, &zscanRange[10])
LOG_ADD(LOG_UINT16, z11, &zscanRange[11])
LOG_ADD(LOG_UINT16, z12, &zscanRange[12])
LOG_ADD(LOG_UINT16, z13, &zscanRange[13])
LOG_ADD(LOG_UINT16, z14, &zscanRange[14])
LOG_ADD(LOG_UINT16, z15, &zscanRange[15])
/**
 * @brief Bit i set when zone i had RangeStatus 0 (valid) in its last range
 */
LOG_ADD(LOG_UINT16, valid, &zscanValid)
/**
 * @brief Completed scans
 */
LOG_ADD(LOG_UINT16, seq, &zscanSeq)
/**
 * @brief Duration of the last complete scan [ms]
 */
LOG_ADD(LOG_UINT16, periodMs, &zscanPeriodMs)
/**
 * @brief Grid in use, 0 when off or refused
 */
LOG_ADD(LOG_UINT8, active, &zscanActive)
/**
 * @brief Ranges refused because the drone was armed or flying
 */
LOG_ADD(LOG_UINT16, refused, &zscanRefused)
LOG_GROUP_STOP(zscan)
