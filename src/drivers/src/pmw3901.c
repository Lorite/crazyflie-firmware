/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Copyright 2017, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */
/* pwm3901.c: PMW3901 driver */
#define DEBUG_MODULE "PMW"

#include "FreeRTOS.h"
#include "task.h"

#include "pmw3901.h"

#include <string.h>
#include "deck.h"
#include "debug.h"
#include "system.h"
#include "log.h"
#include "param.h"
#include "sleepus.h"
#include "usec_time.h"


static bool isInit = false;

static void registerWrite(const deckPin_t csPin, uint8_t reg, uint8_t value)
{
  // Set MSB to 1 for write
  reg |= 0x80u;

  spiBeginTransaction(SPI_BAUDRATE_2MHZ);
  digitalWrite(csPin, LOW);

  sleepus(50);

  spiExchange(1, &reg, &reg);
  sleepus(50);
  spiExchange(1, &value, &value);

  sleepus(50);

  digitalWrite(csPin, HIGH);
  spiEndTransaction();
  sleepus(200);
}

static uint8_t registerRead(const deckPin_t csPin, uint8_t reg)
{
  uint8_t data = 0;
  uint8_t dummy = 0;

  // Set MSB to 0 for read
  reg &= ~0x80u;

  spiBeginTransaction(SPI_BAUDRATE_2MHZ);
  digitalWrite(csPin, LOW);

  sleepus(50);

  spiExchange(1, &reg, &reg);
  sleepus(500);
  spiExchange(1, &dummy, &data);

  sleepus(50);

  digitalWrite(csPin, HIGH);
  spiEndTransaction();
  sleepus(200);

  return data;
}

static void InitRegisters(const deckPin_t csPin)
{
  registerWrite(csPin, 0x7F, 0x00);
  registerWrite(csPin, 0x61, 0xAD);
  registerWrite(csPin, 0x7F, 0x03);
  registerWrite(csPin, 0x40, 0x00);
  registerWrite(csPin, 0x7F, 0x05);
  registerWrite(csPin, 0x41, 0xB3);
  registerWrite(csPin, 0x43, 0xF1);
  registerWrite(csPin, 0x45, 0x14);
  registerWrite(csPin, 0x5B, 0x32);
  registerWrite(csPin, 0x5F, 0x34);
  registerWrite(csPin, 0x7B, 0x08);
  registerWrite(csPin, 0x7F, 0x06);
  registerWrite(csPin, 0x44, 0x1B);
  registerWrite(csPin, 0x40, 0xBF);
  registerWrite(csPin, 0x4E, 0x3F);
  registerWrite(csPin, 0x7F, 0x08);
  registerWrite(csPin, 0x65, 0x20);
  registerWrite(csPin, 0x6A, 0x18);
  registerWrite(csPin, 0x7F, 0x09);
  registerWrite(csPin, 0x4F, 0xAF);
  registerWrite(csPin, 0x5F, 0x40);
  registerWrite(csPin, 0x48, 0x80);
  registerWrite(csPin, 0x49, 0x80);
  registerWrite(csPin, 0x57, 0x77);
  registerWrite(csPin, 0x60, 0x78);
  registerWrite(csPin, 0x61, 0x78);
  registerWrite(csPin, 0x62, 0x08);
  registerWrite(csPin, 0x63, 0x50);
  registerWrite(csPin, 0x7F, 0x0A);
  registerWrite(csPin, 0x45, 0x60);
  registerWrite(csPin, 0x7F, 0x00);
  registerWrite(csPin, 0x4D, 0x11);
  registerWrite(csPin, 0x55, 0x80);
  registerWrite(csPin, 0x74, 0x1F);
  registerWrite(csPin, 0x75, 0x1F);
  registerWrite(csPin, 0x4A, 0x78);
  registerWrite(csPin, 0x4B, 0x78);
  registerWrite(csPin, 0x44, 0x08);
  registerWrite(csPin, 0x45, 0x50);
  registerWrite(csPin, 0x64, 0xFF);
  registerWrite(csPin, 0x65, 0x1F);
  registerWrite(csPin, 0x7F, 0x14);
  registerWrite(csPin, 0x65, 0x67);
  registerWrite(csPin, 0x66, 0x08);
  registerWrite(csPin, 0x63, 0x70);
  registerWrite(csPin, 0x7F, 0x15);
  registerWrite(csPin, 0x48, 0x48);
  registerWrite(csPin, 0x7F, 0x07);
  registerWrite(csPin, 0x41, 0x0D);
  registerWrite(csPin, 0x43, 0x14);
  registerWrite(csPin, 0x4B, 0x0E);
  registerWrite(csPin, 0x45, 0x0F);
  registerWrite(csPin, 0x44, 0x42);
  registerWrite(csPin, 0x4C, 0x80);
  registerWrite(csPin, 0x7F, 0x10);
  registerWrite(csPin, 0x5B, 0x02);
  registerWrite(csPin, 0x7F, 0x07);
  registerWrite(csPin, 0x40, 0x41);
  registerWrite(csPin, 0x70, 0x00);

  vTaskDelay(M2T(10)); // delay 10ms

  registerWrite(csPin, 0x32, 0x44);
  registerWrite(csPin, 0x7F, 0x07);
  registerWrite(csPin, 0x40, 0x40);
  registerWrite(csPin, 0x7F, 0x06);
  registerWrite(csPin, 0x62, 0xF0);
  registerWrite(csPin, 0x63, 0x00);
  registerWrite(csPin, 0x7F, 0x0D);
  registerWrite(csPin, 0x48, 0xC0);
  registerWrite(csPin, 0x6F, 0xD5);
  registerWrite(csPin, 0x7F, 0x00);
  registerWrite(csPin, 0x5B, 0xA0);
  registerWrite(csPin, 0x4E, 0xA8);
  registerWrite(csPin, 0x5A, 0x50);
  registerWrite(csPin, 0x40, 0x80);

  registerWrite(csPin, 0x7F, 0x00);
  registerWrite(csPin, 0x5A, 0x10);
  registerWrite(csPin, 0x54, 0x00);
}

static void powerOnResetAndConfigure(const deckPin_t csPin)
{
  // Power on reset
  registerWrite(csPin, 0x3a, 0x5a);
  vTaskDelay(M2T(5));

  // Reading the motion registers one time
  registerRead(csPin, 0x02);
  registerRead(csPin, 0x03);
  registerRead(csPin, 0x04);
  registerRead(csPin, 0x05);
  registerRead(csPin, 0x06);
  vTaskDelay(M2T(1));

  InitRegisters(csPin);
}

bool pmw3901Init(const deckPin_t csPin)
{
  if (isInit) {
    return true;
  }

  // Initialize CS Pin
  pinMode(csPin, OUTPUT);
  digitalWrite(csPin, HIGH);

  spiBegin();
  vTaskDelay(M2T(40));

  digitalWrite(csPin, HIGH);
  vTaskDelay(M2T(2));
  digitalWrite(csPin, LOW);
  vTaskDelay(M2T(2));
  digitalWrite(csPin, HIGH);
  vTaskDelay(M2T(2));

  uint8_t chipId    = registerRead(csPin, 0x00);
  uint8_t invChipId = registerRead(csPin, 0x5f);

  DEBUG_PRINT("Motion chip id: 0x%x:0x%x\n", chipId, invChipId);

  if (chipId == 0x49 && invChipId == 0xB6)
  {
    powerOnResetAndConfigure(csPin);

    isInit = true;
  }

  return isInit;
}

void pmw3901ReadMotion(const deckPin_t csPin, motionBurst_t * motion)
{
  uint8_t address = 0x16;

  spiBeginTransaction(SPI_BAUDRATE_2MHZ);
  digitalWrite(csPin,LOW);
  sleepus(50);
  spiExchange(1, &address, &address);
  sleepus(50);
  spiExchange(sizeof(motionBurst_t), (uint8_t*)motion, (uint8_t*)motion);
  sleepus(50);
  digitalWrite(csPin, HIGH);
  spiEndTransaction();
  sleepus(50);

  uint16_t realShutter = (motion->shutter >> 8) & 0x0FF;
  realShutter |= (motion->shutter & 0x0ff) << 8;
  motion->shutter = realShutter;
}

/*
 * Raw frame capture (Lorite #171).
 *
 * Sources: the frame-mode register sequence is the one of Bitcraze's Arduino
 * driver (bitcraze/Bitcraze_PMW3901, enableFrameBuffer() and readFrameBuffer(),
 * commit 374d488). The pixel assembly follows Pimoroni's pmw3901-python
 * frame_capture(): a byte tagged 01 in bits 7:6 holds pixel bits 7:2 in its
 * bits 5:0, the next byte tagged 10 holds pixel bits 1:0 in its bits 3:2.
 * (The Arduino driver adds those bits 3:2 without shifting them down, which
 * corrupts the two low bits of every pixel.) Tags 00 and 11 mean "no data yet".
 */

// Register read with a configurable address-data delay (datasheet tSRAD = 35 us)
// and the datasheet's 20 us read-to-next-command gap (tSRR). The motion driver's
// registerRead() above waits 800 us per read, which would make one frame take 2 s.
pmw3901FrameDiag_t pmw3901FrameDiag = {.sequence = 0, .csSetupUs = 1, .csHoldUs = 0};

static uint8_t registerReadTimed(const deckPin_t csPin, uint8_t reg, uint32_t addrDataDelayUs)
{
  uint8_t data = 0;
  uint8_t dummy = 0;

  reg &= ~0x80u;

  spiBeginTransaction(SPI_BAUDRATE_2MHZ);
  digitalWrite(csPin, LOW);
  sleepus(pmw3901FrameDiag.csSetupUs);
  spiExchange(1, &reg, &reg);
  sleepus(addrDataDelayUs);
  spiExchange(1, &dummy, &data);
  if (pmw3901FrameDiag.csHoldUs > 0) {
    sleepus(pmw3901FrameDiag.csHoldUs);
  }
  digitalWrite(csPin, HIGH);
  spiEndTransaction();
  sleepus(20);

  return data;
}

void pmw3901EnableFrameCapture(const deckPin_t csPin)
{
  // The frame-grab sequence of Pimoroni's pmw3901-python frame_capture().
  // Bitcraze's Arduino enableFrameBuffer() adds 0x41 = 0x1D and 0x70 = 0x00 and
  // never starts the grab: on crazylori_brushless_1 (2026-11-05) that sequence
  // returned no pixels in 3 s.
  if (pmw3901FrameDiag.sequence == 1) {
    // Bitcraze_PMW3901 enableFrameBuffer(), commit 374d488
    registerWrite(csPin, 0x7F, 0x07);
    registerWrite(csPin, 0x41, 0x1D);
    registerWrite(csPin, 0x4C, 0x00);
    registerWrite(csPin, 0x7F, 0x08);
    registerWrite(csPin, 0x6A, 0x38);
    registerWrite(csPin, 0x7F, 0x00);
    registerWrite(csPin, 0x55, 0x04);
    registerWrite(csPin, 0x40, 0x80);
    registerWrite(csPin, 0x4D, 0x11);
    registerWrite(csPin, 0x70, 0x00);
    registerWrite(csPin, 0x58, 0xFF);
    return;
  }

  registerWrite(csPin, 0x7F, 0x07);
  registerWrite(csPin, 0x4C, 0x00);
  registerWrite(csPin, 0x7F, 0x08);
  registerWrite(csPin, 0x6A, 0x38);
  registerWrite(csPin, 0x7F, 0x00);
  registerWrite(csPin, 0x55, 0x04);
  registerWrite(csPin, 0x40, 0x80);
  registerWrite(csPin, 0x4D, 0x11);

  vTaskDelay(M2T(10));

  registerWrite(csPin, 0x7F, 0x00);
  registerWrite(csPin, 0x58, 0xFF);
}

bool pmw3901ReadFrame(const deckPin_t csPin, uint8_t *frame, uint32_t addrDataDelayUs,
                      uint32_t timeoutUs, pmw3901FrameStats_t *stats)
{
  const uint64_t start = usecTimestamp();
  stats->captureUs = 0;
  stats->pixels = 0;
  stats->invalidReads = 0;
  stats->orderErrors = 0;
  stats->readsZero = 0;
  stats->readsFF = 0;
  stats->readsTag00 = 0;
  stats->readsTag11 = 0;
  memset(stats->first, 0, sizeof(stats->first));
  stats->status = 0;

  if (pmw3901FrameDiag.sequence == 1) {
    // Arduino driver: wait until the 0x58 status bits leave 11 (busy)
    do {
      stats->status = registerReadTimed(csPin, 0x58, addrDataDelayUs);
      if (usecTimestamp() - start > timeoutUs) {
        stats->captureUs = (uint32_t)(usecTimestamp() - start);
        return false;
      }
    } while ((stats->status >> 6) == 0x03);
  } else {
    // Pimoroni driver: wait until the raw-data grab status (0x59) reports
    // ready, then start the grab by writing 0x00 to 0x58
    do {
      stats->status = registerReadTimed(csPin, 0x59, addrDataDelayUs);
      if (usecTimestamp() - start > timeoutUs) {
        stats->captureUs = (uint32_t)(usecTimestamp() - start);
        return false;
      }
    } while ((stats->status & 0xC0u) == 0);
    registerWrite(csPin, 0x58, 0x00);
  }

  uint16_t reads = 0;

  uint16_t n = 0;
  bool haveHigh = false;
  uint8_t high = 0;
  while (n < PMW3901_FRAME_PIXELS) {
    const uint8_t value = registerReadTimed(csPin, 0x58, addrDataDelayUs);
    const uint8_t tag = value >> 6;
    if (reads < sizeof(stats->first)) {
      stats->first[reads] = value;
    }
    reads++;

    if (tag == 0x01) {
      if (haveHigh) {
        stats->orderErrors++;
      }
      high = (uint8_t)((value & 0x3Fu) << 2);
      haveHigh = true;
    } else if (tag == 0x02) {
      if (haveHigh) {
        frame[n++] = high | (uint8_t)((value & 0x0Cu) >> 2);
        haveHigh = false;
      } else {
        stats->orderErrors++;
      }
    } else {
      stats->invalidReads++;
      if (value == 0x00) {
        stats->readsZero++;
      } else if (value == 0xFF) {
        stats->readsFF++;
      } else if (tag == 0x00) {
        stats->readsTag00++;
      } else {
        stats->readsTag11++;
      }
    }

    if (usecTimestamp() - start > timeoutUs) {
      break;
    }
  }

  stats->captureUs = (uint32_t)(usecTimestamp() - start);
  stats->pixels = n;
  return n == PMW3901_FRAME_PIXELS;
}

bool pmw3901Reinit(const deckPin_t csPin)
{
  if (!isInit) {
    return false;
  }

  const uint8_t chipId    = registerRead(csPin, 0x00);
  const uint8_t invChipId = registerRead(csPin, 0x5f);
  if (chipId != 0x49 || invChipId != 0xB6) {
    DEBUG_PRINT("Re-init: wrong chip id 0x%x:0x%x\n", chipId, invChipId);
    return false;
  }

  powerOnResetAndConfigure(csPin);
  return true;
}
