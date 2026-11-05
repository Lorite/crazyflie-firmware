/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * pmw3901.h - Header file for pmw3901 driver
 */

#ifndef PMW3901_H_
#define PMW3901_H_

#include <stdint.h>
#include "deck.h"

typedef struct motionBurst_s {
  union {
    uint8_t motion;
    struct {
      uint8_t frameFrom0    : 1;
      uint8_t runMode       : 2;
      uint8_t reserved1     : 1;
      uint8_t rawFrom0      : 1;
      uint8_t reserved2     : 2;
      uint8_t motionOccurred: 1;
    };
  };

  uint8_t observation;
  int16_t deltaX;
  int16_t deltaY;

  uint8_t squal;

  uint8_t rawDataSum;
  uint8_t maxRawData;
  uint8_t minRawData;

  uint16_t shutter;
} __attribute__((packed)) motionBurst_t;

/**
 * Initialize the PMW3901 sensor
 *
 * @param csPin Chip Select pin as defined in deck pinout driver.
 *
 * @return  true if init successful else false.
 */
bool pmw3901Init(const deckPin_t csPin);

/**
 * Read the current accumulated motion.
 *
 * @param csPin   Chip Select pin as defined in deck pinout driver.
 * @param motion  A filled in motionBurst_t structure with the latest motion information
 */
void pmw3901ReadMotion(const deckPin_t csPin, motionBurst_t * motion);



/*
 * Raw frame capture (Lorite #171). The PMW3901 can return one 35 x 35 frame
 * of 8-bit pixels through register 0x58. While it is in frame mode it does not
 * deliver motion, and the frame-mode writes overwrite motion-tuning registers,
 * so the caller must call pmw3901Reinit() after a capture.
 */
#define PMW3901_FRAME_SIDE   35
#define PMW3901_FRAME_PIXELS (PMW3901_FRAME_SIDE * PMW3901_FRAME_SIDE)

typedef struct pmw3901FrameStats_s {
  uint32_t captureUs;     // from the first status read to the last pixel
  uint16_t pixels;        // pixels read, PMW3901_FRAME_PIXELS on success
  uint16_t invalidReads;  // reads of 0x58 whose status bits held no pixel data
  uint16_t orderErrors;   // a low-bits byte without a high-bits byte before it, or two high-bits bytes
} pmw3901FrameStats_t;

/**
 * Switch the sensor into frame mode (Bitcraze's Arduino driver sequence,
 * bitcraze/Bitcraze_PMW3901 enableFrameBuffer()).
 */
void pmw3901EnableFrameCapture(const deckPin_t csPin);

/**
 * Read one frame after pmw3901EnableFrameCapture().
 *
 * @param frame            PMW3901_FRAME_PIXELS bytes, row-major
 * @param addrDataDelayUs  wait between the address byte and the data byte (tSRAD)
 * @param timeoutUs        give up after this time
 * @return true when all PMW3901_FRAME_PIXELS pixels were read
 */
bool pmw3901ReadFrame(const deckPin_t csPin, uint8_t *frame, uint32_t addrDataDelayUs,
                      uint32_t timeoutUs, pmw3901FrameStats_t *stats);

/**
 * Power-on reset and the motion register set of pmw3901Init(), to return to
 * motion mode after a frame capture.
 * @return true when the chip id is right and the sensor was configured
 */
bool pmw3901Reinit(const deckPin_t csPin);

#endif /* PMW3901_H_ */
