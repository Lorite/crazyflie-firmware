/**
 * Crazyflie control firmware
 *
 * Copyright (C) 2026 Alejandro Lorite Mora
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * ext_pose_history.h - short history of the estimated position, so a delayed
 * external measurement can be compared against the state as it was at its
 * capture epoch rather than against the state at reception (issue #80).
 *
 * The buffer is written once per prediction step and read from the same task,
 * so it needs no locking. It is deliberately a standalone module with no
 * dependency on the estimator globals, so the interpolation math can be
 * validated by a native test.
 */

#ifndef __EXT_POSE_HISTORY_H__
#define __EXT_POSE_HISTORY_H__

#include <stdbool.h>
#include <stdint.h>

// One sample per prediction step (PREDICT_RATE = 100 Hz), so 64 slots cover 640 ms,
// comfortably past the 500 ms age clamp. Power of two so the wrap is a mask.
// Cost: 64 * 16 B = 1 kB of RAM.
#define EXT_POSE_HISTORY_LEN 64

/** Drop every sample. Call whenever the estimator state is (re)initialised. */
void extPoseHistoryReset(void);

/** Record the estimated position at tMs. Oldest sample is overwritten when full. */
void extPoseHistoryPush(uint32_t tMs, const float pos[3]);

/**
 * Estimated position at targetMs, linearly interpolated between the two bracketing
 * samples (the 10 ms prediction spacing would otherwise cost up to 5 mm at 1 m/s).
 * A targetMs newer than the newest sample clamps to the newest sample.
 *
 * @return false when the history does not reach back to targetMs, which is the case
 *         for the first samples after a reset. The caller must then fall back.
 */
bool extPoseHistoryLookup(uint32_t targetMs, float pos[3]);

/** Number of valid samples currently held. Exposed for tests and diagnostics. */
uint8_t extPoseHistoryCount(void);

#endif // __EXT_POSE_HISTORY_H__
