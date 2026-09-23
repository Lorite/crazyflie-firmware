/**
 * Crazyflie control firmware
 *
 * Copyright (C) 2026 Alejandro Lorite Mora
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * ext_pose_history.h - short history of the position displacement produced by
 * each IMU prediction step, so a delayed external measurement can be shifted
 * by the motion the filter integrated over the measurement's age (issue #80).
 *
 * Why displacements and not states: the first version stored the corrected
 * state S and used S(now) - S(now - age). With the external position trusted
 * at millimetre level, every fused sample pulls S onto the (delayed)
 * measurement, so the shift derived from S fed straight back into the next
 * shift. That loop, S_k = S_{k-1} + z - S_{k-1-D}, is unstable for any delay
 * D >= 2 samples and was observed on hardware on 2026-09-23 as a correction
 * growing past a metre with the drone in hand. Summing only the prediction-step
 * displacements makes the shift a pure function of the IMU integration, which
 * measurement corrections cannot reach.
 *
 * Written once per prediction step and read from the same task, so no locking.
 * Standalone by design so the arithmetic is natively testable.
 */

#ifndef __EXT_POSE_HISTORY_H__
#define __EXT_POSE_HISTORY_H__

#include <stdbool.h>
#include <stdint.h>

// One sample per prediction step (PREDICT_RATE = 100 Hz), so 64 slots cover 640 ms,
// comfortably past the 500 ms age clamp. Power of two so the wrap is a mask.
// Cost: 64 * 18 B = 1152 B of RAM.
#define EXT_POSE_HISTORY_LEN 64

/** Drop every sample. Call whenever the estimator state is (re)initialised. */
void extPoseHistoryReset(void);

/**
 * Record the position displacement the prediction step that ended at tMs produced.
 * The step is taken to span from the previous push to tMs. Oldest sample is
 * overwritten when full.
 */
void extPoseHistoryPush(uint32_t tMs, const float delta[3]);

/**
 * Sum of the recorded displacements over the window (nowMs - ageMs, newest push],
 * with the slot that straddles the window's start counted by its overlapping
 * fraction. Motion after the newest push is NOT included, see
 * extPoseHistoryNewestMs() for the caller to extend with the current velocity.
 *
 * @return false when the history does not reach back to nowMs - ageMs (the first
 *         steps after a reset, or an age past the buffer span). d is untouched.
 */
bool extPoseHistoryDisplacement(uint32_t nowMs, uint32_t ageMs, float d[3]);

/** End time of the newest recorded step, 0 when empty. */
uint32_t extPoseHistoryNewestMs(void);

/** Number of valid samples currently held. Exposed for tests and diagnostics. */
uint8_t extPoseHistoryCount(void);

#endif // __EXT_POSE_HISTORY_H__
