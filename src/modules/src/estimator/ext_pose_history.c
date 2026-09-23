/**
 * Crazyflie control firmware
 *
 * Copyright (C) 2026 Alejandro Lorite Mora
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * ext_pose_history.c - see ext_pose_history.h (issue #80).
 */

#include "ext_pose_history.h"

typedef struct {
  uint32_t tMs;      // end of the prediction step
  uint16_t dtMs;     // duration of the step (0 for the first sample after a reset)
  float delta[3];    // position displacement the step produced
} extPoseHistorySample_t;

static extPoseHistorySample_t history[EXT_POSE_HISTORY_LEN];
static uint8_t next = 0;   // slot the next sample is written to
static uint8_t count = 0;  // valid samples, saturating at EXT_POSE_HISTORY_LEN

static uint8_t slotBack(uint8_t k) {
  // k = 1 is the newest sample
  return (uint8_t)((next + EXT_POSE_HISTORY_LEN - k) & (EXT_POSE_HISTORY_LEN - 1));
}

void extPoseHistoryReset(void) {
  next = 0;
  count = 0;
}

void extPoseHistoryPush(uint32_t tMs, const float delta[3]) {
  uint16_t dtMs = 0;
  if (count > 0) {
    const uint32_t prevT = history[slotBack(1)].tMs;
    const uint32_t dt = tMs - prevT;
    dtMs = dt > 0xFFFFu ? 0xFFFFu : (uint16_t)dt;
  }
  history[next].tMs = tMs;
  history[next].dtMs = dtMs;
  history[next].delta[0] = delta[0];
  history[next].delta[1] = delta[1];
  history[next].delta[2] = delta[2];
  next = (uint8_t)((next + 1) & (EXT_POSE_HISTORY_LEN - 1));
  if (count < EXT_POSE_HISTORY_LEN) {
    count++;
  }
}

uint8_t extPoseHistoryCount(void) {
  return count;
}

uint32_t extPoseHistoryNewestMs(void) {
  return count > 0 ? history[slotBack(1)].tMs : 0;
}

bool extPoseHistoryDisplacement(uint32_t nowMs, uint32_t ageMs, float d[3]) {
  if (count == 0) {
    return false;
  }
  const uint32_t targetMs = nowMs - ageMs;
  float sum[3] = {0.0f, 0.0f, 0.0f};

  for (uint8_t k = 1; k <= count; k++) {
    const extPoseHistorySample_t* s = &history[slotBack(k)];
    if (s->tMs <= targetMs) {
      // This step ended before the window starts, so the window is fully covered.
      d[0] = sum[0]; d[1] = sum[1]; d[2] = sum[2];
      return true;
    }
    if (s->dtMs == 0) {
      // First sample after a reset: its start is unknown, so coverage cannot be proven.
      return false;
    }
    const uint32_t startMs = s->tMs - s->dtMs;
    if (startMs >= targetMs) {
      // Whole step inside the window.
      sum[0] += s->delta[0]; sum[1] += s->delta[1]; sum[2] += s->delta[2];
    } else {
      // Step straddles the window start: count the overlapping fraction only.
      const float frac = (float)(s->tMs - targetMs) / (float)s->dtMs;
      sum[0] += frac * s->delta[0]; sum[1] += frac * s->delta[1]; sum[2] += frac * s->delta[2];
      d[0] = sum[0]; d[1] = sum[1]; d[2] = sum[2];
      return true;
    }
  }
  // Ran out of samples before reaching targetMs: the history is too short.
  return false;
}
