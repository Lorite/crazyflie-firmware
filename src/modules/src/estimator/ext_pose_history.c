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
  uint32_t tMs;
  float pos[3];
} extPoseHistorySample_t;

static extPoseHistorySample_t history[EXT_POSE_HISTORY_LEN];
static uint8_t next = 0;   // slot the next sample is written to
static uint8_t count = 0;  // valid samples, saturating at EXT_POSE_HISTORY_LEN

void extPoseHistoryReset(void) {
  next = 0;
  count = 0;
}

void extPoseHistoryPush(uint32_t tMs, const float pos[3]) {
  history[next].tMs = tMs;
  history[next].pos[0] = pos[0];
  history[next].pos[1] = pos[1];
  history[next].pos[2] = pos[2];
  next = (uint8_t)((next + 1) & (EXT_POSE_HISTORY_LEN - 1));
  if (count < EXT_POSE_HISTORY_LEN) {
    count++;
  }
}

uint8_t extPoseHistoryCount(void) {
  return count;
}

bool extPoseHistoryLookup(uint32_t targetMs, float pos[3]) {
  if (count < 2) {
    return false;
  }
  // Walk back from the newest pair until targetMs falls inside a bracket.
  for (uint8_t k = 1; k < count; k++) {
    const uint8_t iNewer = (uint8_t)((next + EXT_POSE_HISTORY_LEN - k) & (EXT_POSE_HISTORY_LEN - 1));
    const uint8_t iOlder = (uint8_t)((next + EXT_POSE_HISTORY_LEN - k - 1) & (EXT_POSE_HISTORY_LEN - 1));
    const extPoseHistorySample_t* newer = &history[iNewer];
    const extPoseHistorySample_t* older = &history[iOlder];
    if (targetMs >= older->tMs) {
      const uint32_t span = newer->tMs - older->tMs;
      float alpha = 1.0f;  // targetMs at or past the newer sample
      if (targetMs < newer->tMs && span > 0) {
        alpha = (float)(targetMs - older->tMs) / (float)span;
      }
      for (int i = 0; i < 3; i++) {
        pos[i] = older->pos[i] + alpha * (newer->pos[i] - older->pos[i]);
      }
      return true;
    }
  }
  return false;  // older than everything we hold
}
