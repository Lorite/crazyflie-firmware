// File under test ext_pose_history.c
//
// Issue #80: the state history that lets a delayed external measurement be compared
// against the state as it was at its capture epoch, instead of against the state at
// reception. The load-bearing case is testCurvedMotion...: it is the one where the
// old constant-velocity compensation (v_now * age) is measurably wrong and this
// module is right, which is why the latency A/B flies figure8 rather than hover.

#include "ext_pose_history.h"

#include "unity.h"

// Quadratic reference trajectory along x: x(t) = 0.5 * a * t^2, a = 10 m/s^2.
// Velocity therefore differs from the mean velocity over any window, which is exactly
// what breaks the constant-velocity assumption.
static const float ACCEL_MPS2 = 10.0f;
static const uint32_t PREDICT_STEP_MS = 10;  // PREDICT_RATE = 100 Hz

static float refX(uint32_t tMs) {
  const float t = (float)tMs * 0.001f;
  return 0.5f * ACCEL_MPS2 * t * t;
}

static float refV(uint32_t tMs) {
  return ACCEL_MPS2 * (float)tMs * 0.001f;
}

// Fill the history with the reference trajectory, one sample per prediction step.
static void pushReferenceUpTo(uint32_t lastMs) {
  for (uint32_t t = 0; t <= lastMs; t += PREDICT_STEP_MS) {
    const float pos[3] = {refX(t), 0.0f, 0.0f};
    extPoseHistoryPush(t, pos);
  }
}

void setUp(void) {
  extPoseHistoryReset();
}

void tearDown(void) {
  // Empty
}

void testThatLookupFailsOnAnEmptyHistory() {
  float pos[3] = {9.0f, 9.0f, 9.0f};
  TEST_ASSERT_FALSE(extPoseHistoryLookup(0, pos));
}

void testThatLookupFailsWithASingleSample() {
  const float p[3] = {1.0f, 2.0f, 3.0f};
  extPoseHistoryPush(100, p);

  float pos[3];
  TEST_ASSERT_EQUAL_UINT8(1, extPoseHistoryCount());
  TEST_ASSERT_FALSE(extPoseHistoryLookup(100, pos));
}

void testThatAnExactSampleTimeReturnsThatSample() {
  pushReferenceUpTo(200);

  float pos[3];
  TEST_ASSERT_TRUE(extPoseHistoryLookup(100, pos));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, refX(100), pos[0]);
}

void testThatATimeBetweenSamplesIsInterpolated() {
  pushReferenceUpTo(200);

  // 105 ms sits halfway between the 100 ms and 110 ms samples. Linear interpolation
  // of a quadratic leaves well under a millimetre of error at this step size.
  float pos[3];
  TEST_ASSERT_TRUE(extPoseHistoryLookup(105, pos));
  TEST_ASSERT_FLOAT_WITHIN(0.0005f, refX(105), pos[0]);
}

void testThatATimeNewerThanTheNewestSampleClampsToIt() {
  pushReferenceUpTo(200);

  float pos[3];
  TEST_ASSERT_TRUE(extPoseHistoryLookup(207, pos));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, refX(200), pos[0]);
}

void testThatATimeOlderThanTheWholeHistoryFails() {
  const float a[3] = {1.0f, 0.0f, 0.0f};
  const float b[3] = {2.0f, 0.0f, 0.0f};
  extPoseHistoryPush(1000, a);
  extPoseHistoryPush(1010, b);

  float pos[3];
  TEST_ASSERT_FALSE(extPoseHistoryLookup(900, pos));
}

void testThatTheRingWrapsAndKeepsTheRecentSamples() {
  // Three times the buffer length, so every slot has been overwritten twice.
  const uint32_t lastMs = 3 * EXT_POSE_HISTORY_LEN * PREDICT_STEP_MS;
  pushReferenceUpTo(lastMs);

  TEST_ASSERT_EQUAL_UINT8(EXT_POSE_HISTORY_LEN, extPoseHistoryCount());

  // The newest 640 ms are still exact...
  float pos[3];
  const uint32_t insideMs = lastMs - (EXT_POSE_HISTORY_LEN - 1) * PREDICT_STEP_MS;
  TEST_ASSERT_TRUE(extPoseHistoryLookup(insideMs, pos));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, refX(insideMs), pos[0]);

  // ...and anything older than the window is honestly reported as unavailable,
  // rather than silently clamped to the oldest sample.
  TEST_ASSERT_FALSE(extPoseHistoryLookup(insideMs - PREDICT_STEP_MS - 1, pos));
}

void testThatResetDropsEverything() {
  pushReferenceUpTo(200);
  TEST_ASSERT_TRUE(extPoseHistoryCount() > 0);

  extPoseHistoryReset();

  float pos[3];
  TEST_ASSERT_EQUAL_UINT8(0, extPoseHistoryCount());
  TEST_ASSERT_FALSE(extPoseHistoryLookup(100, pos));
}

void testThatOnCurvedMotionTheHistoryDeltaBeatsConstantVelocity() {
  // A measurement captured at 100 ms arrives when the filter is at 200 ms, so the
  // correction that shifts it to "now" should be the distance actually travelled in
  // that window.
  const uint32_t nowMs = 200;
  const uint32_t ageMs = 100;
  pushReferenceUpTo(nowMs);

  float posThen[3];
  TEST_ASSERT_TRUE(extPoseHistoryLookup(nowMs - ageMs, posThen));

  const float trueDisplacement = refX(nowMs) - refX(nowMs - ageMs);   // 0.15 m
  const float historyDelta = refX(nowMs) - posThen[0];
  const float velocityDelta = refV(nowMs) * (float)ageMs * 0.001f;    // 0.20 m

  // The history reproduces the real displacement to well under a millimetre.
  TEST_ASSERT_FLOAT_WITHIN(0.0005f, trueDisplacement, historyDelta);

  // The constant-velocity assumption overshoots it by 5 cm here, which is the error
  // this module exists to remove. Assert the gap so the test fails loudly if someone
  // ever makes the two equivalent.
  TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.05f, velocityDelta - trueDisplacement);
}
