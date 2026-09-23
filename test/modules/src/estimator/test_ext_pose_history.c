// File under test ext_pose_history.c
//
// Issue #80: the history of IMU prediction displacements that lets a delayed external
// measurement be shifted by the motion the filter integrated over its age. The
// load-bearing cases are testCurvedMotion... (why this beats v_now * age) and
// testCorrections... (why the buffer holds prediction deltas and not the state: the
// state-based version fed every correction back into the next and diverged on
// hardware on 2026-09-23).

#include "ext_pose_history.h"

#include "unity.h"

// Quadratic reference trajectory along x: x(t) = 0.5 * a * t^2, a = 10 m/s^2.
// Velocity therefore differs from the mean velocity over any window, which is exactly
// what breaks the constant-velocity assumption.
static const float ACCEL_MPS2 = 10.0f;
static const uint32_t STEP_MS = 10;  // PREDICT_RATE = 100 Hz

static float refX(uint32_t tMs) {
  const float t = (float)tMs * 0.001f;
  return 0.5f * ACCEL_MPS2 * t * t;
}

static float refV(uint32_t tMs) {
  return ACCEL_MPS2 * (float)tMs * 0.001f;
}

// Push the reference trajectory as per-step displacements, one per prediction step,
// starting with a zero-extent sample at t0 (what the first push after a reset is).
static void pushReference(uint32_t t0Ms, uint32_t lastMs) {
  const float zero[3] = {0.0f, 0.0f, 0.0f};
  extPoseHistoryPush(t0Ms, zero);
  for (uint32_t t = t0Ms + STEP_MS; t <= lastMs; t += STEP_MS) {
    const float delta[3] = {refX(t) - refX(t - STEP_MS), 0.0f, 0.0f};
    extPoseHistoryPush(t, delta);
  }
}

void setUp(void) {
  extPoseHistoryReset();
}

void tearDown(void) {
  // Empty
}

void testThatAnEmptyHistoryCoversNothing() {
  float d[3] = {9.0f, 9.0f, 9.0f};
  TEST_ASSERT_FALSE(extPoseHistoryDisplacement(100, 50, d));
  TEST_ASSERT_EQUAL_UINT32(0, extPoseHistoryNewestMs());
}

void testThatTheFirstSampleAloneCannotProveCoverage() {
  const float delta[3] = {1.0f, 0.0f, 0.0f};
  extPoseHistoryPush(100, delta);

  float d[3];
  TEST_ASSERT_EQUAL_UINT8(1, extPoseHistoryCount());
  TEST_ASSERT_FALSE(extPoseHistoryDisplacement(100, 50, d));
}

void testThatAWindowAlignedToStepsSumsExactly() {
  pushReference(0, 200);

  // Window (100, 200]: ten whole steps.
  float d[3];
  TEST_ASSERT_TRUE(extPoseHistoryDisplacement(200, 100, d));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, refX(200) - refX(100), d[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, d[1]);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, d[2]);
}

void testThatAStraddlingStepCountsByItsFraction() {
  pushReference(0, 200);

  // Window (105, 200]: nine whole steps plus half of the (100, 110] step. Linear
  // interpolation of a quadratic leaves well under a millimetre of error.
  float d[3];
  TEST_ASSERT_TRUE(extPoseHistoryDisplacement(200, 95, d));
  TEST_ASSERT_FLOAT_WITHIN(0.0005f, refX(200) - refX(105), d[0]);
}

void testThatMotionAfterTheNewestStepIsNotInvented() {
  pushReference(0, 200);

  // The caller is at 207 ms but the newest step ended at 200: the history reports
  // only (107, 200] and tells the caller where it ends, so the caller can extend.
  float d[3];
  TEST_ASSERT_TRUE(extPoseHistoryDisplacement(207, 100, d));
  TEST_ASSERT_FLOAT_WITHIN(0.0005f, refX(200) - refX(107), d[0]);
  TEST_ASSERT_EQUAL_UINT32(200, extPoseHistoryNewestMs());
}

void testThatAnAgePastTheHistoryIsReportedNotClamped() {
  pushReference(1000, 1050);

  float d[3];
  TEST_ASSERT_FALSE(extPoseHistoryDisplacement(1050, 100, d));
}

void testThatTheRingWrapsAndKeepsTheRecentSteps() {
  // Three times the buffer length, so every slot has been overwritten twice.
  const uint32_t lastMs = 3 * EXT_POSE_HISTORY_LEN * STEP_MS;
  pushReference(0, lastMs);

  TEST_ASSERT_EQUAL_UINT8(EXT_POSE_HISTORY_LEN, extPoseHistoryCount());

  // A window inside the retained 640 ms is exact...
  float d[3];
  TEST_ASSERT_TRUE(extPoseHistoryDisplacement(lastMs, 500, d));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, refX(lastMs) - refX(lastMs - 500), d[0]);

  // ...and one reaching past the oldest retained step is honestly unavailable,
  // never silently truncated.
  TEST_ASSERT_FALSE(extPoseHistoryDisplacement(lastMs, EXT_POSE_HISTORY_LEN * STEP_MS + 1, d));
}

void testThatResetDropsEverything() {
  pushReference(0, 200);
  TEST_ASSERT_TRUE(extPoseHistoryCount() > 0);

  extPoseHistoryReset();

  float d[3];
  TEST_ASSERT_EQUAL_UINT8(0, extPoseHistoryCount());
  TEST_ASSERT_FALSE(extPoseHistoryDisplacement(200, 100, d));
}

void testThatOnCurvedMotionTheHistoryBeatsConstantVelocity() {
  // A measurement captured at 100 ms arrives at 200 ms. The shift that moves it to
  // "now" should be the distance actually travelled in that window.
  const uint32_t nowMs = 200;
  const uint32_t ageMs = 100;
  pushReference(0, nowMs);

  float d[3];
  TEST_ASSERT_TRUE(extPoseHistoryDisplacement(nowMs, ageMs, d));

  const float trueDisplacement = refX(nowMs) - refX(nowMs - ageMs);   // 0.15 m
  const float velocityDelta = refV(nowMs) * (float)ageMs * 0.001f;    // 0.20 m

  TEST_ASSERT_FLOAT_WITHIN(1e-6f, trueDisplacement, d[0]);

  // The constant-velocity assumption overshoots by 5 cm here, which is the error this
  // module exists to remove. Assert the gap so the test fails loudly if someone ever
  // makes the two equivalent.
  TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.05f, velocityDelta - trueDisplacement);
}

void testThatMeasurementCorrectionsCannotLeakIntoTheShift() {
  // The drone is perfectly still, so every prediction step moves it by zero. Meanwhile
  // (in the estimator, not here) trusted measurements keep yanking the corrected state
  // around. Because only prediction deltas are pushed, the shift stays exactly zero no
  // matter what the corrected state did. The state-based design failed this on
  // hardware: with a 12-sample delay the loop S_k = S_{k-1} + z - S_{k-1-D} diverged.
  const float zero[3] = {0.0f, 0.0f, 0.0f};
  for (uint32_t t = 0; t <= 300; t += STEP_MS) {
    extPoseHistoryPush(t, zero);
  }

  float d[3] = {1.0f, 1.0f, 1.0f};
  TEST_ASSERT_TRUE(extPoseHistoryDisplacement(300, 100, d));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, d[0]);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, d[1]);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, d[2]);
}
