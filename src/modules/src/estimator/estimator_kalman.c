/**
 * Authored by Michael Hamer (http://www.mikehamer.info), June 2016
 * Thank you to Mark Mueller (www.mwm.im) for advice during implementation,
 * and for derivation of the original filter in the below-cited paper.
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
 * ============================================================================
 *
 * The Kalman filter implemented in this file is based on the papers:
 *
 * "Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation"
 * http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=7139421
 *
 * and
 *
 * "Covariance Correction Step for Kalman Filtering with an Attitude"
 * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
 *
 * Academic citation would be appreciated.
 *
 * BIBTEX ENTRIES:
 * \verbatim
      @INPROCEEDINGS{MuellerHamerUWB2015,
      author  = {Mueller, Mark W and Hamer, Michael and D’Andrea, Raffaello},
      title   = {Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation},
      booktitle = {2015 IEEE International Conference on Robotics and Automation (ICRA)},
      year    = {2015},
      month   = {May},
      pages   = {1730-1736},
      doi     = {10.1109/ICRA.2015.7139421},
      ISSN    = {1050-4729}}

      @ARTICLE{MuellerCovariance2016,
      author={Mueller, Mark W and Hehn, Markus and D’Andrea, Raffaello},
      title={Covariance Correction Step for Kalman Filtering with an Attitude},
      journal={Journal of Guidance, Control, and Dynamics},
      pages={1--7},
      year={2016},
      publisher={American Institute of Aeronautics and Astronautics}}
 * \endverbatim
 *
 * ============================================================================
 *
 * MAJOR CHANGELOG:
 * 2016.06.28, Mike Hamer: Initial version
 * 2019.04.12, Kristoffer Richardsson: Refactored, separated kalman implementation from OS related functionality
 * 2021.03.15, Wolfgang Hoenig: Refactored queue handling
 */

#include <math.h>

#include "ext_pose_history.h"
#include "kalman_core.h"
#include "kalman_core_params_defaults.h"
#include "kalman_supervisor.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "sensors.h"
#include "static_mem.h"

#include "estimator.h"
#include "estimator_kalman.h"
#include "system.h"
#include "log.h"
#include "param.h"
#include "physicalConstants.h"
#include "supervisor.h"
#include "axis3fSubSampler.h"
#include "deck.h"

#include "statsCnt.h"
#include "rateSupervisor.h"

// Measurement models
#include "mm_distance.h"
#include "mm_absolute_height.h"
#include "mm_position.h"
#include "mm_pose.h"
#include "mm_tdoa.h"
#include "mm_flow.h"
#include "mm_tof.h"
#include "mm_yaw_error.h"
#include "mm_sweep_angles.h"

#include "mm_tdoa_robust.h"
#include "mm_distance_robust.h"

#define DEBUG_MODULE "ESTKALMAN"
#include "debug.h"
#include "cfassert.h"


// #define KALMAN_USE_BARO_UPDATE


// Semaphore to signal that we got data from the stabilizer loop to process
static SemaphoreHandle_t runTaskSemaphore;

// Mutex to protect data that is shared between the task and
// functions called by the stabilizer loop
static SemaphoreHandle_t dataMutex;
static StaticSemaphore_t dataMutexBuffer;


/**
 * Tuning parameters
 */
#define PREDICT_RATE RATE_100_HZ // this is slower than the IMU update rate of 1000Hz
const uint32_t PREDICTION_UPDATE_INTERVAL_MS = 1000 / PREDICT_RATE;

// The bounds on the covariance, these shouldn't be hit, but sometimes are... why?
#define MAX_COVARIANCE (100)
#define MIN_COVARIANCE (1e-6f)

// Use the robust implementations of TWR and TDoA, off by default but can be turned on through a parameter.
// The robust implementations use around 10% more CPU VS the standard flavours
static bool robustTwr = false;
static bool robustTdoa = false;
// Issue #80: compensate the latency of a delayed external (MOCAP / markerless)
// position or pose measurement before fusing it, using the per-sample age carried
// on the wire. Default off => behaviour identical to upstream; enable only after
// in-flight validation.
//
//   0 EXT_POSE_EXTRAP_OFF       fuse the measurement as received (upstream)
//   1 EXT_POSE_EXTRAP_VELOCITY  plan (b): add v_world * age, i.e. assume the current
//                               velocity held for the whole age window
//   2 EXT_POSE_EXTRAP_HISTORY   plan (c): add the position displacement the IMU
//                               prediction steps produced over the age window, summed
//                               from ext_pose_history. Uses the velocity the filter held
//                               at each instant rather than the current one, so it is
//                               exact for the mean on a curved path where v*age is not.
//                               Only the PREDICTION displacements are buffered, never the
//                               corrected state: deriving the shift from the corrected
//                               state fed each correction into the next and diverged on
//                               hardware (2026-09-23). The covariance is still the current
//                               P with the age-inflated R of plan (a) (locSrv.extPoseAgeStd*),
//                               NOT the historical covariance a full OOSM replay would use.
#define EXT_POSE_EXTRAP_OFF      0
#define EXT_POSE_EXTRAP_VELOCITY 1
#define EXT_POSE_EXTRAP_HISTORY  2
static uint8_t extPoseExtrapMode = EXT_POSE_EXTRAP_OFF;

// Safety cap against a bogus age injecting a jump. Also bounds the history lookup.
#define EXT_POSE_MAX_AGE_MS 500

// Magnitude of the last latency correction actually applied (m). Logged so a bag can
// prove the mechanism fired, rather than the campaign having to trust the param.
static float extrapLastNormM = 0.0f;

/**
 * Quadrocopter State
 *
 * The internally-estimated state is:
 * - X, Y, Z: the quad's position in the global frame
 * - PX, PY, PZ: the quad's velocity in its body frame
 * - D0, D1, D2: attitude error
 *
 * For more information, refer to the paper
 */

NO_DMA_CCM_SAFE_ZERO_INIT static kalmanCoreData_t coreData;

/**
 * Internal variables. Note that static declaration results in default initialization (to 0)
 */

static bool isInit = false;

static Axis3fSubSampler_t accSubSampler;
static Axis3fSubSampler_t gyroSubSampler;
static Axis3f accLatest;
static Axis3f gyroLatest;

static OutlierFilterTdoaState_t outlierFilterTdoaState;
static OutlierFilterLhState_t sweepOutlierFilterState;


// Indicates that the internal state is corrupt and should be reset
bool resetEstimation = false;

static kalmanCoreParams_t coreParams = {
  KALMAN_CORE_DEFAULT_PARAMS_INIT
};

// Data used to enable the task and stabilizer loop to run with minimal locking
static state_t taskEstimatorState; // The estimator state produced by the task, copied to the stabilizer when needed.

// Statistics
#define ONE_SECOND 1000
static STATS_CNT_RATE_DEFINE(updateCounter, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(predictionCounter, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(finalizeCounter, ONE_SECOND);
// static STATS_CNT_RATE_DEFINE(measurementAppendedCounter, ONE_SECOND);
// static STATS_CNT_RATE_DEFINE(measurementNotAppendedCounter, ONE_SECOND);

static rateSupervisor_t rateSupervisorContext;

#define WARNING_HOLD_BACK_TIME_MS 2000
static uint32_t warningBlockTimeMs = 0;

#ifdef KALMAN_USE_BARO_UPDATE
static const bool useBaroUpdate = true;
#else
static const bool useBaroUpdate = false;
#endif

static void kalmanTask(void* parameters);
static void updateQueuedMeasurements(const uint32_t nowMs, const bool quadIsFlying);

STATIC_MEM_TASK_ALLOC_STACK_NO_DMA_CCM_SAFE(kalmanTask, KALMAN_TASK_STACKSIZE);

// --------------------------------------------------

// Called one time during system startup
void estimatorKalmanTaskInit() {
  // It would be logical to set the params->attitudeReversion here, based on deck requirements, but the decks are
  // not initialized yet at this point so it is done in estimatorKalmanInit().

  // Created in the 'empty' state, meaning the semaphore must first be given, that is it will block in the task
  // until released by the stabilizer loop
  runTaskSemaphore = xSemaphoreCreateBinary();
  ASSERT(runTaskSemaphore);

  dataMutex = xSemaphoreCreateMutexStatic(&dataMutexBuffer);

  STATIC_MEM_TASK_CREATE(kalmanTask, kalmanTask, KALMAN_TASK_NAME, NULL, KALMAN_TASK_PRI);

  isInit = true;
}

bool estimatorKalmanTaskTest() {
  return isInit;
}

static void kalmanTask(void* parameters) {
  systemWaitStart();

  uint32_t nowMs = T2M(xTaskGetTickCount());
  uint32_t nextPredictionMs = nowMs;

  rateSupervisorInit(&rateSupervisorContext, nowMs, ONE_SECOND, PREDICT_RATE - 1, PREDICT_RATE + 1, 1);

  while (true) {
    xSemaphoreTake(runTaskSemaphore, portMAX_DELAY);
    nowMs = T2M(xTaskGetTickCount()); // would be nice if this had a precision higher than 1ms...

    if (resetEstimation) {
      estimatorKalmanInit();
      resetEstimation = false;
    }

    #ifdef CONFIG_ESTIMATOR_KALMAN_GENERAL_PURPOSE
    bool quadIsFlying = false;
    #else
    bool quadIsFlying = supervisorIsFlying();
    #endif

  #ifdef KALMAN_DECOUPLE_XY
    kalmanCoreDecoupleXY(&coreData);
  #endif

    // Run the system dynamics to predict the state forward.
    if (nowMs >= nextPredictionMs) {
      axis3fSubSamplerFinalize(&accSubSampler);
      axis3fSubSamplerFinalize(&gyroSubSampler);

      // Issue #80: record the displacement this prediction step produces, so a late
      // measurement can be shifted by the motion integrated over its age
      // (EXT_POSE_EXTRAP_HISTORY). Deliberately the prediction's own delta and not the
      // state, so measurement corrections never feed back into the shift.
      const float posBefore[3] = {coreData.S[KC_STATE_X], coreData.S[KC_STATE_Y], coreData.S[KC_STATE_Z]};
      kalmanCorePredict(&coreData, &coreParams, &accSubSampler.subSample, &gyroSubSampler.subSample, nowMs, quadIsFlying);
      nextPredictionMs = nowMs + PREDICTION_UPDATE_INTERVAL_MS;
      const float predictDelta[3] = {coreData.S[KC_STATE_X] - posBefore[0],
                                     coreData.S[KC_STATE_Y] - posBefore[1],
                                     coreData.S[KC_STATE_Z] - posBefore[2]};
      extPoseHistoryPush(nowMs, predictDelta);

      STATS_CNT_RATE_EVENT(&predictionCounter);

      if (!rateSupervisorValidate(&rateSupervisorContext, nowMs)) {
        DEBUG_PRINT("WARNING: Kalman prediction rate off (%lu)\n", rateSupervisorLatestCount(&rateSupervisorContext));
      }
    }

    // Add process noise every loop, rather than every prediction
    kalmanCoreAddProcessNoise(&coreData, &coreParams, nowMs);

    updateQueuedMeasurements(nowMs, quadIsFlying);

    if (kalmanCoreFinalize(&coreData))
    {
      STATS_CNT_RATE_EVENT(&finalizeCounter);
    }

    if (! kalmanSupervisorIsStateWithinBounds(&coreData)) {
      resetEstimation = true;

      if (nowMs > warningBlockTimeMs) {
        warningBlockTimeMs = nowMs + WARNING_HOLD_BACK_TIME_MS;
        DEBUG_PRINT("State out of bounds, resetting\n");
      }
    }

    /**
     * Finally, the internal state is externalized.
     * This is done every round, since the external state includes some sensor data
     */
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    kalmanCoreExternalizeState(&coreData, &taskEstimatorState, &accLatest);
    xSemaphoreGive(dataMutex);

    STATS_CNT_RATE_EVENT(&updateCounter);
  }
}

void estimatorKalman(state_t *state, const stabilizerStep_t stabilizerStep) {
  // This function is called from the stabilizer loop. It is important that this call returns
  // as quickly as possible. The dataMutex must only be locked short periods by the task.
  xSemaphoreTake(dataMutex, portMAX_DELAY);

  // Copy the latest state, calculated by the task
  memcpy(state, &taskEstimatorState, sizeof(state_t));
  xSemaphoreGive(dataMutex);

  xSemaphoreGive(runTaskSemaphore);
}

// Issue #80: shift a delayed external position/pose measurement forward to "now", so
// the EKF fuses it against the current state instead of introducing a systematic lag
// along the velocity vector. A no-op when disabled or when the age is 0 (unknown).
// See the EXT_POSE_EXTRAP_* comment above for what each mode does. Mode 2 falls back
// to mode 1 when the history cannot cover the age, so a fresh reset degrades rather
// than silently skipping the compensation.
static void extrapolateExternalPos(float pos[3], uint16_t captureAgeMs, uint32_t nowMs) {
  if (extPoseExtrapMode == EXT_POSE_EXTRAP_OFF || captureAgeMs == 0) {
    extrapLastNormM = 0.0f;  // the log must read 0 whenever nothing is applied
    return;
  }
  const uint16_t ageMs = captureAgeMs > EXT_POSE_MAX_AGE_MS ? EXT_POSE_MAX_AGE_MS : captureAgeMs;
  float d[3] = {0.0f, 0.0f, 0.0f};
  bool haveDelta = false;

  float vWorld[3];
  for (int i = 0; i < 3; i++) {
    vWorld[i] = coreData.R[i][0] * coreData.S[KC_STATE_PX]
              + coreData.R[i][1] * coreData.S[KC_STATE_PY]
              + coreData.R[i][2] * coreData.S[KC_STATE_PZ];
  }

  if (extPoseExtrapMode == EXT_POSE_EXTRAP_HISTORY) {
    if (extPoseHistoryDisplacement(nowMs, ageMs, d)) {
      // The history ends at the newest prediction step; cover the few ms since then
      // with the current velocity, capped so a tiny age never over-extends.
      uint32_t tailMs = nowMs - extPoseHistoryNewestMs();
      if (tailMs > ageMs) {
        tailMs = ageMs;
      }
      const float tailSec = (float)tailMs * 0.001f;
      for (int i = 0; i < 3; i++) {
        d[i] += vWorld[i] * tailSec;
      }
      haveDelta = true;
    }
  }

  if (!haveDelta) {
    const float ageSec = (float)ageMs * 0.001f;
    for (int i = 0; i < 3; i++) {
      d[i] = vWorld[i] * ageSec;
    }
  }

  for (int i = 0; i < 3; i++) {
    pos[i] += d[i];
  }
  extrapLastNormM = sqrtf(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
}

static void updateQueuedMeasurements(const uint32_t nowMs, const bool quadIsFlying) {
  /**
   * Sensor measurements can come in sporadically and faster than the stabilizer loop frequency,
   * we therefore consume all measurements since the last loop, rather than accumulating
   */

  // Pull the latest sensors values of interest; discard the rest
  measurement_t m;
  while (estimatorDequeue(&m)) {
    switch (m.type) {
      case MeasurementTypeTDOA:
        if(robustTdoa){
          // robust KF update with TDOA measurements
          kalmanCoreRobustUpdateWithTdoa(&coreData, &m.data.tdoa, &outlierFilterTdoaState);
        }else{
          // standard KF update
          kalmanCoreUpdateWithTdoa(&coreData, &m.data.tdoa, nowMs, &outlierFilterTdoaState);
        }
        break;
      case MeasurementTypePosition:
        extrapolateExternalPos(m.data.position.pos, m.data.position.captureAgeMs, nowMs);
        kalmanCoreUpdateWithPosition(&coreData, &m.data.position);
        break;
      case MeasurementTypePose:
        extrapolateExternalPos(m.data.pose.pos, m.data.pose.captureAgeMs, nowMs);
        kalmanCoreUpdateWithPose(&coreData, &m.data.pose);
        break;
      case MeasurementTypeDistance:
        if(robustTwr){
            // robust KF update with UWB TWR measurements
            kalmanCoreRobustUpdateWithDistance(&coreData, &m.data.distance);
        }else{
            // standard KF update
            kalmanCoreUpdateWithDistance(&coreData, &m.data.distance);
        }
        break;
      case MeasurementTypeTOF:
        kalmanCoreUpdateWithTof(&coreData, &m.data.tof);
        break;
      case MeasurementTypeAbsoluteHeight:
        kalmanCoreUpdateWithAbsoluteHeight(&coreData, &m.data.height);
        break;
      case MeasurementTypeFlow:
        kalmanCoreUpdateWithFlow(&coreData, &m.data.flow, &gyroLatest);
        break;
      case MeasurementTypeYawError:
        kalmanCoreUpdateWithYawError(&coreData, &m.data.yawError);
        break;
      case MeasurementTypeSweepAngle:
        kalmanCoreUpdateWithSweepAngles(&coreData, &m.data.sweepAngle, nowMs, &sweepOutlierFilterState);
        break;
      case MeasurementTypeGyroscope:
        axis3fSubSamplerAccumulate(&gyroSubSampler, &m.data.gyroscope.gyro);
        gyroLatest = m.data.gyroscope.gyro;
        break;
      case MeasurementTypeAcceleration:
        axis3fSubSamplerAccumulate(&accSubSampler, &m.data.acceleration.acc);
        accLatest = m.data.acceleration.acc;
        break;
      case MeasurementTypeBarometer:
        if (useBaroUpdate) {
          kalmanCoreUpdateWithBaro(&coreData, &coreParams, m.data.barometer.baro.asl, quadIsFlying);
        }
        break;
      default:
        break;
    }
  }
}

// Called when this estimator is activated
void estimatorKalmanInit(void)
{
  #ifdef CONFIG_DECK_LOCO_2D_POSITION
  coreParams.attitudeReversion = 0.0f;
  #else
  if (deckGetRequiredKalmanEstimatorAttitudeReversionOff())
  {
    coreParams.attitudeReversion = 0.0f;
    DEBUG_PRINT("Attitude reversion deactivated by deck\n");
  }
  #endif

  axis3fSubSamplerInit(&accSubSampler, GRAVITY_MAGNITUDE);
  axis3fSubSamplerInit(&gyroSubSampler, DEG_TO_RAD);

  outlierFilterTdoaReset(&outlierFilterTdoaState);
  outlierFilterLighthouseReset(&sweepOutlierFilterState, 0);

  extPoseHistoryReset();
  extrapLastNormM = 0.0f;

  uint32_t nowMs = T2M(xTaskGetTickCount());
  kalmanCoreInit(&coreData, &coreParams, nowMs);
}

bool estimatorKalmanTest(void)
{
  return isInit;
}

void estimatorKalmanGetEstimatedPos(point_t* pos) {
  pos->x = coreData.S[KC_STATE_X];
  pos->y = coreData.S[KC_STATE_Y];
  pos->z = coreData.S[KC_STATE_Z];
}

void estimatorKalmanGetEstimatedRot(float * rotationMatrix) {
  memcpy(rotationMatrix, coreData.R, 9*sizeof(float));
}

/**
 * Variables and results from the Extended Kalman Filter
 */
LOG_GROUP_START(kalman)
 /**
 * @brief Issue #80: magnitude (m) of the last external-pose latency correction that
 * was actually applied. 0 while compensation is off, the age is unknown, or the drone
 * is stationary. Streamed so a bag can prove the mechanism fired on that trial.
 */
  LOG_ADD(LOG_FLOAT, extrapNorm, &extrapLastNormM)
 /**
 * @brief Issue #80: active external-pose latency compensation mode (0 off, 1 v*age,
 * 2 state-history displacement). Mirrors the kalman.extPoseExtrap parameter.
 */
  LOG_ADD(LOG_UINT8, extrapMode, &extPoseExtrapMode)
 /**
 * @brief State position in the global frame x
 *
 *   Note: This is similar to stateEstimate.x.
 */
  LOG_ADD(LOG_FLOAT, stateX, &coreData.S[KC_STATE_X])
 /**
 * @brief State position in the global frame y
 *
 *  Note: This is similar to stateEstimate.y
 */
  LOG_ADD(LOG_FLOAT, stateY, &coreData.S[KC_STATE_Y])
 /**
 * @brief State position in the global frame z
 *
 *  Note: This is similar to stateEstimate.z
 */
  LOG_ADD(LOG_FLOAT, stateZ, &coreData.S[KC_STATE_Z])
  /**
  * @brief State velocity in its body frame x
  *
  *  Note: This should be part of stateEstimate
  */
  LOG_ADD(LOG_FLOAT, statePX, &coreData.S[KC_STATE_PX])
  /**
  * @brief State velocity in its body frame y
  *
  *  Note: This should be part of stateEstimate
  */
  LOG_ADD(LOG_FLOAT, statePY, &coreData.S[KC_STATE_PY])
  /**
  * @brief State velocity in its body frame z
  *
  *  Note: This should be part of stateEstimate
  */
  LOG_ADD(LOG_FLOAT, statePZ, &coreData.S[KC_STATE_PZ])
  /**
  * @brief State attitude error roll
  */
  LOG_ADD(LOG_FLOAT, stateD0, &coreData.S[KC_STATE_D0])
  /**
  * @brief State attitude error pitch
  */
  LOG_ADD(LOG_FLOAT, stateD1, &coreData.S[KC_STATE_D1])
  /**
  * @brief State attitude error yaw
  */
  LOG_ADD(LOG_FLOAT, stateD2, &coreData.S[KC_STATE_D2])
  /**
  * @brief Covariance matrix position x
  */
  LOG_ADD(LOG_FLOAT, varX, &coreData.P[KC_STATE_X][KC_STATE_X])
  /**
  * @brief Covariance matrix position y
  */
  LOG_ADD(LOG_FLOAT, varY, &coreData.P[KC_STATE_Y][KC_STATE_Y])
  /**
  * @brief Covariance matrix position z
  */
  LOG_ADD(LOG_FLOAT, varZ, &coreData.P[KC_STATE_Z][KC_STATE_Z])
  /**
  * @brief Covariance matrix velocity x
  */
  LOG_ADD(LOG_FLOAT, varPX, &coreData.P[KC_STATE_PX][KC_STATE_PX])
  /**
  * @brief Covariance matrix velocity y
  */
  LOG_ADD(LOG_FLOAT, varPY, &coreData.P[KC_STATE_PY][KC_STATE_PY])
  /**
  * @brief Covariance matrix velocity z
  */
  LOG_ADD(LOG_FLOAT, varPZ, &coreData.P[KC_STATE_PZ][KC_STATE_PZ])
  /**
  * @brief Covariance matrix attitude error roll
  */
  LOG_ADD(LOG_FLOAT, varD0, &coreData.P[KC_STATE_D0][KC_STATE_D0])
  /**
  * @brief Covariance matrix attitude error pitch
  */
  LOG_ADD(LOG_FLOAT, varD1, &coreData.P[KC_STATE_D1][KC_STATE_D1])
  /**
  * @brief Covariance matrix attitude error yaw
  */
  LOG_ADD(LOG_FLOAT, varD2, &coreData.P[KC_STATE_D2][KC_STATE_D2])
  /**
  * @brief Estimated Attitude quarternion w
  */
  LOG_ADD(LOG_FLOAT, q0, &coreData.q[0])
  /**
  * @brief Estimated Attitude quarternion x
  */
  LOG_ADD(LOG_FLOAT, q1, &coreData.q[1])
  /**
  * @brief Estimated Attitude quarternion y
  */
  LOG_ADD(LOG_FLOAT, q2, &coreData.q[2])
  /**
  * @brief Estimated Attitude quarternion z
  */
  LOG_ADD(LOG_FLOAT, q3, &coreData.q[3])
  /**
  * @brief Statistics rate of update step
  */
  STATS_CNT_RATE_LOG_ADD(rtUpdate, &updateCounter)
  /**
  * @brief Statistics rate of prediction step
  */
  STATS_CNT_RATE_LOG_ADD(rtPred, &predictionCounter)
  /**
  * @brief Statistics rate full estimation step
  */
  STATS_CNT_RATE_LOG_ADD(rtFinal, &finalizeCounter)
LOG_GROUP_STOP(kalman)

LOG_GROUP_START(outlierf)
  LOG_ADD(LOG_INT32, lhWin, &sweepOutlierFilterState.openingWindowMs)
LOG_GROUP_STOP(outlierf)

/**
 * Tuning parameters for the Extended Kalman Filter (EKF)
 *     estimator
 */
PARAM_GROUP_START(kalman)
/**
 * @brief Reset the kalman estimator
 */
  PARAM_ADD_CORE(PARAM_UINT8, resetEstimation, &resetEstimation)
/**
 * @brief Nonzero to use robust TDOA method (default: 0)
 */
  PARAM_ADD_CORE(PARAM_UINT8, robustTdoa, &robustTdoa)
/**
 * @brief Nonzero to use robust TWR method (default: 0)
 */
  PARAM_ADD_CORE(PARAM_UINT8, robustTwr, &robustTwr)
/**
 * @brief Latency compensation for delayed external (MOCAP / markerless) pose and
 * position, using the per-sample age carried on the wire (issue #80). Default 0.
 * 0 = off (upstream behaviour), 1 = add v*age, 2 = add the filter's own integrated
 * displacement over the age window (exact for the mean on curved trajectories).
 */
  PARAM_ADD_CORE(PARAM_UINT8, extPoseExtrap, &extPoseExtrapMode)
/**
 * @brief Process noise for x and y acceleration
 */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, pNAcc_xy, &coreParams.procNoiseAcc_xy)
/**
 * @brief Process noise for z acceleration
 */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, pNAcc_z, &coreParams.procNoiseAcc_z)
  /**
 * @brief Process noise for velocity
 */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, pNVel, &coreParams.procNoiseVel)
  /**
 * @brief Process noise for position
 */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, pNPos, &coreParams.procNoisePos)
  /**
 * @brief Process noise for attitude
 */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, pNAtt, &coreParams.procNoiseAtt)
  /**
 * @brief Measurement noise for barometer
 */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, mNBaro, &coreParams.measNoiseBaro)
  /**
 * @brief Measurement noise for roll/pitch gyros
 */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, mNGyro_rollpitch, &coreParams.measNoiseGyro_rollpitch)
  /**
 * @brief Measurement noise for yaw gyro
 */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, mNGyro_yaw, &coreParams.measNoiseGyro_yaw)
  /**
 * @brief Initial X after reset [m]
 */
  PARAM_ADD_CORE(PARAM_FLOAT, initialX, &coreParams.initialX)
  /**
 * @brief Initial Y after reset [m]
 */
  PARAM_ADD_CORE(PARAM_FLOAT, initialY, &coreParams.initialY)
  /**
 * @brief Initial Z after reset [m]
 */
  PARAM_ADD_CORE(PARAM_FLOAT, initialZ, &coreParams.initialZ)
  /**
 * @brief Initial yaw after reset [rad]
 */
  PARAM_ADD_CORE(PARAM_FLOAT, initialYaw, &coreParams.initialYaw)
  /**
  * @brief Drag in x direction (in N*s/m)
  */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, dragB_x, &coreParams.dragB_x)
  /**
   * @brief Drag in y direction (in N*s/m)
   */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, dragB_y, &coreParams.dragB_y)
  /**
   * @brief Drag in z direction (in N*s/m)
   */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, dragB_z, &coreParams.dragB_z)
  /**
   * @brief Center of pressure X (in meters)
   */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, cop_x, &coreParams.cop_x)
  /**
   * @brief Center of pressure Y (in meters)
   */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, cop_y, &coreParams.cop_y)
  /**
   * @brief Center of pressure Z (in meters)
   */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, cop_z, &coreParams.cop_z)
 
 PARAM_GROUP_STOP(kalman)
