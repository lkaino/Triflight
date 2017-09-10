/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <float.h>

#define MIXER_TRICOPTER_INTERNALS

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/filter.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/adc.h"
#include "drivers/time.h"

#include "rx/rx.h"

#include "io/beeper.h"
#include "io/motors.h"
#include "io/servos.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"

#include "flight/mixer.h"
#include "flight/mixer_tricopter.h"
#include "flight/imu.h"
#include "flight/pid.h"

#include "fc/fc_rc.h"
#include "fc/config.h"
#include "fc/runtime_config.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"

static const uint8_t TRI_TAIL_MOTOR_INDEX = 0;

// Use the first once at the top of every function that will use one of the other
#define InitDelayMeasurement_ms() const uint32_t now_ms = millis()
#define IsDelayElapsed_ms(timestamp_ms, delay_ms) ((uint32_t) (now_ms - timestamp_ms) >= delay_ms)
#define GetCurrentDelay_ms(timestamp_ms) (now_ms - timestamp_ms)
#define GetCurrentTime_ms() (now_ms)

enum {
    DEBUG_TRI_MOTOR_CORRECTION = 0,
    DEBUG_TRI_TAIL_MOTOR = 1,
    DEBUG_TRI_YAW_ERROR = 2,
    DEBUG_TRI_SERVO_OUTPUT_ANGLE_OR_TAIL_TUNE_STATE = 3,
};

static tailTune_t tailTune = { .mode = TT_MODE_NONE };
STATIC_UNIT_TESTED tailServo_t tailServo = { .angle = TRI_TAIL_SERVO_ANGLE_MID, .ADCChannel = ADC_RSSI };
STATIC_UNIT_TESTED tailMotor_t tailMotor = { .virtualFeedBack = 1000.0f };
//! Yaw output gain per servo angle. Index 0 is angle TRI_CURVE_FIRST_INDEX_ANGLE.
static float yawOutputGainCurve[TRI_YAW_FORCE_CURVE_SIZE];
//! Tail motor correction per servo angle. Index 0 is angle TRI_CURVE_FIRST_INDEX_ANGLE.
static float motorPitchCorrectionCurve[TRI_YAW_FORCE_CURVE_SIZE];
//! Configured output throttle range (max - min)
static triMixerConfig_t *gpTriMixerConfig;
static uint32_t preventArmingFlags = 0;

static void initYawForceCurve(void);
STATIC_UNIT_TESTED uint16_t getServoValueAtAngle(servoParam_t *servoConf, float angle);
static float getPitchCorrectionAtTailAngle(float angle, float thrustFactor);
STATIC_UNIT_TESTED float getAngleForYawOutput(float yawOutput);
STATIC_UNIT_TESTED float getServoAngle(servoParam_t *servoConf, uint16_t servoValue);
STATIC_UNIT_TESTED float binarySearchOutput(float yawOutput, float motorWoPitchCorr);
STATIC_UNIT_TESTED uint16_t getLinearServoValue(servoParam_t *servoConf, float scaledPIDOutput, float pidSumLimit);
static uint16_t getNormalServoValue(servoParam_t *servoConf, float constrainedPIDOutput, float pidSumLimit);
static float virtualServoStep(float currentAngle, int16_t servoSpeed, float dT, servoParam_t *servoConf,
        uint16_t servoValue);
static float feedbackServoStep(triMixerConfig_t *mixerConf, uint16_t tailServoADC);
STATIC_UNIT_TESTED void tailTuneModeThrustTorque(thrustTorque_t *pTT, const bool isThrottleHigh);
static void tailTuneModeServoSetup(struct servoSetup_t *pSS, servoParam_t *pServoConf, int16_t *pServoVal);
static void triTailTuneStep(servoParam_t *pServoConf, int16_t *pServoVal);
static void updateServoAngle(float dT);
static AdcChannel getServoFeedbackADCChannel(uint8_t tri_servo_feedback);
static void predictGyroOnDeceleration(void);
static void tailMotorStep(int16_t setpoint, float dT);
static int8_t triGetServoDirection(void);
static void preventArming(triArmingPreventFlag_e flag, _Bool enable);
static void checkArmingPrevent(void);
#if USE_AUX_CHANNEL_TUNING
static int16_t scaleAUXChannel(u8 channel, int16_t scale);
#endif

PG_REGISTER_WITH_RESET_FN(triMixerConfig_t, triMixerConfig, PG_TRICOPTER_CONFIG, 0);
#include "target.h"

#ifndef DEFAULT_SERVO_FEEDBACK_SOURCE
#define DEFAULT_SERVO_FEEDBACK_SOURCE TRI_SERVO_FB_VIRTUAL
#endif

void pgResetFn_triMixerConfig(triMixerConfig_t *triMixerConfig)
{
    triMixerConfig->tri_motor_acc_yaw_correction = 0;
    triMixerConfig->tri_motor_acceleration = 18;
    triMixerConfig->tri_servo_feedback = DEFAULT_SERVO_FEEDBACK_SOURCE;
    triMixerConfig->tri_servo_max_adc = 0;
    triMixerConfig->tri_servo_mid_adc = 0;
    triMixerConfig->tri_servo_min_adc = 0;
    triMixerConfig->tri_tail_motor_thrustfactor = 54; // Default for RCExplorer Baby tricopter
    triMixerConfig->tri_tail_servo_speed = 300; // Default for BMS-210DMH at 5V
    triMixerConfig->tri_yaw_boost = 240;
    triMixerConfig->tri_dynamic_yaw_maxthrottle = 38;
    triMixerConfig->tri_servo_angle_at_max = 40;
}

void triInitMixer(servoParam_t *pTailServoConfig, int16_t *pTailServoOutput)
{
    gpTriMixerConfig = triMixerConfigMutable();

    tailServo.pConf = pTailServoConfig;
    tailServo.pOutput = pTailServoOutput;
    tailServo.thrustFactor = gpTriMixerConfig->tri_tail_motor_thrustfactor / 10.0f;
    tailServo.maxDeflection = gpTriMixerConfig->tri_servo_angle_at_max;
    tailServo.angleAtMin = TRI_TAIL_SERVO_ANGLE_MID - tailServo.maxDeflection;
    tailServo.angleAtMax = TRI_TAIL_SERVO_ANGLE_MID + tailServo.maxDeflection;
    tailServo.speed = gpTriMixerConfig->tri_tail_servo_speed;
    tailServo.ADCChannel = getServoFeedbackADCChannel(gpTriMixerConfig->tri_servo_feedback);

    tailMotor.outputRange = mixGetMotorOutputHigh() - mixGetMotorOutputLow();
    tailMotor.minOutput = mixGetMotorOutputLow();
    tailMotor.linearMinOutput = tailMotor.outputRange * 0.05;
    tailMotor.pitchCorrectionGain = gpTriMixerConfig->tri_yaw_boost / 100.0f;

    const float tri_motor_acceleration_float = (float)gpTriMixerConfig->tri_motor_acceleration / 100;
    tailMotor.acceleration = (float) tailMotor.outputRange / tri_motor_acceleration_float;

    initYawForceCurve();
}

void triInitFilters()
{
    const float dT = getdT();
    pt1FilterInit(&tailMotor.feedbackFilter, TRI_MOTOR_FEEDBACK_LPF_CUTOFF_HZ, dT);
    pt1FilterInit(&tailServo.feedbackFilter, TRI_SERVO_FEEDBACK_LPF_CUTOFF_HZ, dT);
}

static float motorToThrust(float motor)
{
    return motor * (1.0f + (motor / tailMotor.outputRange) * 2.0f);
}

static void initYawForceCurve(void)
{
    // DERIVATE(1/(sin(x)-cos(x)/tailServoThrustFactor)) = 0
    // Multiplied by 10 to get decidegrees
    const int16_t minAngle = tailServo.angleAtMin;
    const int16_t maxAngle = tailServo.angleAtMax;
    float maxNegForce = 0;
    float maxPosForce = 0;

    tailServo.pitchZeroAngle = 2.0f
            * atanf((sqrtf(tailServo.thrustFactor * tailServo.thrustFactor + 1) + 1) / tailServo.thrustFactor);

    int16_t angle = TRI_CURVE_FIRST_INDEX_ANGLE;
    for (int32_t i = 0; i < TRI_YAW_FORCE_CURVE_SIZE; i++) {
        const float angleRad = DEGREES_TO_RADIANS(angle);

        yawOutputGainCurve[i] = (-tailServo.thrustFactor * cosf(angleRad) - sinf(angleRad));
        motorPitchCorrectionCurve[i] = tailMotor.pitchCorrectionGain * getPitchCorrectionAtTailAngle(angleRad, tailServo.thrustFactor);
        // Only calculate the top forces in the configured angle range
        if ((angle >= minAngle) && (angle <= maxAngle)) {
            maxNegForce = MIN(yawOutputGainCurve[i], maxNegForce);
            maxPosForce = MAX(yawOutputGainCurve[i], maxPosForce);
        }
        angle++;
    }

    float minLinearAngle;
    float maxLinearAngle;
    if (ABS(maxPosForce) < ABS(maxNegForce)) {
        const float maxOutput = ABS(maxPosForce);
        maxLinearAngle = maxAngle;
        const uint32_t indexMax = TRI_YAW_FORCE_CURVE_SIZE - 1;
        tailServo.maxYawOutput = maxOutput * motorToThrust(motorPitchCorrectionCurve[indexMax]);
        minLinearAngle = binarySearchOutput(-tailServo.maxYawOutput, 0);
    } else {
        // This would be the case if tail motor is spinning CW
        // Not supported yet
    }

    tailServo.angleAtLinearMin = minLinearAngle;
    tailServo.angleAtLinearMax = maxLinearAngle;
}

float triGetCurrentServoAngle(void)
{
    return tailServo.angle;
}

STATIC_UNIT_TESTED uint16_t getLinearServoValue(servoParam_t *servoConf, float scaledPIDOutput, float pidSumLimit)
{
    // maxYawOutput is the maximum output we can get at zero motor output with the pitch correction
    const float yawOutput = tailServo.maxYawOutput * scaledPIDOutput / pidSumLimit;
    const float correctedAngle = getAngleForYawOutput(yawOutput);
    const uint16_t linearServoValue = getServoValueAtAngle(servoConf, correctedAngle);

    DEBUG_SET(DEBUG_TRI, DEBUG_TRI_SERVO_OUTPUT_ANGLE_OR_TAIL_TUNE_STATE, correctedAngle * 10);

    return linearServoValue;
}

static uint16_t getNormalServoValue(servoParam_t *servoConf, float constrainedPIDOutput, float pidSumLimit)
{
    const int16_t angle = TRI_TAIL_SERVO_ANGLE_MID + constrainedPIDOutput / pidSumLimit * tailServo.maxDeflection;
    const uint16_t normalServoValue = getServoValueAtAngle(servoConf, angle);

    return normalServoValue;
}

void triServoMixer(float scaledYawPid, float pidSumLimit)
{
    const float dT = getdT();

    // Update the tail motor speed from feedback
    tailMotorStep(motor[TRI_TAIL_MOTOR_INDEX], dT);

    // Update the servo angle from feedback
    updateServoAngle(dT);

    // Correct the servo output to produce linear yaw output in armed state
    if (ARMING_FLAG(ARMED)) {
        *tailServo.pOutput = getLinearServoValue(tailServo.pConf, scaledYawPid, pidSumLimit);
    } else {
        *tailServo.pOutput = getNormalServoValue(tailServo.pConf, scaledYawPid, pidSumLimit);
    }

    // Run tail tune mode
    triTailTuneStep(tailServo.pConf, tailServo.pOutput);

    // Check for tail motor decelaration and determine expected produced yaw error
    predictGyroOnDeceleration();

    checkArmingPrevent();
}

int16_t triGetMotorCorrection(uint8_t motorIndex)
{
    uint16_t correction = 0;

    if (motorIndex == TRI_TAIL_MOTOR_INDEX) {
        // TODO: revise this function, is the speed up lag really needed? Test without.

        // Adjust tail motor speed based on servo angle. Check how much to adjust speed from pitch force curve based on servo angle.
        // Take motor speed up lag into account by shifting the phase of the curve
        // Not taking into account the motor braking lag (yet)
        const float servoAngle = triGetCurrentServoAngle();
        correction = getPitchCorrectionAtTailAngle(DEGREES_TO_RADIANS(servoAngle), tailServo.thrustFactor);

        // Multiply the correction to get more authority (yaw boost)
        if (isAirmodeActive() && tailServo.feedbackHealthy)
        {
            correction *= tailMotor.pitchCorrectionGain;
        }
        tailMotor.lastCorrection = correction;
        DEBUG_SET(DEBUG_TRI, DEBUG_TRI_MOTOR_CORRECTION, 1000 + correction);
    }

    return correction;
}

_Bool triIsEnabledServoUnarmed(void)
{
    const _Bool isEnabledServoUnarmed = (gpTriMixerConfig->tri_unarmed_servo != 0) || FLIGHT_MODE(TAILTUNE_MODE);

    return isEnabledServoUnarmed;
}

_Bool triMixerInUse(void)
{
    const _Bool isEnabledServoUnarmed = ((mixerConfig()->mixerMode == MIXER_TRI) || (mixerConfig()->mixerMode == MIXER_CUSTOM_TRI));

    return isEnabledServoUnarmed;
}

_Bool triIsServoSaturated(float rateError)
{
    if (fabsf(rateError) > TRI_SERVO_SATURED_GYRO_ERROR) {
        return true;
    } else {
        return false;
    }
}

STATIC_UNIT_TESTED uint16_t getServoValueAtAngle(servoParam_t *servoConf, float angle)
{
    const float servoMid = servoConf->middle;
    uint16_t servoValue;

    if (angle == TRI_TAIL_SERVO_ANGLE_MID) {
        servoValue = servoMid;
    } else {
        const int8_t direction = triGetServoDirection();
        const float angleRange = tailServo.maxDeflection;
        float angleDiff;
        int8_t servoRange; // -1 == min-mid, 1 == mid-max

        if (angle < TRI_TAIL_SERVO_ANGLE_MID) {
            angleDiff = TRI_TAIL_SERVO_ANGLE_MID - angle;
            if (direction > 0) {
                servoRange = -1;
            } else {
                servoRange = 1;
            }
        } else {
            angleDiff = angle - TRI_TAIL_SERVO_ANGLE_MID;
            if (direction > 0) {
                servoRange = 1;
            } else {
                servoRange = -1;
            }
        }
        if (servoRange < 0) {
            const float servoMin = servoConf->min;

            servoValue = servoMid - angleDiff * (servoMid - servoMin) / angleRange;
        } else {
            const float servoMax = servoConf->max;

            servoValue = lroundf(servoMid + angleDiff * (servoMax - servoMid) / angleRange);
        }
    }

    return servoValue;
}

static float getPitchCorrectionAtTailAngle(float angle, float thrustFactor)
{
    const float pitchCorrection = 1.0f / (sin_approx(angle) - cos_approx(angle) / thrustFactor);
    const float motorCorrection = (tailMotor.outputRange * pitchCorrection) - tailMotor.outputRange;
    return motorCorrection;
}

STATIC_UNIT_TESTED float binarySearchOutput(float yawOutput, float motorWoPitchCorr)
{
    int32_t lower = 0;
    int32_t higher = TRI_YAW_FORCE_CURVE_SIZE - 1;

    while (higher > lower + 1) {
        const int32_t mid = (lower + higher) / 2;
        if ((motorToThrust(motorWoPitchCorr + motorPitchCorrectionCurve[mid]) * yawOutputGainCurve[mid]) > yawOutput) {
            higher = mid;
        } else {
            lower = mid;
        }
    }

    // Interpolating
    const float outputLow = (motorToThrust(motorWoPitchCorr + motorPitchCorrectionCurve[lower])) * yawOutputGainCurve[lower];
    const float outputHigh = (motorToThrust(motorWoPitchCorr + motorPitchCorrectionCurve[higher])) * yawOutputGainCurve[higher];
    const float angleLow = TRI_CURVE_FIRST_INDEX_ANGLE + lower;
    const float angle = angleLow + (yawOutput - outputLow) / (outputHigh - outputLow);

    return angle;
}

STATIC_UNIT_TESTED float getAngleForYawOutput(float yawOutput)
{
    float angle;

    float motorWoPitchCorr = tailMotor.virtualFeedBack - tailMotor.minOutput - tailMotor.lastCorrection;
    motorWoPitchCorr = MAX(tailMotor.linearMinOutput, motorWoPitchCorr);
    if (yawOutput < (motorToThrust((motorWoPitchCorr + motorPitchCorrectionCurve[0])) * yawOutputGainCurve[0])) {
        // No force that low
        angle = tailServo.angleAtLinearMin;
    } else if (yawOutput > (motorToThrust((motorWoPitchCorr + motorPitchCorrectionCurve[TRI_YAW_FORCE_CURVE_SIZE - 1])) * yawOutputGainCurve[TRI_YAW_FORCE_CURVE_SIZE - 1])) {
        // No force that high
        angle = tailServo.angleAtLinearMax;
    } else {
        // Binary search: yawForceCurve[lower] <= force, yawForceCurve[higher] > force
        angle = binarySearchOutput(yawOutput, motorWoPitchCorr);

        if (angle < tailServo.angleAtLinearMin) {
            angle = tailServo.angleAtLinearMin;
        } else if (angle > tailServo.angleAtLinearMax) {
            angle = tailServo.angleAtLinearMax;
        }
    }

    return angle;
}

STATIC_UNIT_TESTED float getServoAngle(servoParam_t *servoConf, uint16_t servoValue)
{
    const int16_t midValue = servoConf->middle;
    const int16_t endValue = servoValue < midValue ? servoConf->min : servoConf->max;
    const float endAngle = servoValue < midValue ? tailServo.angleAtMin : tailServo.angleAtMax;
    const float servoAngle = (float)(endAngle - TRI_TAIL_SERVO_ANGLE_MID) * (float)(servoValue - midValue)
            / (endValue - midValue) + TRI_TAIL_SERVO_ANGLE_MID;

    return servoAngle;
}

static float virtualServoStep(float currentAngle, int16_t servoSpeed, float dT, servoParam_t *servoConf,
        uint16_t servoValue)
{
    const float angleSetPoint = getServoAngle(servoConf, servoValue);
    const float dA = dT * servoSpeed; // Max change of an angle since last check

    if (ABS(currentAngle - angleSetPoint) < dA) {
        // At set-point after this moment
        currentAngle = angleSetPoint;
    } else if (currentAngle < angleSetPoint) {
        currentAngle += dA;
    } else {
        // tailServoAngle.virtual > angleSetPoint
        currentAngle -= dA;
    }

    return currentAngle;
}

static float feedbackServoStep(triMixerConfig_t *mixerConf, uint16_t tailServoADC)
{
    // Feedback servo
    const int32_t ADCFeedback = tailServoADC;
    const int16_t midValue = mixerConf->tri_servo_mid_adc;
    const int16_t endValue = ADCFeedback < midValue ? mixerConf->tri_servo_min_adc : mixerConf->tri_servo_max_adc;
    const float tailServoMaxAngle = tailServo.angleAtMax;
    const float endAngle = ADCFeedback < midValue ?
            TRI_TAIL_SERVO_ANGLE_MID - tailServoMaxAngle : TRI_TAIL_SERVO_ANGLE_MID + tailServoMaxAngle;
    const float currentAngle = ((endAngle - TRI_TAIL_SERVO_ANGLE_MID) * (ADCFeedback - midValue)
            / (endValue - midValue) + TRI_TAIL_SERVO_ANGLE_MID);

    return currentAngle;
}

static void triTailTuneStep(servoParam_t *pServoConf, int16_t *pServoVal)
{
    if (!IS_RC_MODE_ACTIVE(BOXTAILTUNE)) {
        if (FLIGHT_MODE(TAILTUNE_MODE)) {
            preventArming(TRI_ARMING_PREVENT_FLAG_UNARMED_TAIL_TUNE, false);
            DISABLE_FLIGHT_MODE(TAILTUNE_MODE);
            tailTune.mode = TT_MODE_NONE;
        }
    } else {
        ENABLE_FLIGHT_MODE(TAILTUNE_MODE);
        if (tailTune.mode == TT_MODE_NONE) {
            if (ARMING_FLAG(ARMED)) {
                tailTune.mode = TT_MODE_THRUST_TORQUE;
                tailTune.tt.state = TT_IDLE;
            } else {
                // Prevent accidental arming in servo setup mode
                preventArming(TRI_ARMING_PREVENT_FLAG_UNARMED_TAIL_TUNE, true);
                tailTune.mode = TT_MODE_SERVO_SETUP;
                tailTune.ss.servoVal = pServoConf->middle;
            }
        }
        switch (tailTune.mode) {
        case TT_MODE_THRUST_TORQUE:
            tailTuneModeThrustTorque(&tailTune.tt,
                    (THROTTLE_HIGH == calculateThrottleStatus()));
            break;
        case TT_MODE_SERVO_SETUP:
            tailTuneModeServoSetup(&tailTune.ss, pServoConf, pServoVal);
            break;
        case TT_MODE_NONE:
            break;
        }
    }
}

STATIC_UNIT_TESTED void tailTuneModeThrustTorque(thrustTorque_t *pTT, const bool isThrottleHigh)
{
    InitDelayMeasurement_ms();
    switch (pTT->state) {
    case TT_IDLE:
        // Calibration has been requested, only start when throttle is up
        if (isThrottleHigh && ARMING_FLAG(ARMED)) {
            beeper(BEEPER_BAT_LOW);
            pTT->startBeepDelay_ms = 1000;
            pTT->timestamp_ms = GetCurrentTime_ms();
            pTT->timestamp2_ms = GetCurrentTime_ms();
            pTT->lastAdjTime_ms = GetCurrentTime_ms();
            pTT->state = TT_WAIT;
            pTT->servoAvgAngle.sum = 0;
            pTT->servoAvgAngle.numOf = 0;
            pTT->tailTuneGyroLimit = 4.5f;
        }
        break;
    case TT_WAIT:
        if (isThrottleHigh && ARMING_FLAG(ARMED)) {
            // Wait for 5 seconds before activating the tuning.
            // This is so that pilot has time to take off if the tail tune mode was activated on ground.
            if (IsDelayElapsed_ms(pTT->timestamp_ms, 5000)) {
                DEBUG_SET(DEBUG_TRI, DEBUG_TRI_SERVO_OUTPUT_ANGLE_OR_TAIL_TUNE_STATE, 1011);
                // Longer beep when starting
                beeper(BEEPER_BAT_CRIT_LOW);
                pTT->state = TT_ACTIVE;
                pTT->timestamp_ms = GetCurrentTime_ms();
            } else if (IsDelayElapsed_ms(pTT->timestamp_ms, pTT->startBeepDelay_ms)) {
                DEBUG_SET(DEBUG_TRI, DEBUG_TRI_SERVO_OUTPUT_ANGLE_OR_TAIL_TUNE_STATE, 1010);
                // Beep every second until start
                beeper(BEEPER_BAT_LOW);
                pTT->startBeepDelay_ms += 1000;
            }
        } else {
            pTT->state = TT_IDLE;
        }
        break;
    case TT_ACTIVE:
        if (!(isThrottleHigh
                && isRcAxisWithinDeadband(ROLL, TRI_TAIL_TUNE_MIN_DEADBAND)
                && isRcAxisWithinDeadband(PITCH, TRI_TAIL_TUNE_MIN_DEADBAND)
                && isRcAxisWithinDeadband(YAW, TRI_TAIL_TUNE_MIN_DEADBAND))) {
            pTT->timestamp_ms = GetCurrentTime_ms(); // sticks are NOT good
            DEBUG_SET(DEBUG_TRI, DEBUG_TRI_SERVO_OUTPUT_ANGLE_OR_TAIL_TUNE_STATE, 1100);
        }
        if (fabsf(gyro.gyroADCf[FD_YAW]) > pTT->tailTuneGyroLimit) {
            pTT->timestamp2_ms = GetCurrentTime_ms(); // gyro is NOT stable
            DEBUG_SET(DEBUG_TRI, DEBUG_TRI_SERVO_OUTPUT_ANGLE_OR_TAIL_TUNE_STATE, 1200);
        }
        if (IsDelayElapsed_ms(pTT->timestamp_ms, 200)) {
            DEBUG_SET(DEBUG_TRI, DEBUG_TRI_SERVO_OUTPUT_ANGLE_OR_TAIL_TUNE_STATE, 1300);
            // RC commands have been within deadbands for 250 ms
            if (IsDelayElapsed_ms(pTT->timestamp2_ms, 200)) {
                DEBUG_SET(DEBUG_TRI, DEBUG_TRI_SERVO_OUTPUT_ANGLE_OR_TAIL_TUNE_STATE, 1400);
                // Gyro has also been stable for 250 ms
                if (IsDelayElapsed_ms(pTT->lastAdjTime_ms, 20)) {
                    pTT->lastAdjTime_ms = GetCurrentTime_ms();
                    pTT->servoAvgAngle.sum += triGetCurrentServoAngle();
                    pTT->servoAvgAngle.numOf++;
                    DEBUG_SET(DEBUG_TRI, DEBUG_TRI_SERVO_OUTPUT_ANGLE_OR_TAIL_TUNE_STATE, 1600 + pTT->servoAvgAngle.numOf / 10);
                    if ((pTT->servoAvgAngle.numOf & 0x1f) == 0x00) {
                        // once every 32 times
                        beeperConfirmationBeeps(1);
                    }
                    if (pTT->servoAvgAngle.numOf >= 300) {
                        beeper(BEEPER_READY_BEEP);
                        pTT->state = TT_WAIT_FOR_DISARM;
                        pTT->timestamp_ms = GetCurrentTime_ms();
                    }
                }
            } else if (IsDelayElapsed_ms(pTT->lastAdjTime_ms, 300)) {
                // Sticks are OK but there has not been any valid samples in 1 s, try to loosen the gyro criteria a little
                pTT->tailTuneGyroLimit += 0.1f;
                pTT->lastAdjTime_ms = GetCurrentTime_ms();
                if (pTT->tailTuneGyroLimit > 10.0f) {
                    // If there are not enough samples by now it is a fail.
                    pTT->state = TT_FAIL;
                }
                DEBUG_SET(DEBUG_TRI, DEBUG_TRI_SERVO_OUTPUT_ANGLE_OR_TAIL_TUNE_STATE, 1500 + pTT->tailTuneGyroLimit * 10);
            }
        } else {
            DEBUG_SET(DEBUG_TRI, DEBUG_TRI_SERVO_OUTPUT_ANGLE_OR_TAIL_TUNE_STATE, 1250);
        }
        break;
    case TT_WAIT_FOR_DISARM:
        DEBUG_SET(DEBUG_TRI, DEBUG_TRI_SERVO_OUTPUT_ANGLE_OR_TAIL_TUNE_STATE, 1700 + pTT->tailTuneGyroLimit * 10);
        if (!ARMING_FLAG(ARMED)) {
            float averageServoAngle = pTT->servoAvgAngle.sum / pTT->servoAvgAngle.numOf;
            if (averageServoAngle > 90.5f && averageServoAngle < 120.f) {
                averageServoAngle -= 90.0f;
                averageServoAngle *= RAD;
                gpTriMixerConfig->tri_tail_motor_thrustfactor = 10.0f * cos_approx(averageServoAngle)
                        / sin_approx(averageServoAngle);

                saveConfigAndNotify();

                pTT->state = TT_DONE;
            } else {
                pTT->state = TT_FAIL;
            }
            pTT->timestamp_ms = GetCurrentTime_ms();
        } else {
            if (IsDelayElapsed_ms(pTT->timestamp_ms, 2000)) {
                beeper(BEEPER_READY_BEEP);
                pTT->timestamp_ms = GetCurrentTime_ms();
            }
        }
        break;
    case TT_DONE:
        DEBUG_SET(DEBUG_TRI, DEBUG_TRI_SERVO_OUTPUT_ANGLE_OR_TAIL_TUNE_STATE, 1800);
        if (IsDelayElapsed_ms(pTT->timestamp_ms, 2000)) {
            beeper(BEEPER_READY_BEEP);
            pTT->timestamp_ms = GetCurrentTime_ms();
        }
        break;
    case TT_FAIL:
        DEBUG_SET(DEBUG_TRI, DEBUG_TRI_SERVO_OUTPUT_ANGLE_OR_TAIL_TUNE_STATE, 1850);
        if (IsDelayElapsed_ms(pTT->timestamp_ms, 2000)) {
            beeper(BEEPER_ACC_CALIBRATION_FAIL);
            pTT->timestamp_ms = GetCurrentTime_ms();
        }
        break;
    }
}

static void tailTuneModeServoSetup(struct servoSetup_t *pSS, servoParam_t *pServoConf, int16_t *pServoVal)
{
    InitDelayMeasurement_ms();
    // Check mode select
    if (isRcAxisWithinDeadband(PITCH, TRI_TAIL_TUNE_MIN_DEADBAND) && (rcCommand[ROLL] < -100)) {
        pSS->servoVal = pServoConf->min;
        pSS->pLimitToAdjust = &pServoConf->min;
        beeperConfirmationBeeps(1);
        pSS->state = SS_SETUP;
    } else if (isRcAxisWithinDeadband(ROLL, TRI_TAIL_TUNE_MIN_DEADBAND) && (rcCommand[PITCH] > 100)) {
        pSS->servoVal = pServoConf->middle;
        pSS->pLimitToAdjust = &pServoConf->middle;
        beeperConfirmationBeeps(2);
        pSS->state = SS_SETUP;
    } else if (isRcAxisWithinDeadband(PITCH, TRI_TAIL_TUNE_MIN_DEADBAND) && (rcCommand[ROLL] > 100)) {
        pSS->servoVal = pServoConf->max;
        pSS->pLimitToAdjust = &pServoConf->max;
        beeperConfirmationBeeps(3);
        pSS->state = SS_SETUP;
    } else if (isRcAxisWithinDeadband(ROLL, TRI_TAIL_TUNE_MIN_DEADBAND) && (rcCommand[PITCH] < -100)) {
        pSS->state = SS_CALIB;
        pSS->cal.state = SS_C_IDLE;
    }
    switch (pSS->state) {
    case SS_IDLE:
        break;
    case SS_SETUP:
        if (!isRcAxisWithinDeadband(YAW, TRI_TAIL_TUNE_MIN_DEADBAND)) {
            pSS->servoVal += triGetServoDirection() * -1.0f * (float) rcCommand[YAW] * getdT();
            pSS->servoVal = constrainf(pSS->servoVal, 900.0f, 2100.0f);
            *pSS->pLimitToAdjust = pSS->servoVal;
        }
        break;
    case SS_CALIB:
        // State transition
        if ((pSS->cal.done == true) || (pSS->cal.state == SS_C_IDLE)) {
            if (pSS->cal.state == SS_C_IDLE) {
                pSS->cal.state = SS_C_CALIB_MIN_MID_MAX;
                pSS->cal.subState = SS_C_MIN;
                pSS->servoVal = pServoConf->min;
                pSS->cal.avg.pCalibConfig = &gpTriMixerConfig->tri_servo_min_adc;
            } else if (pSS->cal.state == SS_C_CALIB_SPEED) {
                pSS->state = SS_IDLE;
                pSS->cal.subState = SS_C_IDLE;
                beeper(BEEPER_READY_BEEP);
                // Speed calibration should be done as final step so this saves the min, mid, max and speed values.
                saveConfigAndNotify();
            } else {
                if (pSS->cal.state == SS_C_CALIB_MIN_MID_MAX) {
                    switch (pSS->cal.subState) {
                    case SS_C_MIN:
                        pSS->cal.subState = SS_C_MID;
                        pSS->servoVal = pServoConf->middle;
                        pSS->cal.avg.pCalibConfig = &gpTriMixerConfig->tri_servo_mid_adc;
                        break;
                    case SS_C_MID:
                        if (ABS(gpTriMixerConfig->tri_servo_min_adc - gpTriMixerConfig->tri_servo_mid_adc) < 100) {
                            // Not enough difference between min and mid feedback values.
                            // Most likely the feedback signal is not connected.
                            pSS->state = SS_IDLE;
                            pSS->cal.subState = SS_C_IDLE;
                            beeper(BEEPER_ACC_CALIBRATION_FAIL);
                            // Save configuration even after speed calibration failed.
                            // Speed calibration should be done as final step so this saves the min, mid and max values.
                            saveConfigAndNotify();
                        } else {
                            pSS->cal.subState = SS_C_MAX;
                            pSS->servoVal = pServoConf->max;
                            pSS->cal.avg.pCalibConfig = &gpTriMixerConfig->tri_servo_max_adc;
                        }
                        break;
                    case SS_C_MAX:
                        pSS->cal.state = SS_C_CALIB_SPEED;
                        pSS->cal.subState = SS_C_MIN;
                        pSS->servoVal = pServoConf->min;
                        pSS->cal.waitingServoToStop = true;
                        break;
                    }
                }
            }
            pSS->cal.timestamp_ms = GetCurrentTime_ms();
            pSS->cal.avg.sum = 0;
            pSS->cal.avg.numOf = 0;
            pSS->cal.done = false;
        }
        switch (pSS->cal.state) {
        case SS_C_IDLE:
            break;
        case SS_C_CALIB_MIN_MID_MAX:
            if (IsDelayElapsed_ms(pSS->cal.timestamp_ms, 500)) {
                if (IsDelayElapsed_ms(pSS->cal.timestamp_ms, 600)) {
                    *pSS->cal.avg.pCalibConfig = pSS->cal.avg.sum / pSS->cal.avg.numOf;
                    pSS->cal.done = true;
                } else {
                    pSS->cal.avg.sum += tailServo.ADCRaw;
                    pSS->cal.avg.numOf++;
                }
            }
            break;
        case SS_C_CALIB_SPEED:
            switch (pSS->cal.subState) {
            case SS_C_MIN:
                // Wait for the servo to reach min position
                if (tailServo.ADCRaw < (gpTriMixerConfig->tri_servo_min_adc + 10)) {
                    if (!pSS->cal.waitingServoToStop) {
                        pSS->cal.avg.sum += GetCurrentDelay_ms(pSS->cal.timestamp_ms);
                        pSS->cal.avg.numOf++;

                        if (pSS->cal.avg.numOf > 5) {
                            const float avgTime = pSS->cal.avg.sum / pSS->cal.avg.numOf;
                            const float avgServoSpeed = (2.0f * tailServo.maxDeflection) / avgTime * 1000.0f;

                            gpTriMixerConfig->tri_tail_servo_speed = avgServoSpeed;
                            tailServo.speed = gpTriMixerConfig->tri_tail_servo_speed;
                            pSS->cal.done = true;
                            pSS->servoVal = pServoConf->middle;
                        }
                        pSS->cal.timestamp_ms = GetCurrentTime_ms();
                        pSS->cal.waitingServoToStop = true;
                    }
                    // Wait for the servo to fully stop before starting speed measuring
                    else if (IsDelayElapsed_ms(pSS->cal.timestamp_ms, 200)) {
                        pSS->cal.timestamp_ms = GetCurrentTime_ms();
                        pSS->cal.subState = SS_C_MAX;
                        pSS->cal.waitingServoToStop = false;
                        pSS->servoVal = pServoConf->max;
                    }
                }
                break;
            case SS_C_MAX:
                // Wait for the servo to reach max position
                if (tailServo.ADCRaw > (gpTriMixerConfig->tri_servo_max_adc - 10)) {
                    if (!pSS->cal.waitingServoToStop) {
                        pSS->cal.avg.sum += GetCurrentDelay_ms(pSS->cal.timestamp_ms);
                        pSS->cal.avg.numOf++;
                        pSS->cal.timestamp_ms = GetCurrentTime_ms();
                        pSS->cal.waitingServoToStop = true;
                    } else if (IsDelayElapsed_ms(pSS->cal.timestamp_ms, 200)) {
                        pSS->cal.timestamp_ms = GetCurrentTime_ms();
                        pSS->cal.subState = SS_C_MIN;
                        pSS->cal.waitingServoToStop = false;
                        pSS->servoVal = pServoConf->min;
                    }
                }
                break;
            case SS_C_MID:
                // Should not come here
                break;
            }
        }
        break;
    }
    *pServoVal = pSS->servoVal;
}

static void updateServoAngle(float dT)
{
    if (gpTriMixerConfig->tri_servo_feedback == TRI_SERVO_FB_VIRTUAL) {
        tailServo.angle = virtualServoStep(tailServo.angle, tailServo.speed, dT, tailServo.pConf, *tailServo.pOutput);
    } else {
        // Read new servo feedback signal sample and run it through filter
        const uint16_t ADCRaw = pt1FilterApply(&tailServo.feedbackFilter, adcGetChannel(tailServo.ADCChannel));
        tailServo.angle = feedbackServoStep(gpTriMixerConfig, ADCRaw);
        tailServo.ADCRaw = ADCRaw;
    }

    if ((tailServo.angle < (TRI_TAIL_SERVO_INVALID_ANGLE_MIN)) ||
        (tailServo.angle > (TRI_TAIL_SERVO_INVALID_ANGLE_MAX))) {
        tailServo.feedbackHealthy = false;
        preventArming(TRI_ARMING_PREVENT_FLAG_INVALID_SERVO_ANGLE, true);
    } else {
        tailServo.feedbackHealthy = true;
        preventArming(TRI_ARMING_PREVENT_FLAG_INVALID_SERVO_ANGLE, false);
    }
    servo[6] = tailServo.angle * 10;
}

static AdcChannel getServoFeedbackADCChannel(uint8_t tri_servo_feedback)
{
    AdcChannel channel;
    switch (tri_servo_feedback) {
    case TRI_SERVO_FB_RSSI:
        channel = ADC_RSSI;
        break;
    case TRI_SERVO_FB_CURRENT:
        channel = ADC_CURRENT;
        break;
#ifdef EXTERNAL1_ADC_PIN
    case TRI_SERVO_FB_EXT1:
        channel = ADC_EXTERNAL1;
        break;
#endif
    default:
        channel = ADC_RSSI;
        break;
    }

    return channel;
}

static void predictGyroOnDeceleration(void)
{
    static float previousMotorSpeed = 1000.0f;

    if (gpTriMixerConfig->tri_motor_acc_yaw_correction > 0)
    {
        const float tailMotorSpeed = tailMotor.virtualFeedBack;
        // Calculate how much the motor speed changed since last time
        float acceleration = (tailMotorSpeed - previousMotorSpeed);

        previousMotorSpeed = tailMotorSpeed;
        float error = 0;
        if (acceleration < 0.0f)
        {
            // Tests have shown that this is mostly needed when throttle is cut (motor decelerating), so only
            // set the expected gyro error in that case.
            // Set the expected axis error based on tail motor acceleration and configured gain
            error = acceleration * gpTriMixerConfig->tri_motor_acc_yaw_correction * 10.0f;
            error *= sin_approx(DEGREES_TO_RADIANS(triGetCurrentServoAngle()));
        }

        DEBUG_SET(DEBUG_TRI, DEBUG_TRI_YAW_ERROR, 1000 + error);

        pidSetExpectedGyroError(FD_YAW, error);
    }
}

static void tailMotorStep(int16_t setpoint, float dT)
{
    static float current = 1000;
    const float dS = dT * tailMotor.acceleration; // Max change of an speed since last check

    if (ABS(current - setpoint) < dS) {
        // At set-point after this moment
        current = setpoint;
    } else if (current < setpoint) {
        current += dS;
    } else {
        current -= dS;
    }
    // Use a PT1 low-pass filter to add "slowness" to the virtual motor feedback.
    // Cut-off to delay:
    // 2  Hz -> 25 ms
    // 5  Hz -> 14 ms
    // 10 Hz -> 9  ms
    tailMotor.virtualFeedBack = pt1FilterApply(&tailMotor.feedbackFilter, current);
    DEBUG_SET(DEBUG_TRI, DEBUG_TRI_TAIL_MOTOR, tailMotor.virtualFeedBack);
}

static int8_t triGetServoDirection(void)
{
    const int8_t direction = (int8_t) servoDirection(SERVO_RUDDER, INPUT_STABILIZED_YAW);

    return direction;
}

static void preventArming(triArmingPreventFlag_e flag, _Bool enable)
{
    if (enable) {
        preventArmingFlags |= flag;
    } else {
        preventArmingFlags &= ~flag;
    }
}

static void checkArmingPrevent(void)
{
    if (preventArmingFlags) {
        ENABLE_ARMING_FLAG(ARMING_DISABLED_TRICOPTER);
    } else {
        DISABLE_ARMING_FLAG(ARMING_DISABLED_TRICOPTER);
    }
}

#if USE_AUX_CHANNEL_TUNING
static int16_t scaleAUXChannel(u8 channel, int16_t scale)
{
    int16_t constrained = constrain(rcData[channel], 1000, 2000);
    constrained -= 1000;

    return constrained * scale / 1000;
}
#endif
