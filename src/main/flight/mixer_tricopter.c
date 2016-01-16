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
#include "platform.h"
#include "debug.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/encoding.h"
#include "common/utils.h"

#include "drivers/gpio.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/compass.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/accgyro.h"
#include "drivers/light_led.h"

#include "sensors/sensors.h"
#include "sensors/boardalignment.h"
#include "sensors/sonar.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"

#include "io/beeper.h"
#include "io/display.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/serial_cli.h"
#include "io/serial_msp.h"
#include "io/statusindicator.h"

#include "rx/rx.h"
#include "rx/msp.h"

#include "telemetry/telemetry.h"

#include "flight/mixer.h"
#include "flight/mixer_tricopter.h"
#include "flight/altitudehold.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/navigation.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#include "mw.h"


#ifdef USE_SERVOS
#define TRI_TAIL_SERVO_ANGLE_MID (900)
#define TRI_YAW_FORCE_CURVE_SIZE (100)
#define TRI_TAIL_SERVO_MAX_ANGLE (500)

#define SERVO_CALIB_NUM_OF_MEAS  (5)

#define TT_CALIB_I_TARGET       (8)
#define TT_CALIB_I_LARGE_INCREMENT_LIMIT (10)

#define IsDelayElapsed_us(timestamp_us, delay_us) ((uint32_t)(micros() - timestamp_us) >= delay_us)
#define IsDelayElapsed_ms(timestamp_ms, delay_ms) ((uint32_t)(millis() - timestamp_ms) >= delay_ms)

typedef enum {
    TTC_IDLE = 0,
    TTC_ACTIVE,
    TTC_CHECK_WITHIN_LIMITS,
    TTC_DONE,
    TTC_FAIL,
} thrTrqCalibState_e;

typedef struct thrTrqCalibration_s {
    uint32_t timestamp_ms;
    uint32_t lastAdjTime_ms;
    thrTrqCalibState_e state;
    uint16_t target;
} thrTrqCalibration_t;

#endif

#ifdef USE_SERVOS
extern float dT;
extern master_t masterConfig;

static thrTrqCalibration_t thrTrqCalib = {.state = TTC_IDLE};
static int32_t tailServoMaxYawForce = 0;
 float tailServoThrustFactor = 0;
static int16_t tailServoMaxAngle = 0;
static int16_t tailServoSpeed = 0;
static float virtualServoAngle = TRI_TAIL_SERVO_ANGLE_MID / 10.0f;
static int32_t yawForceCurve[TRI_YAW_FORCE_CURVE_SIZE];
static int16_t tailMotorPitchZeroAngle;
static int16_t tailMotorAccelerationDelay_ms = 30;
static int16_t tailMotorDecelerationDelay_ms = 100;
static int16_t tailMotorAccelerationDelay_angle;
static int16_t tailMotorDecelerationDelay_angle;

static servoParam_t * gpTailServoConf;
static int16_t *gpTailServo;
static mixerConfig_t *gpMixerConfig;

static void initCurves();
static uint16_t getServoValueAtAngle(servoParam_t * servoConf, uint16_t angle);
static float getPitchCorrectionAtTailAngle(float angle);
static uint16_t getAngleFromYawCurveAtForce(int32_t force);
static uint16_t getServoAngle(servoParam_t * servoConf, uint16_t servoValue);
static uint16_t getPitchCorrectionMaxPhaseShift(int16_t servoAngle,
        int16_t servoSetpointAngle,
        int16_t motorAccelerationDelayAngle,
        int16_t motorDecelerationDelayAngle,
        int16_t motorDirectionChangeAngle);
static uint16_t getLinearServoValue(servoParam_t *servoConf, uint16_t servoValue);
static void virtualServoStep(float dT, servoParam_t *servoConf, uint16_t servoValue);
static void triThrustTorqueCalibrationStep();

#endif

void triInitMixer(servoParam_t *pTailServoConfig,
        int16_t *pTailServo,
        mixerConfig_t *pMixerConfig)
{
    gpTailServoConf = pTailServoConfig;
    gpTailServo = pTailServo;
    tailServoThrustFactor = pMixerConfig->tri_tail_motor_thrustfactor / 10.0f;
    tailServoMaxAngle = pMixerConfig->tri_servo_angle_at_max;
    tailServoSpeed = pMixerConfig->tri_tail_servo_speed;
    gpMixerConfig = pMixerConfig;

    initCurves();
}

static void initCurves()
{
    // DERIVATE(1/(sin(x)-cos(x)/tailServoThrustFactor)) = 0
    // Multiplied by 10 to get decidegrees
    tailMotorPitchZeroAngle = 10.0f * 2.0f * (atanf(((sqrtf(tailServoThrustFactor * tailServoThrustFactor + 1) + 1) / tailServoThrustFactor)));

    tailMotorAccelerationDelay_angle = 10.0f * (tailMotorAccelerationDelay_ms / 1000.0f) * tailServoSpeed;
    tailMotorDecelerationDelay_angle = 10.0f * (tailMotorDecelerationDelay_ms / 1000.0f) * tailServoSpeed;

    const int16_t minAngle = TRI_TAIL_SERVO_ANGLE_MID - tailServoMaxAngle;
    const int16_t maxAngle = TRI_TAIL_SERVO_ANGLE_MID + tailServoMaxAngle;
    int32_t maxNegForce = 0;
    int32_t maxPosForce = 0;

    int16_t angle = TRI_TAIL_SERVO_ANGLE_MID - TRI_TAIL_SERVO_MAX_ANGLE;
    for (int32_t i = 0; i < TRI_YAW_FORCE_CURVE_SIZE; i++)
    {
        const float angleRad = DEGREES_TO_RADIANS(angle / 10.0f);
        yawForceCurve[i] = 1000.0f * (-tailServoThrustFactor * cosf(angleRad) - sinf(angleRad)) * getPitchCorrectionAtTailAngle(angleRad);
        // Only calculate the top forces in the configured angle range
        if ((angle >= minAngle) && (angle <= maxAngle))
        {
            maxNegForce = MIN(yawForceCurve[i], maxNegForce);
            maxPosForce = MAX(yawForceCurve[i], maxPosForce);
        }
        angle += 10;
    }

    tailServoMaxYawForce = MIN(ABS(maxNegForce), ABS(maxPosForce));
}

float triGetVirtualServoAngle()
{
    return virtualServoAngle;
}

static uint16_t getLinearServoValue(servoParam_t *servoConf, uint16_t servoValue)
{
    const int16_t servoMid = servoConf->middle;
    // First find the yaw force at given servo value from a linear curve
    const int16_t servoRange = (servoValue < servoMid) ? servoMid - servoConf->min : servoConf->max - servoMid;
    const int32_t linearYawForceAtValue = (int32_t)(tailServoMaxYawForce) * (servoValue - servoMid) / servoRange;
    const int16_t correctedAngle = getAngleFromYawCurveAtForce(linearYawForceAtValue);
    return getServoValueAtAngle(servoConf, correctedAngle);
}

void triServoMixer()
{
    if (ARMING_FLAG(ARMED))
    {
        *gpTailServo = getLinearServoValue(gpTailServoConf, *gpTailServo);
        triThrustTorqueCalibrationStep();
    }

    virtualServoStep(dT, gpTailServoConf, *gpTailServo);
}

int16_t triGetMotorCorrection(uint8_t motorIndex)
{
    uint16_t correction = 0;
    if (motorIndex == 0)
    {
        // Adjust tail motor speed based on servo angle. Check how much to adjust speed from pitch force curve based on servo angle.
        // Take motor speed up lag into account by shifting the phase of the curve
        // Not taking into account the motor braking lag (yet)
        const int16_t servoAngle = triGetVirtualServoAngle() * 10.0f;
        const int16_t servoSetpointAngle = getServoAngle(gpTailServoConf, *gpTailServo);

        const uint16_t maxPhaseShift = getPitchCorrectionMaxPhaseShift(servoAngle, servoSetpointAngle, tailMotorAccelerationDelay_angle, tailMotorDecelerationDelay_angle, tailMotorPitchZeroAngle);

        int16_t angleDiff = servoSetpointAngle - servoAngle;
        if (ABS(angleDiff) > maxPhaseShift)
        {
            angleDiff = (int32_t)maxPhaseShift * angleDiff / ABS(angleDiff);
        }

        const int16_t futureServoAngle = constrain(servoAngle + angleDiff, TRI_TAIL_SERVO_ANGLE_MID - tailServoMaxAngle, TRI_TAIL_SERVO_ANGLE_MID + tailServoMaxAngle);
        const uint16_t throttleMotorOutput = rcCommand[THROTTLE] - getCurrentMinthrottle();
        correction = (throttleMotorOutput * getPitchCorrectionAtTailAngle(DEGREES_TO_RADIANS(futureServoAngle / 10.0f))) - throttleMotorOutput;
    }

    return correction;
}

static uint16_t getServoValueAtAngle(servoParam_t *servoConf, uint16_t angle)
{
    const int16_t servoMid = servoConf->middle;
    uint16_t servoValue;

    if (angle < TRI_TAIL_SERVO_ANGLE_MID)
    {
        const int16_t servoMin = servoConf->min;
        servoValue = (int32_t)(angle - tailServoMaxAngle) * (servoMid - servoMin) / (TRI_TAIL_SERVO_ANGLE_MID - tailServoMaxAngle) + servoMin;
    }
    else if (angle > TRI_TAIL_SERVO_ANGLE_MID)
    {
        servoValue = (int32_t)(angle - TRI_TAIL_SERVO_ANGLE_MID) * (servoConf->max - servoMid) / tailServoMaxAngle + servoMid;
    }
    else
    {
        servoValue = servoMid;
    }

    return servoValue;
}

static float getPitchCorrectionAtTailAngle(float angle)
{
    return 1 / (sin_approx(angle) - cos_approx(angle) / tailServoThrustFactor);
}

static uint16_t getAngleFromYawCurveAtForce(int32_t force)
{
    if (force < yawForceCurve[0]) // No force that low
    {
        return TRI_TAIL_SERVO_ANGLE_MID - TRI_TAIL_SERVO_MAX_ANGLE;
    }
    else if (!(force < yawForceCurve[TRI_YAW_FORCE_CURVE_SIZE - 1])) // No force that high
    {
        return TRI_TAIL_SERVO_ANGLE_MID + TRI_TAIL_SERVO_MAX_ANGLE;
    }
    // Binary search: yawForceCurve[lower] <= force, yawForceCurve[higher] > force
    int32_t lower = 0, higher = TRI_YAW_FORCE_CURVE_SIZE - 1;
    while (higher > lower + 1)
    {
        const int32_t mid = (lower + higher) / 2;
        if (yawForceCurve[mid] > force)
        {
            higher = mid;
        }
        else
        {
            lower = mid;
        }
    }
    // Interpolating
    return TRI_TAIL_SERVO_ANGLE_MID - TRI_TAIL_SERVO_MAX_ANGLE + lower * 10 + (int32_t)(force - yawForceCurve[lower]) * 10 / (yawForceCurve[higher] - yawForceCurve[lower]);
}

static uint16_t getServoAngle(servoParam_t *servoConf, uint16_t servoValue)
{
    const int16_t midValue = servoConf->middle;
    const int16_t endValue = servoValue < midValue ? servoConf->min : servoConf->max;
    const int16_t endAngle = servoValue < midValue ? TRI_TAIL_SERVO_ANGLE_MID - tailServoMaxAngle : TRI_TAIL_SERVO_ANGLE_MID + tailServoMaxAngle;
    const int16_t servoAngle = (int32_t)(endAngle - TRI_TAIL_SERVO_ANGLE_MID) * (servoValue - midValue) / (endValue - midValue) + TRI_TAIL_SERVO_ANGLE_MID;
    return servoAngle;
}

static uint16_t getPitchCorrectionMaxPhaseShift(int16_t servoAngle,
        int16_t servoSetpointAngle,
        int16_t motorAccelerationDelayAngle,
        int16_t motorDecelerationDelayAngle,
        int16_t motorDirectionChangeAngle)
{
    uint16_t maxPhaseShift;

    if (((servoAngle > servoSetpointAngle) && (servoAngle >= (motorDirectionChangeAngle + motorAccelerationDelayAngle))) ||
        ((servoAngle < servoSetpointAngle) && (servoAngle <= (motorDirectionChangeAngle - motorAccelerationDelayAngle))))
    {
        // Motor is braking
        maxPhaseShift = ABS(servoAngle - motorDirectionChangeAngle) >= motorDecelerationDelayAngle ?
                motorDecelerationDelayAngle:
                ABS(servoAngle - motorDirectionChangeAngle);
    }
    else
    {
        // Motor is accelerating
        maxPhaseShift = motorAccelerationDelayAngle;
    }

    return maxPhaseShift;
}

static void virtualServoStep(float dT, servoParam_t *servoConf, uint16_t servoValue)
{
    const float angleSetPoint = getServoAngle(servoConf, servoValue) / 10.0f;
    const float dA = dT * tailServoSpeed; // Max change of an angle since last check
    if ( fabsf(virtualServoAngle - angleSetPoint) < dA )
    {
        // At set-point after this moment
        virtualServoAngle = angleSetPoint;
    }
    else if (virtualServoAngle < angleSetPoint)
    {
        virtualServoAngle += dA;
    }
    else // virtualServoAngle > angleSetPoint
    {
        virtualServoAngle -= dA;
    }
}

static void triThrustTorqueCalibrationStep()
{
    if (!IS_RC_MODE_ACTIVE(BOXTAILTUNE))
    {
        if (FLIGHT_MODE(TAILTUNE_MODE))
        {
            DISABLE_FLIGHT_MODE(TAILTUNE_MODE);
            thrTrqCalib.state = TTC_IDLE;
        }
        return;
    }

    const bool isThrottleHigh = (THROTTLE_HIGH == calculateThrottleStatus(&masterConfig.rxConfig, masterConfig.flight3DConfig.deadband3d_throttle));

    switch(thrTrqCalib.state)
    {
    case TTC_IDLE:
        // Calibration has been requested, only start when throttle is up
        if (isThrottleHigh)
        {
            ENABLE_FLIGHT_MODE(TAILTUNE_MODE);
            beeper(BEEPER_READY_BEEP);
            thrTrqCalib.state = TTC_ACTIVE;
            thrTrqCalib.timestamp_ms = millis();
            thrTrqCalib.lastAdjTime_ms = millis();
            thrTrqCalib.target = TT_CALIB_I_TARGET;
        }
        break;
    case TTC_ACTIVE:
        if (isThrottleHigh &&
            isRcAxisWithinDeadband(ROLL) &&
            isRcAxisWithinDeadband(PITCH) &&
            isRcAxisWithinDeadband(YAW))
        {
            if (IsDelayElapsed_ms(thrTrqCalib.timestamp_ms, 500))
            {
                // RC commands have been within deadbands for 500ms
                if (IsDelayElapsed_ms(thrTrqCalib.lastAdjTime_ms, 500))
                {
                    const int32_t abs_I = ABS(axisPID_I[YAW]);
                    if (abs_I < thrTrqCalib.target)
                    {
                        // I is within limits
                        thrTrqCalib.state = TTC_CHECK_WITHIN_LIMITS;
                        thrTrqCalib.timestamp_ms = millis();
                    }
                    else
                    {
                        // Base increment is 0.1f which is the resolution of the CLI parameter
                        float increment = 0.1f;

                        // If I term is greater than the limit, use greater increment for faster result
                        if (abs_I > TT_CALIB_I_LARGE_INCREMENT_LIMIT)
                        {
                            increment += 0.15f * ((float)abs_I / TT_CALIB_I_LARGE_INCREMENT_LIMIT);
                        }

                        uint8_t beeps = 2;
                        if (axisPID_I[YAW] > 0)
                        {
                            increment *= -1.0f;
                            beeps = 1;
                        }
                        beeperConfirmationBeeps(beeps);

                        float newThrustFactor = tailServoThrustFactor + increment;
                        if ((newThrustFactor < TAIL_THRUST_FACTOR_MIN_FLOAT) || (newThrustFactor > TAIL_THRUST_FACTOR_MAX_FLOAT))
                        {
                            beeper(BEEPER_ACC_CALIBRATION_FAIL);
                            thrTrqCalib.state = TTC_FAIL;
                            thrTrqCalib.timestamp_ms = millis();
                        }
                        else
                        {
                            tailServoThrustFactor = newThrustFactor;
                            initCurves();
                        }
                    }
                    thrTrqCalib.lastAdjTime_ms = millis();
                }
            }
        }
        else
        {
            thrTrqCalib.timestamp_ms = millis();
        }
        break;
    case TTC_CHECK_WITHIN_LIMITS:
        if (isThrottleHigh &&
            (ABS(axisPID_I[YAW]) < thrTrqCalib.target))
        {
            if (IsDelayElapsed_ms(thrTrqCalib.timestamp_ms, 1000))
            {
                beeper(BEEPER_READY_BEEP);
                thrTrqCalib.state = TTC_DONE;
                thrTrqCalib.timestamp_ms = millis();
            }
        }
        else
        {
            // I didn't stay inside limits, back to adjustment
            thrTrqCalib.state = TTC_ACTIVE;
            thrTrqCalib.timestamp_ms = millis();
            if (IsDelayElapsed_ms(thrTrqCalib.lastAdjTime_ms, 200))
            {
                thrTrqCalib.target++;
            }
        }
        break;
    case TTC_DONE:
        gpMixerConfig->tri_tail_motor_thrustfactor = tailServoThrustFactor * 10.0f;

        if (IsDelayElapsed_ms(thrTrqCalib.timestamp_ms, 2000))
        {
            beeper(BEEPER_READY_BEEP);
            thrTrqCalib.timestamp_ms = millis();
        }
        break;
    case TTC_FAIL:
        if (IsDelayElapsed_ms(thrTrqCalib.timestamp_ms, 2000))
        {
            beeper(BEEPER_ACC_CALIBRATION_FAIL);
            thrTrqCalib.timestamp_ms = millis();
        }
        break;
    }
}

