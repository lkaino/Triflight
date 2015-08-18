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

#include <stdint.h>
#include <stdbool.h>

#include <limits.h>

extern "C" {
#include "debug.h"
#include "platform.h"

#include "config/runtime_config.h"

#include "common/axis.h"
#include "common/filter.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"

#include "flight/mixer.h"
#define MIXER_TRICOPTER_INTERNALS
#include "flight/mixer_tricopter.h"

#include "io/beeper.h"
#include "io/rc_controls.h"

servoParam_t servoConf;
mixerConfig_t mixerConfig;
tailTune_t tailTune;
int16_t servo[MAX_SUPPORTED_SERVOS];
controlRateConfig_t *currentControlRateProfile;
int16_t motor[MAX_SUPPORTED_MOTORS];

void tailTuneModeThrustTorque(thrustTorque_t *pTT, const bool isThrottleHigh);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

class ThrustFactorCalculationTest: public ::testing::Test {
    // We expect factor = 1 / tan(angle) (but adjusted for formats)
    // Say we want mixerConfig.tri_tail_motor_thrustfactor to be 139, i.e. the factor should be 13.9
    // angle = 1 / atan(factor), according to #25
    // adjust to decidegrees and multiply by servoAvgAngle.numOf
    // i.e. multiply by 3000, then round to integer
    // so even if 12345 looks like an arbitrarily chosen number, it is the result of this calculation and corresponds to 4.115 degrees.
    // after that, add 270000 (90 deg) since the angles actually start at horisontal left
    // Due to possible rounding effects we add a tolerance to the test
protected:
    virtual void SetUp() {
        memset(&servoConf, 0, sizeof(servoConf));
        servoConf.min = DEFAULT_SERVO_MIN;
        servoConf.max = DEFAULT_SERVO_MAX;
        servoConf.middle = DEFAULT_SERVO_MIDDLE;
        servoConf.rate = 100;
        servoConf.forwardFromChannel = CHANNEL_FORWARDING_DISABLED;

        // give all servos a default command
        for (uint8_t i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            servo[i] = DEFAULT_SERVO_MIDDLE;
        }

        mixerConfig.tri_tail_motor_thrustfactor = 123; // so we can check it's unchanged on TT_FAIL
        triInitMixer(&servoConf, &servo[5], &mixerConfig);
        tailTune.mode = TT_MODE_THRUST_TORQUE;
        tailTune.tt.state = TT_WAIT_FOR_DISARM;
        tailTune.tt.servoAvgAngle.numOf = 300;
    }
};

TEST_F(ThrustFactorCalculationTest, 139) {
    // given
    tailTune.tt.servoAvgAngle.sum = 12345 + 270000;
    // and
    tailTuneModeThrustTorque(&tailTune.tt, true);
    // then
    EXPECT_NEAR(139, mixerConfig.tri_tail_motor_thrustfactor, 1);
    EXPECT_EQ(tailTune.tt.state, TT_DONE);
}

TEST_F(ThrustFactorCalculationTest, 145) {
    // given
    tailTune.tt.servoAvgAngle.sum = 11836 + 270000;
    // and
    tailTuneModeThrustTorque(&tailTune.tt, true);
    // then
    EXPECT_NEAR(145, mixerConfig.tri_tail_motor_thrustfactor, 1);
    EXPECT_EQ(tailTune.tt.state, TT_DONE);
}

TEST_F(ThrustFactorCalculationTest, 125) {
    // given
    tailTune.tt.servoAvgAngle.sum = 13722 + 270000;
    // and
    tailTuneModeThrustTorque(&tailTune.tt, true);
    // then
    EXPECT_NEAR(125, mixerConfig.tri_tail_motor_thrustfactor, 1);
    EXPECT_EQ(tailTune.tt.state, TT_DONE);
}

TEST_F(ThrustFactorCalculationTest, 80) {
    // given
    tailTune.tt.servoAvgAngle.sum = 21375 + 270000;
    // and
    tailTuneModeThrustTorque(&tailTune.tt, true);
    // then
    EXPECT_NEAR(80, mixerConfig.tri_tail_motor_thrustfactor, 1);
    EXPECT_EQ(tailTune.tt.state, TT_DONE);
}

TEST_F(ThrustFactorCalculationTest, err90) {
    // given
    tailTune.tt.servoAvgAngle.sum = 270000;
    // and
    tailTuneModeThrustTorque(&tailTune.tt, true);
    // then
    EXPECT_EQ(123, mixerConfig.tri_tail_motor_thrustfactor);
    EXPECT_EQ(tailTune.tt.state, TT_FAIL);
}

TEST_F(ThrustFactorCalculationTest, err130) {
    // given
    tailTune.tt.servoAvgAngle.sum = 390000;
    // and
    tailTuneModeThrustTorque(&tailTune.tt, true);
    // then
    EXPECT_EQ(123, mixerConfig.tri_tail_motor_thrustfactor);
    EXPECT_EQ(tailTune.tt.state, TT_FAIL);
}

//STUBS
extern "C" {

typedef struct master_s {
} master_t;

float dT;
uint8_t armingFlags;
int16_t rcCommand[4];
uint32_t rcModeActivationMask;
uint16_t flightModeFlags = 0;
int16_t debug[DEBUG16_VALUE_COUNT];
gyro_t gyro;
int32_t gyroADC[XYZ_AXIS_COUNT];
master_t masterConfig;

uint32_t millis(void) {
    return 0;
}

uint16_t getCurrentMinthrottle(void) {
    return 0;
}

void beeper(beeperMode_e mode) {
    UNUSED(mode);
}

bool isRcAxisWithinDeadband(int32_t axis) {
    UNUSED(axis);
    return true;
}

uint16_t disableFlightMode(flightModeFlags_e mask) {
    UNUSED(mask);
    return 0;
}

void beeperConfirmationBeeps(uint8_t beepCount) {
    UNUSED(beepCount);
}

uint16_t enableFlightMode(flightModeFlags_e mask) {
    UNUSED(mask);
    return 0;
}

throttleStatus_e calculateThrottleStatus(rxConfig_t *rxConfig,
        uint16_t deadband3d_throttle) {
    UNUSED(rxConfig);
    UNUSED(deadband3d_throttle);
    return (throttleStatus_e) 0;
}

uint16_t adcGetChannel(uint8_t channel) {
    UNUSED(channel);
    return 0;
}

float filterApplyPt1(float input, filterStatePt1_t *filter, uint8_t f_cut,
        float dt) {
    UNUSED(input);
    UNUSED(filter);
    UNUSED(f_cut);
    UNUSED(dt);
    return 0.0;
}

void saveConfigAndNotify(void) {
}

uint16_t getCurrentMaxthrottle(void) {
    return 2000;
}

void pidResetErrorGyroAxis(flight_dynamics_index_t axis) {
    UNUSED(axis);
}

void pidSetExpectedGyroError(flight_dynamics_index_t axis, int16_t error) {
    UNUSED(axis);
    UNUSED(error);
}

}

