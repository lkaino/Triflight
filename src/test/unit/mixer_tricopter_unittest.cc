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
  #include "platform.h"
  #include "flight/mixer.h"
  #include "flight/mixer_tricopter.h"
  #include "io/beeper.h"
  #include "io/rc_controls.h"
  #include "config/runtime_config.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

TEST(TriMixerUnittest, WARNING_NO_TEST_DEFINED_WARNING_NO_TEST_DEFINED_WARNING_NO_TEST_DEFINED_WARNING_NO_TEST_DEFINED)
{
}

//STUBS
extern "C" {
  float dT;
  uint32_t millis(void) { return 0; }
  uint8_t armingFlags;
  int16_t rcCommand[4];
  uint16_t getCurrentMinthrottle(void){ return 0; }
  void beeper(beeperMode_e mode){UNUSED(mode);}
  bool isRcAxisWithinDeadband(int32_t axis) { UNUSED(axis); return true;}
  uint32_t rcModeActivationMask;
  uint16_t flightModeFlags = 0;
uint16_t disableFlightMode(flightModeFlags_e mask){ UNUSED(mask); return 0;}
int32_t axisPID_I[3];
void beeperConfirmationBeeps(uint8_t beepCount){UNUSED(beepCount);}
uint16_t enableFlightMode(flightModeFlags_e mask){ UNUSED(mask); return 0; }

typedef struct master_s{
} master_t;

master_t masterConfig;

throttleStatus_e calculateThrottleStatus(rxConfig_t *rxConfig, uint16_t deadband3d_throttle) { UNUSED(rxConfig); UNUSED(deadband3d_throttle); return (throttleStatus_e)0;}
}

