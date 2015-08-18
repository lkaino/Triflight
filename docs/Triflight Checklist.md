# Setup instructions for Triflight 0.5 Beta3

This guide is intended more as a practical checklist than a fully detailed instruction.
It has some pointers that may help you to find more details, typically you should read the release notes or search the [forum at RCExplorer](http://rcexplorer.se/forums/).

## Propeller Direction

Your racing tricopter will fly better if you have the following motor rotations on it:

```
left	CCW
right	CW
tail	CCW
```

It will help in coordinated turns.
It’s not a big deal if front motors are swapped, with left rotating CW and right CCW.
It will only mean that you will be fighting it a bit more with stick inputs in the turns (with a larger AP tricopter you may want that).
The CCW rotation of the tail motor is absolutely critical for proper triflight behavior.
It will not fly well with the CW tail motor.

If you have it wrong you may use the BLHeliSuite to change.
Otherwise you have to swap any two cables to the motor, which is easiest done before finishing the heat shrink around the ESC.

Chose the right prop for each motor.
The text on the prop shall be up and it shall push air down when rotating.

The props are very angry and they seriously want to hurt you! Only attach the flight battery when you are just about to start flying, and **after** you checked all the switches on the radio.
Attaching the flight battery on the bench is dangerous and you should always remove the props first.

## Installation

Download the triflight_NAZE.hex file from [GitHub](https://github.com/lkaino/Triflight/releases).
Flash it to your flight controller board with the cleanflight-configurator and make sure to tick “full chip erase”.

Make a back-up of the configurator and a CLI dump and store.
It's good to have the stock settings for reference.

## Configuration

### Setup your receiver

Midpoints around 1500, min as close to 1000 from above as possible, max as close to 2000 from below as possible.

For deadband and yaw_deadband, check the servo tab and release the stick to center both slow and abrupt.
Check how much the value differs from 1500.
Add 2-3 and you have your deadband value.
To have the mid's inside the deadband tolerance is critical for the tail tune to work.

### Configurator settings

Make sure you get the receiver connection type right for your receiver.
PWM, PPM or Serial.
Also check the channel map.

Board orientation: for integrated tricopter frame set pitch adjustment to 180.

Failsafe and any other features you would like to use, as well as any mode switches you might desire.

Use a switch for Arm.
You probably also want a switch for Angle/Rate, one for Tail Tune and one for Buzzer.
(Note: this can be done with a 6-channel radio if it has mixers that can mix an aux-channel to itself).

Check so you get Angle/Rate the right way. Rate is the default so if you want Angle at switch position = 0 you may have to think twice.

The Airmode can be on the same switch as Arm.

## ESC bench calibration

Don’t forget to calibrate your ESCs also.
Triflight requires BLHeli 14.0 or higher or one of the latest versions of SimonK firmware on your ESCs to work in Oneshot125 mode.

This calibration should be good until you change a motor or an ESC.

## Accelerometer bench calibration

Calibrate the accelerometer on a flat surface.
You may use a glass or a can, the important thing is to keep the actual board stable at level.

## Tail servo bench tuning

Tricopter requires some extra configuration related to the tail servo.
The most important is servo midpoint and endpoint adjustments.
All the algorithms added for more stable and precise yaw control rely on the correct servo configuration, so follow the steps below with the most accuracy possible.

Make sure your tricopter is level on the roll axis.
If you have calibrated your accelerometer on the level surface already, make sure it shows zero roll.

You will need to connect the flight battery to your tricopter to continue, so make sure the propellers are off!

Without the configurator.
Zero throttle and not touching sticks.
Connect battery.
Do not arm.

- Turn on tail tune mode
- Push cyclic stick left, right or up shortly to select min, max or mid
- Adjust with yaw stick (mid must be very accurate)
- Turn off tail tune mode
- Save values with stick command (both max down and out)
- Remove battery

If you want to cancel before finishing this process you just dont save before removing the battery (but if you start the hardware servo feedback calibration the values will be saved).
If you use the hardware servo feedback there is one more step.
See "Extra for hardware servo feedback" below.

Refer to [this video](https://www.youtube.com/watch?v=AwS4IqAXTBk) for more on usage.

Your stock mini tricopter should be good to go now!

## Tail servo bench tuning - Alternative method

Despite all recommendations you can use the tail prop on, but be careful and do not arm.
Be sure you have tested that arm/disarm is working first.

Adjust as above.
Find the midpoint where the prop tips have the same distance to the bench, when pointing straight left-right.

## Accelerometer hover calibration

Double check your COG (center of gravity) first.
It shall be on the centerline, one third from the front props to the aft prop.

On a calm day, hover in angle mode at least eyeheight.
Look for sideways drift and then land.

When disarmed throttle max up, cyclic max to the side opposite to where it drifts.
The board will be blinking and make a short beep (if you have a Buzzer) for every 0.1 degree of adjustment.  

Adjust, arm, fly, disarm, repeat till it doesn’t start drifting.

0.1 is not much; it can be good to trim by 5-10 ticks each adjustment to start with, and then go back by half amount when it drifts the other way.

Land and disarm.
Save values with stick command (both max down and out) before turning off.

## Tail servo hover tuning

To activate in-flight tail tuning, arm the copter before switching tail tune on.
Tail tune can be activated in air and it will sound a Buzzer pattern.
After activating the mode user has 5 seconds to take copter into hover.

You need a very calm day.
Let it hover and do not touch the sticks for about 30s.
It's ok to use the sticks to adjust the hover position but for that time the tuning is paused.

When the tuning measurements are done there will be a series of very short beeps.
After the tune it will sound the Buzzer pattern again.

Land and disarm, still having tail tune on.
The ready beeping should continue every 2 seconds.
The values are now automatically saved, if you disarm first, wait a few seconds and then deactivate the tail tune. **Do not** use stick command.

Use the configurator CLI and check *tri_tail_motor_thrustfactor*.
If it's still 138 (default) it's likely the tuning failed.

## PID Settings and more

The best PID settings depend on your configuration.
The default settings are for the MiniTri with the new EMACS 2300 motor.

Recommended setting for a MiniTri with the DYS 1800 motor is:
```
set p_pitch = 43
set i_pitch = 30
set d_pitch = 26
set p_roll = 49
set i_roll = 30
set d_roll = 48
set p_yaw = 170
set i_yaw = 45
set d_yaw = 45

set roll_rate = 75
set pitch_rate = 75
set yaw_rate = 75

set tri_motor_acc_yaw_correction = 16

set gyro_soft_lpf =  90.000
```

However, you may want to adjust the rate and expo settings if you want more stability for relaxed LOS flying.
A beginner could for example try to:
Increase *p_level* and *i_level* by 30-50% to have more self stabilisation.
Reduce *roll_rate* *pitch_rate* and *yaw_rate* to 30.
Reduce *rc_expo* to 50.
Reduce *max_angle_inclination* to 300.

When you are more comfortable you change the values gradually to the stock settings.

**Do not** paste full CLI setups from any other firmware or Triflight release.

## Backup you settings

Make a new back-up of the configurator and a CLI dump and store with a new filename.
It's good to have your favorite settings for reference.
It is also good to use a file comparison tool so you can compare with the stock settings and identify your changes easy.

## Extra for custom servo

If you’re using some other servo or powering your servo with something other than 5V, you might need to adjust the *tri_tail_servo_speed* CLI variable to match the speed of your servo.
It is the angular speed of the servo in degrees per second under load.
Default value is 300 and it’s close enough for the BlueBird BMS-210DMH servo (sold by David), powered by something close to 5V.

## Extra for hardware servo feedback

To improve the yaw control even more you can modify the servo and add a feedback wire.
David has explained this servo mod [here](http://rcexplorer.se/forums/topic/debugging-the-tricopter-mini-racer/page/45/#post-24499).

Servo feedback signal must be max 3.3V!
Use voltage divider if your signal is greater than that.

The calibration will not automatically select the source, the user must define it in the CLI before using it.
Before connecting the servo feedback cable, define in CLI the source pin for the feedback:
```
set tri_servo_feedback = VIRTUAL | RSSI | CURRENT | EXT1
```

Feedback signal is calibrated by pulling down on pitch stick while doing the Tail servo bench tuning.
Min, mid and max positions must be set before this (see "Tail servo bench tuning" above).
If you start this, all bench tuning values will be saved automatically (there is no way to cancel).

This also sets the *tri_tail_servo_speed*.
Check your servo speed from CLI!
If servo speed is set, the FW will use the calibrated feedback signal from this point on.

To fall back to using virtual servo, perform CLI ```set tri_servo_feedback = VIRTUAL``` and initiate the feedback calibration again.
The servo will go to min position and then mid position.

Failure beep pattern can be heard from buzzer during calibration.
If your servo does this while feedback cable is connected, there is something wrong in your feedback signal.
