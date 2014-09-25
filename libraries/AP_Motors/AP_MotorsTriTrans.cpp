// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_MotorsTriTrans.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *       Transitioning Code by Nathan Knight
 *
 */
#include <AP_HAL.h>
#include <AP_Math.h>
#include "AP_MotorsTriTrans.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsTriTrans::var_info[] PROGMEM = {

	// @Param: REV_LF
    // @DisplayName: Reverse left front servo
    // @Description: Ensure the feedback is negative
    // @Values: -1:Opposite direction,1:Same direction
    AP_GROUPINFO("REV_LF", 8, AP_MotorsTriTrans, _rev_lf, 1 ),

	// @Param: REV_RF
    // @DisplayName: Reverse right front servo
    // @Description: Ensure the feedback is negative
    // @Values: -1:Opposite direction,1:Same direction
    AP_GROUPINFO("REV_RF", 8, AP_MotorsTriTrans, _rev_rf, 1 ),

	// @Param: REV_REAR
    // @DisplayName: Reverse rear servo
    // @Description: Ensure the feedback is negative
    // @Values: -1:Opposite direction,1:Same direction
    AP_GROUPINFO("REV_REAR", 8, AP_MotorsTriTrans, _rev_rear, 1 ),

	// @Param: REV_LE
    // @DisplayName: Reverse left elevon
    // @Description: Ensure the feedback is negative
    // @Values: -1:Opposite direction,1:Same direction
    AP_GROUPINFO("REV_LE", 8, AP_MotorsTriTrans, _rev_le, 1 ),

	// @Param: REV_RE
    // @DisplayName: Reverse right elevon
    // @Description: Ensure the feedback is negative
    // @Values: -1:Opposite direction,1:Same direction
    AP_GROUPINFO("REV_RE", 8, AP_MotorsTriTrans, _rev_re, 1 ),

// TODO: things to add = PID values for elevons (or perhaps those should go in the control area).

    AP_GROUPEND
};

// init
void AP_MotorsTriTrans::Init()
{
    // call parent Init function to set-up throttle curve
    AP_Motors::Init();

    // set update rate for the 3 motors (but not the servo on channel 7)
    set_update_rate(_speed_hz);

    // set the motor_enabled flag so that the ESCs can be calibrated like other frame types
    motor_enabled[AP_MOTORS_MOT_1] = true;
    motor_enabled[AP_MOTORS_MOT_2] = true;
    motor_enabled[AP_MOTORS_MOT_3] = true;
}

// set update rate to motors - a value in hertz
void AP_MotorsTriTrans::set_update_rate( uint16_t speed_hz )
{
    // record requested speed
    _speed_hz = speed_hz;

    // set update rate for the 3 motors
    uint32_t mask = 
	    1U << _motor_to_channel_map[AP_MOTORS_MOT_1] |
	    1U << _motor_to_channel_map[AP_MOTORS_MOT_2] |
	    1U << _motor_to_channel_map[AP_MOTORS_MOT_3];
    hal.rcout->set_freq(mask, _speed_hz);
}

// enable - starts allowing signals to be sent to motors and servos
void AP_MotorsTriTrans::enable()
{
    // enable output channels
    hal.rcout->enable_ch(_motor_to_channel_map[AP_MOTORS_MOT_1]);
    hal.rcout->enable_ch(_motor_to_channel_map[AP_MOTORS_MOT_2]);
    hal.rcout->enable_ch(_motor_to_channel_map[AP_MOTORS_MOT_3]);
    hal.rcout->enable_ch(_motor_to_channel_map[AP_MOTORS_MOT_4]);
    hal.rcout->enable_ch(_motor_to_channel_map[AP_MOTORS_MOT_5]);
    hal.rcout->enable_ch(_motor_to_channel_map[AP_MOTORS_MOT_6]);
    hal.rcout->enable_ch(_motor_to_channel_map[AP_MOTORS_MOT_7]);
    hal.rcout->enable_ch(_motor_to_channel_map[AP_MOTORS_MOT_8]);
}

// output_min - sends minimum values out to the motors
void AP_MotorsTriTrans::output_min()
{
    // set lower limit flag
    limit.throttle_lower = true;

    // set all motors to minimum
    motor_out[AP_MOTORS_MOT_1] = _rc_throttle->radio_min;
    motor_out[AP_MOTORS_MOT_2] = _rc_throttle->radio_min;
    motor_out[AP_MOTORS_MOT_3] = _rc_throttle->radio_min;

    // send minimum value to each motor
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_1], _rc_throttle->radio_min);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_2], _rc_throttle->radio_min);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_3], _rc_throttle->radio_min);
    //hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_4], _leftservo->radio_trim);  // do we want to affect the pos of this servo here?
    //hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_5], _rightservo->radio_trim);  // do we want to affect the pos of this servo here?
    //hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_6], _nate_temp_transition_state ); // do we want to affect the pos of this servo here?
    //hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_7], _leftelevonservo->radio_trim ); // do we want to affect the pos of this servo here?
    //hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_8], _rightelevonservo->radio_trim ); // do we want to affect the pos of this servo here?
}

// output_armed - sends commands to the motors
void AP_MotorsTriTrans::output_armed()
{
    int16_t out_min = _rc_throttle->radio_min + _min_throttle;
    int16_t out_max = _rc_throttle->radio_max;

    // initialize lower limit flag
    limit.throttle_lower = false;

    // Throttle is 0 to 1000 only
    _rc_throttle->servo_out = constrain_int16(_rc_throttle->servo_out, 0, _max_throttle);

    // capture desired roll, pitch, yaw and throttle
    _rc_roll->calc_pwm();
    _rc_pitch->calc_pwm();
    _rc_throttle->calc_pwm();
    _rc_yaw->calc_pwm();

    // if we are not sending a throttle output, we cut the motors
    if(_rc_throttle->servo_out == 0) {
        // range check spin_when_armed
        if (_spin_when_armed_ramped < 0) {
            _spin_when_armed_ramped = 0;
        }
        if (_spin_when_armed_ramped > _min_throttle) {
            _spin_when_armed_ramped = _min_throttle;
        }
        motor_out[AP_MOTORS_MOT_1] = _rc_throttle->radio_min + _spin_when_armed_ramped;
        motor_out[AP_MOTORS_MOT_2] = _rc_throttle->radio_min + _spin_when_armed_ramped;
        motor_out[AP_MOTORS_MOT_3] = _rc_throttle->radio_min + _spin_when_armed_ramped;

        // Every thing is limited
        limit.throttle_lower = true;

    }else{
        // Factor in tricopter geometry and transition timing
        float   front_motor_roll_factor  = 0.866f;   // cos(60+90) TODO: This will gradually be reduced and replaced with elevons during transition
        int16_t front_motor_roll_out     = (float)_rc_roll->pwm_out * front_motor_roll_factor; 
        
        float   front_motor_pitch_factor = 0.500f;   // cos(60) TODO: This will gradually be reduced and replaced with elevons during transition
        int16_t front_motor_pitch_out    = (float)_rc_pitch->pwm_out * front_motor_pitch_factor;

        float   front_motor_yaw_factor   = 0.0f;     //  TODO: This will gradually be increased during transition.
        int16_t front_motor_yaw_out      = (float)_rc_yaw->pwm_out * front_motor_yaw_factor;  
        
        float   rear_motor_pitch_factor = -1.0f;   // cos(180) TODO: This will gradually be reduced and replaced with elevons during transition
        int16_t rear_motor_pitch_out    = (float)_rc_pitch->pwm_out * rear_motor_pitch_factor;

        float   front_servo_yaw_factor   = 1.0f;     //  TODO: This will gradually be decreased during transition.
        int16_t front_servo_yaw_servo_out  = (float)_rc_yaw->servo_out * front_servo_yaw_factor;

        float   left_elevon_roll_servo_factor   = 0.5f;     //  TODO: This could gradually be increased from 0 during transition.
        int16_t left_elevon_roll_servo_out  = (float)_rc_roll->servo_out * left_elevon_roll_servo_factor;
        float   left_elevon_pitch_servo_factor   = 0.5f;     //  TODO: This could gradually be increased from 0 during transition.
        int16_t left_elevon_pitch_servo_out  = (float)_rc_pitch->servo_out * left_elevon_pitch_servo_factor;

        float   right_elevon_roll_servo_factor   = 0.5f;     //  TODO: This could gradually be increased from 0 during transition.
        int16_t right_elevon_roll_servo_out  = (float)_rc_roll->servo_out * right_elevon_roll_servo_factor;
        float   right_elevon_pitch_servo_factor   = -0.5f;     //  TODO: This could gradually be increased from 0 during transition.
        int16_t right_elevon_pitch_servo_out  = (float)_rc_pitch->servo_out * right_elevon_pitch_servo_factor;

        int16_t transition_servo_out  = 0;  // TODO: This will gradually rotate all three motor servos during transition.
        
        
        // check if throttle is below limit
        if (_rc_throttle->radio_out <= out_min) {
            limit.throttle_lower = true;
        }
        //left front motor
        motor_out[AP_MOTORS_MOT_2] = _rc_throttle->radio_out + front_motor_roll_out + front_motor_pitch_out - front_motor_yaw_out;
        //right front motor
        motor_out[AP_MOTORS_MOT_1] = _rc_throttle->radio_out - front_motor_roll_out + front_motor_pitch_out + front_motor_yaw_out;
        // rear motor
        motor_out[AP_MOTORS_MOT_3] = _rc_throttle->radio_out + rear_motor_pitch_out;
       
        //left front servo
        _leftservo->servo_out = (-1*_rev_lf*front_servo_yaw_servo_out) + transition_servo_out;
        //right front servo
        _rightservo->servo_out = (_rev_rf*front_servo_yaw_servo_out) + transition_servo_out;
        //rear servo
        _rearservo->servo_out = _rev_rear* ( -4500 + transition_servo_out );

        //left elevon servo
        _leftelevonservo->servo_out = _rev_le*(left_elevon_pitch_servo_out + left_elevon_roll_servo_out );  // This will likely need its own PID values seperate from the motors.
        //right elevon servo
        _rightelevonservo->servo_out = _rev_re*(right_elevon_pitch_servo_out + right_elevon_roll_servo_out );  // This will likely need its own PID values seperate from the motors.

        _leftservo->calc_pwm();
        _rightservo->calc_pwm();
        _rearservo->calc_pwm();
        _leftelevonservo->calc_pwm();
        _rightelevonservo->calc_pwm();

        motor_out[AP_MOTORS_MOT_4] = _leftservo->radio_out;
        motor_out[AP_MOTORS_MOT_5] = _rightservo->radio_out;
        motor_out[AP_MOTORS_MOT_6] = _rearservo->radio_out;
        motor_out[AP_MOTORS_MOT_7] = _leftelevonservo->radio_out;
        motor_out[AP_MOTORS_MOT_8] = _rightelevonservo->radio_out;

        // Tridge's stability patch
        if(motor_out[AP_MOTORS_MOT_1] > out_max) {
            motor_out[AP_MOTORS_MOT_2] -= (motor_out[AP_MOTORS_MOT_1] - out_max);
            motor_out[AP_MOTORS_MOT_3] -= (motor_out[AP_MOTORS_MOT_1] - out_max);
            motor_out[AP_MOTORS_MOT_1] = out_max;
        }

        if(motor_out[AP_MOTORS_MOT_2] > out_max) {
            motor_out[AP_MOTORS_MOT_1] -= (motor_out[AP_MOTORS_MOT_2] - out_max);
            motor_out[AP_MOTORS_MOT_3] -= (motor_out[AP_MOTORS_MOT_2] - out_max);
            motor_out[AP_MOTORS_MOT_2] = out_max;
        }

        if(motor_out[AP_MOTORS_MOT_3] > out_max) {
            motor_out[AP_MOTORS_MOT_1] -= (motor_out[AP_MOTORS_MOT_3] - out_max);
            motor_out[AP_MOTORS_MOT_2] -= (motor_out[AP_MOTORS_MOT_3] - out_max);
            motor_out[AP_MOTORS_MOT_3] = out_max;
        }

        // adjust for throttle curve
        if( _throttle_curve_enabled ) {
            motor_out[AP_MOTORS_MOT_1] = _throttle_curve.get_y(motor_out[AP_MOTORS_MOT_1]);
            motor_out[AP_MOTORS_MOT_2] = _throttle_curve.get_y(motor_out[AP_MOTORS_MOT_2]);
            motor_out[AP_MOTORS_MOT_3] = _throttle_curve.get_y(motor_out[AP_MOTORS_MOT_3]);
        }

        // ensure motors don't drop below a minimum value and stop
        motor_out[AP_MOTORS_MOT_1] = max(motor_out[AP_MOTORS_MOT_1],    out_min);
        motor_out[AP_MOTORS_MOT_2] = max(motor_out[AP_MOTORS_MOT_2],    out_min);
        motor_out[AP_MOTORS_MOT_3] = max(motor_out[AP_MOTORS_MOT_3],    out_min);
    }

    // send output to each motor
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_1], motor_out[AP_MOTORS_MOT_1]);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_2], motor_out[AP_MOTORS_MOT_2]);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_3], motor_out[AP_MOTORS_MOT_3]);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_4], motor_out[AP_MOTORS_MOT_4]);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_5], motor_out[AP_MOTORS_MOT_5]);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_6], motor_out[AP_MOTORS_MOT_6]);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_7], motor_out[AP_MOTORS_MOT_7]);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_8], motor_out[AP_MOTORS_MOT_8]);

}

// output_disarmed - sends commands to the motors
void AP_MotorsTriTrans::output_disarmed()
{
    // fill the motor_out[] array for HIL use
    for (unsigned char i = AP_MOTORS_MOT_1; i < AP_MOTORS_MOT_3; i++) {
        motor_out[i] = _rc_throttle->radio_min;
    }

    // Send minimum values to all motors
    output_min();
}

// output_test - spin each motor for a moment to allow the user to confirm the motor order and spin direction
void AP_MotorsTriTrans::output_test()
{
    // Send minimum values to all motors
    output_min();

    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_2], _rc_throttle->radio_min);
    hal.scheduler->delay(4000);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_1], _rc_throttle->radio_min + _min_throttle);
    hal.scheduler->delay(300);

    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_1], _rc_throttle->radio_min);
    hal.scheduler->delay(2000);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_3], _rc_throttle->radio_min + _min_throttle);
    hal.scheduler->delay(300);

    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_3], _rc_throttle->radio_min);
    hal.scheduler->delay(2000);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_2], _rc_throttle->radio_min + _min_throttle);
    hal.scheduler->delay(300);

    // Send minimum values to all motors
    output_min();
}
