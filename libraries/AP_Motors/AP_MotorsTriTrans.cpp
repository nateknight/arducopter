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
#include <AP_Transition.h>
#include "AP_MotorsTriTrans.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsTriTrans::var_info[] PROGMEM = {

	// @Param: REV_LF
    // @DisplayName: Reverse left front servo
    // @Description: Ensure the feedback is negative
    // @Values: -1:Opposite direction,1:Same direction
    AP_GROUPINFO("REV_LF", 1, AP_MotorsTriTrans, _rev_lf, 1 ),

	// @Param: REV_RF
    // @DisplayName: Reverse right front servo
    // @Description: Ensure the feedback is negative
    // @Values: -1:Opposite direction,1:Same direction
    AP_GROUPINFO("REV_RF", 2, AP_MotorsTriTrans, _rev_rf, 1 ),

	// @Param: REV_REAR
    // @DisplayName: Reverse rear servo
    // @Description: Ensure the feedback is negative
    // @Values: -1:Opposite direction,1:Same direction
    AP_GROUPINFO("REV_REAR", 3, AP_MotorsTriTrans, _rev_rear, 1 ),

	// @Param: REV_LE
    // @DisplayName: Reverse left elevon
    // @Description: Ensure the feedback is negative
    // @Values: -1:Opposite direction,1:Same direction
    AP_GROUPINFO("REV_LE", 4, AP_MotorsTriTrans, _rev_le, 1 ),

	// @Param: REV_RE
    // @DisplayName: Reverse right elevon
    // @Description: Ensure the feedback is negative
    // @Values: -1:Opposite direction,1:Same direction
    AP_GROUPINFO("REV_RE", 5, AP_MotorsTriTrans, _rev_re, 1 ),

	// @Param: HOV_SVO_CEN_LF
    // @DisplayName: LF Servo Center Position when in hover mode
    // @Description: Position around which servo will pivot for yaw control in hover mode.  In Degrees
    // @Values: 0-45
    AP_GROUPINFO("H_CEN_LF", 6, AP_MotorsTriTrans, _hover_servo_center_lf, 3500 ),

	// @Param: HOV_SVO_CEN_RF
    // @DisplayName: RF Servo Center Position when in hover mode
    // @Description: Position around which servo will pivot for yaw control in hover mode.  In Degrees
    // @Values: 0-45
    AP_GROUPINFO("H_CEN_RF", 7, AP_MotorsTriTrans, _hover_servo_center_rf, 3500 ),

	// @Param: HOV_SVO_RANGE
    // @DisplayName: Servo range of motion when in hover mode
    // @Description: Range that servo will pivot for yaw control in hover mode.  In Degrees
    // @Values: 0-45
    AP_GROUPINFO("H_RANGE", 8, AP_MotorsTriTrans, _hover_servo_range, 2000 ),

	// @Param: PLA_SVO_CEN_LF
    // @DisplayName: LF Servo Center Position when in plane mode
    // @Description: Position around which servo will pivot for yaw control in plane mode.  In Degrees
    // @Values: 0-45
    AP_GROUPINFO("P_CEN_LF", 9, AP_MotorsTriTrans, _plane_servo_center_lf, 3500 ),

	// @Param: PLA_SVO_CEN_RF
    // @DisplayName: RF Servo Center Position when in plane mode
    // @Description: Position around which servo will pivot for yaw control in plane mode.  In Degrees
    // @Values: 0-45
    AP_GROUPINFO("P_CEN_RF", 10, AP_MotorsTriTrans, _plane_servo_center_rf, 3500 ),

	// @Param: PLA_SVO_RANGE
    // @DisplayName: Servo range of motion when in plane mode
    // @Description: Range that servo will pivot for yaw control in plane mode.  In Degrees
    // @Values: 0-45
    AP_GROUPINFO("P_RANGE", 11, AP_MotorsTriTrans, _plane_servo_range, 0 ),

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
    // set limits flags
    limit.roll_pitch = true;
    limit.yaw = true;
    limit.throttle_lower = true;
    limit.throttle_upper = false;

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
    int16_t out_min_pwm = _rc_throttle->radio_min + _min_throttle;      // minimum pwm value we can send to the motors
    int16_t out_max_pwm = _rc_throttle->radio_max;                      // maximum pwm value we can send to the motors
    int16_t out_mid_pwm = (out_min_pwm+out_max_pwm)/2;                  // mid pwm value we can send to the motors

    float   front_motor_roll_factor;          // Degree to which the front motor's speed affects roll ( -1 - 1 )
    int16_t front_motor_roll_out;             // Roll input scaled by the front motors' contribution.
    float   front_motor_pitch_factor;         // Degree to which the front motors' speed affects pitch ( -1 - 1 ) 
    int16_t front_motor_pitch_out;            // Pitch input scaled by the front motors' contribution.   
    float   front_motor_yaw_factor;           // Degree to which the front motors' speed affects yaw ( -1 - 1 )
    int16_t front_motor_yaw_out;              // Yaw input scaled by the front motors' contribution.              
    float   rear_motor_roll_factor;           // Degree to which the rear motor speed affects roll ( -1 - 1 )
    int16_t rear_motor_roll_out;              // Roll input scaled by the rear motor's contribution.              
    float   rear_motor_pitch_factor;          // Degree to which the rear motor speed affects pitch ( -1 - 1 )
    int16_t rear_motor_pitch_out;             // Pitch input scaled by the rear motor's contribution.              
    float   front_servo_yaw_factor;           // Degree to which the front servos affect yaw ( -1 - 1 ) 
    int16_t front_servo_yaw_servo_out;        // Yaw input scaled by the front servos' contribution.          
    float   elevon_roll_servo_factor;         // Degree to which the elevons affect roll ( -1 - 1 )
    int16_t elevon_roll_servo_out;            // Roll input scaled by the elevons' contribution.   
    float   elevon_pitch_servo_factor;        // Degree to which the elevons affect pitch ( -1 - 1 )
    int16_t elevon_pitch_servo_out;           // Pitch input scaled by the elevons' contribution.   

    int16_t servo_range, left_servo_center, right_servo_center; // Range and center of front servos, based upon transition state.
    
    // We use the elevon channels to hold our elevon rate controller data
    RC_Channel*         rc_elevon_roll = _leftelevonservo;
    RC_Channel*         rc_elevon_pitch = _rightelevonservo;
    
    // initialize limits flag
    limit.roll_pitch = false;
    limit.yaw = false;
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    // Throttle is 0 to 1000 only
    // To-Do: we should not really be limiting this here because we don't "own" this _rc_throttle object
    if (_rc_throttle->servo_out < 0) {
        _rc_throttle->servo_out = 0;
        limit.throttle_lower = true;
    }
    if (_rc_throttle->servo_out > _max_throttle) {
        _rc_throttle->servo_out = _max_throttle;
        limit.throttle_upper = true;
    }

    // capture desired roll, pitch, yaw and throttle
    _rc_roll->calc_pwm();
    _rc_pitch->calc_pwm();
    _rc_throttle->calc_pwm();
    _rc_yaw->calc_pwm();

    // Factor in tricopter geometry and transition mixing
    front_motor_roll_factor   = 0.866f * ( 1.0f - *_transition_state );  // cos(60+90) for hover, 0 for plane
    front_motor_pitch_factor  = 0.500f * ( 1.0f - *_transition_state );  // cos(60) for hover, 0 for plane
    front_motor_yaw_factor    = 0.500f * ( *_transition_state );       // 0 for hover, half for plane mode
    rear_motor_roll_factor    = 0.0f;                                  // rear motor speed never used to control roll
    rear_motor_pitch_factor   = -1.0f * ( 1.0f - *_transition_state );   // cos(180) for hover, 0 for plane. Can change to account for different motor/prop combo than front motors
    front_servo_yaw_factor    = 0.5f * ( 1.0f - *_transition_state );    // half for hover, 0 for plane
    elevon_roll_servo_factor  = 0.5f * ( *_transition_state );         // 0 for hover, half for plane
    elevon_pitch_servo_factor = 0.5f * ( *_transition_state );         // 0 for hover, half for plane

    // Calculate the pwm contribution of each control entity
    front_motor_roll_out      = (int16_t)( (float)_rc_roll->pwm_out * front_motor_roll_factor );
    front_motor_pitch_out     = (int16_t)( (float)_rc_pitch->pwm_out * front_motor_pitch_factor );
    front_motor_yaw_out       = (int16_t)( (float)_rc_yaw->pwm_out * front_motor_yaw_factor );
    rear_motor_roll_out       = (int16_t)( (float)_rc_roll->pwm_out * rear_motor_roll_factor );
    rear_motor_pitch_out      = (int16_t)( (float)_rc_pitch->pwm_out * rear_motor_pitch_factor );
    front_servo_yaw_servo_out = (int16_t)( (float)_rc_yaw->servo_out * front_servo_yaw_factor );
    //elevon_roll_servo_out     = (int16_t)( (float)_rc_roll->servo_out * elevon_roll_servo_factor );
    //elevon_pitch_servo_out    = (int16_t)( (float)_rc_pitch->servo_out * elevon_pitch_servo_factor );
    elevon_roll_servo_out     = (int16_t)( (float)rc_elevon_roll->servo_out * elevon_roll_servo_factor );
    elevon_pitch_servo_out    = (int16_t)( (float)rc_elevon_pitch->servo_out * elevon_pitch_servo_factor );

    // check if throttle is below limit
    if (_rc_throttle->radio_out <= out_min_pwm) {
        limit.throttle_lower = true;
    }

    // FRONT SERVOS
    // Calculate front servo range and center based on transition state
    servo_range = (int16_t)((( 1.0f - *_transition_state ) * (float)_hover_servo_range ) + (( *_transition_state ) * (float)_plane_servo_range ));
    left_servo_center = (int16_t)((( 1.0f - *_transition_state ) * (float)_hover_servo_center_lf ) + (( *_transition_state ) * (float)_plane_servo_center_lf ));
    right_servo_center = (int16_t)((( 1.0f - *_transition_state ) * (float)_hover_servo_center_rf ) + (( *_transition_state ) * (float)_plane_servo_center_rf ));
   
    //left front servo
    _leftservo->servo_out = (-1*_rev_lf*(( left_servo_center ) + ( front_servo_yaw_servo_out * ( servo_range / 9000 ))));
    //right front servo
    _rightservo->servo_out = (_rev_rf*(( right_servo_center ) + ( front_servo_yaw_servo_out * ( servo_range / 9000 ))));
    //rear servo
    _rearservo->servo_out = _rev_rear* ( -4500 + ( 9000 * ( *_transition_state )));

    //left elevon servo
    _leftelevonservo->servo_out = _rev_le*(elevon_pitch_servo_out + elevon_roll_servo_out );  // This will likely need its own PID values seperate from the motors.
    //right elevon servo
    _rightelevonservo->servo_out = _rev_re*((-1*elevon_pitch_servo_out) + elevon_roll_servo_out );  // This will likely need its own PID values seperate from the motors.

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
        limit.roll_pitch = true;
        limit.yaw = true;
        limit.throttle_lower = true;

    }else{

        //left front motor
        motor_out[AP_MOTORS_MOT_2] = _rc_throttle->radio_out + front_motor_roll_out + front_motor_pitch_out - front_motor_yaw_out;
        //right front motor
        motor_out[AP_MOTORS_MOT_1] = _rc_throttle->radio_out - front_motor_roll_out + front_motor_pitch_out + front_motor_yaw_out;
        // rear motor
        motor_out[AP_MOTORS_MOT_3] = _rc_throttle->radio_out + rear_motor_roll_out + rear_motor_pitch_out;

        // Tridge's stability patch
        if(motor_out[AP_MOTORS_MOT_1] > out_max_pwm) {
            motor_out[AP_MOTORS_MOT_2] -= (motor_out[AP_MOTORS_MOT_1] - out_max_pwm);
            motor_out[AP_MOTORS_MOT_3] -= (motor_out[AP_MOTORS_MOT_1] - out_max_pwm);
            motor_out[AP_MOTORS_MOT_1] = out_max_pwm;
        }

        if(motor_out[AP_MOTORS_MOT_2] > out_max_pwm) {
            motor_out[AP_MOTORS_MOT_1] -= (motor_out[AP_MOTORS_MOT_2] - out_max_pwm);
            motor_out[AP_MOTORS_MOT_3] -= (motor_out[AP_MOTORS_MOT_2] - out_max_pwm);
            motor_out[AP_MOTORS_MOT_2] = out_max_pwm;
        }

        if(motor_out[AP_MOTORS_MOT_3] > out_max_pwm) {
            motor_out[AP_MOTORS_MOT_1] -= (motor_out[AP_MOTORS_MOT_3] - out_max_pwm);
            motor_out[AP_MOTORS_MOT_2] -= (motor_out[AP_MOTORS_MOT_3] - out_max_pwm);
            motor_out[AP_MOTORS_MOT_3] = out_max_pwm;
        }

        // adjust for throttle curve
        if( _throttle_curve_enabled ) {
            motor_out[AP_MOTORS_MOT_1] = _throttle_curve.get_y(motor_out[AP_MOTORS_MOT_1]);
            motor_out[AP_MOTORS_MOT_2] = _throttle_curve.get_y(motor_out[AP_MOTORS_MOT_2]);
            motor_out[AP_MOTORS_MOT_3] = _throttle_curve.get_y(motor_out[AP_MOTORS_MOT_3]);
        }

        // ensure motors don't drop below a minimum value and stop
        motor_out[AP_MOTORS_MOT_1] = max(motor_out[AP_MOTORS_MOT_1],    out_min_pwm);
        motor_out[AP_MOTORS_MOT_2] = max(motor_out[AP_MOTORS_MOT_2],    out_min_pwm);
        motor_out[AP_MOTORS_MOT_3] = max(motor_out[AP_MOTORS_MOT_3],    out_min_pwm);
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
