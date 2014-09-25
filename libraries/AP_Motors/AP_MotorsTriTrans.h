// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_MotorsTri.h
/// @brief	Motor control class for Tricopters

#ifndef __AP_MOTORS_TRITRANS_H__
#define __AP_MOTORS_TRITRANS_H__

#include <AP_Common.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel.h>     // RC Channel Library
#include "AP_Motors.h"

/// @class      AP_MotorsTri
class AP_MotorsTriTrans : public AP_Motors {
public:

    /// Constructor
    AP_MotorsTriTrans( RC_Channel* rc_roll, RC_Channel* rc_pitch, RC_Channel* rc_throttle, RC_Channel* rc_yaw, RC_Channel* leftservo, RC_Channel* rightservo, RC_Channel* rearservo, RC_Channel* leftelevonservo, RC_Channel* rightelevonservo, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_Motors(rc_roll, rc_pitch, rc_throttle, rc_yaw, speed_hz),
        _leftservo(leftservo),
        _rightservo(rightservo),
        _rearservo(rearservo),
        _leftelevonservo(leftelevonservo),
        _rightelevonservo(rightelevonservo) 
    {
    };

    // init
    virtual void        Init();

    // set update rate to motors - a value in hertz
    void                set_update_rate( uint16_t speed_hz );

    // enable - starts allowing signals to be sent to motors
    virtual void        enable();

    // output_test - spin each motor for a moment to allow the user to confirm the motor order and spin direction
    virtual void        output_test();

    // output_min - sends minimum values out to the motors
    virtual void        output_min();

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // output - sends commands to the motors
    virtual void        output_armed();
    virtual void        output_disarmed();

    AP_Int8             _rev_yaw;          // REV yaw feedback
    AP_Int8             _rev_trans;        // REV rear transition servo
    RC_Channel*         _leftservo;        // left front servo controlling yaw and forward transition
    RC_Channel*         _rightservo;       // right front servo controlling yaw and forward transition
    RC_Channel*         _rearservo;        // rear servo controlling forward transition only.
    RC_Channel*         _leftelevonservo;  // left elevon servo
    RC_Channel*         _rightelevonservo; // right elevno servo
};

#endif  // AP_MOTORSTRI
