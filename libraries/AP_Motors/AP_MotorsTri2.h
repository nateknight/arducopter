// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_MotorsTri.h
/// @brief	Motor control class for Tricopters

#ifndef __AP_MOTORS_TRI2_H__
#define __AP_MOTORS_TRI2_H__

#include <AP_Common.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel.h>     // RC Channel Library
#include "AP_Motors.h"

// tail servo uses channel 7
#define AP_MOTORS_CH_TRI2_YAWR    CH_7
#define AP_MOTORS_CH_TRI2_YAWL    CH_8

/// @class      AP_MotorsTri2
class AP_MotorsTri2 : public AP_Motors {
public:

    /// Constructor
    AP_MotorsTri2( RC_Channel* rc_roll, RC_Channel* rc_pitch, RC_Channel* rc_throttle, RC_Channel* rc_yaw, RC_Channel* rc_yawr, RC_Channel* rc_yawl, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_Motors(rc_roll, rc_pitch, rc_throttle, rc_yaw, speed_hz),
        _rc_yawr(rc_yawr), _rc_yawl(rc_yawl) {
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

protected:
    // output - sends commands to the motors
    virtual void        output_armed();
    virtual void        output_disarmed();

    RC_Channel*         _rc_yawr;       // REV parameter used from this channel to determine direction of right yaw servo movement
    RC_Channel*         _rc_yawl;       // REV parameter used from this channel to determine direction of left yaw servo movement
};

#endif  // AP_MOTORSTRI
