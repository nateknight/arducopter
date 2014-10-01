/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *       APM_Transition.cpp - transitional flight controller
 *
 */
#include <AP_Common.h>
#include <AP_Transition.h>
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_Transition::var_info[] PROGMEM = {

    // @Param: TRANS_MS
    // @DisplayName: Transition Period
    // @Description: Transition time in milliseconds
    // @Increment: 1
    AP_GROUPINFO("TRANS_MS", 2, AP_Transition, _transition_ms, 3000),

    AP_GROUPEND
};

// This implements a linear transition between 0 and 1
void AP_Transition::update( bool hover_mode )
{
    int32_t now_ms = (int32_t)( hal.scheduler->millis() );
    float incr_value;
    
    // Check for uninitialized _time_of_last_update_ms
    if(( _time_of_last_update_ms == 0 ) || (( now_ms - _time_of_last_update_ms ) > _transition_ms ))
    {
      _time_of_last_update_ms = now_ms;
    }
    
    incr_value = ( now_ms - _time_of_last_update_ms ) / (float)_transition_ms;
    
    // If in hover mode, decrement counter so that it will reach 0 _transition_ms after beginning transition 
    if( hover_mode )
    {
      // Decrement toward 0, but constrain at 0.
      if(( _transition_state - incr_value ) < 0 )
      {
        _transition_state = 0;
      }
      else
      {
        _transition_state -= incr_value;
      }
    }
    // If in plane mode, increment counter so that it will reach max _transition_ms after beginning transition
    else
    {
      // Increment toward 1, but constrain at 1.
      if(( _transition_state + incr_value ) > 1 )
      {
        _transition_state = 1;
      }
      else
      {
        _transition_state += incr_value;
      }
    }      

    _time_of_last_update_ms = now_ms;
}

float AP_Transition::read()
{
    return( _transition_state );
}
