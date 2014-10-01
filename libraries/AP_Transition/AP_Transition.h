/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_TRANSITION_H__
#define __AP_TRANSITION_H__

#include <AP_Param.h>

class AP_Transition 
{
private:
    AP_Float _transition_state;  // State of transition 0 - 1, 0 = hover, 1 = plane.
    AP_Int32 _transition_ms;
    uint32_t _time_of_last_update_ms;
    void set(int32_t percent); // Sets percent of transition

public:
    AP_Transition( AP_Float* transition_state) 
    {
		    AP_Param::setup_object_defaults(this, var_info);
		    transition_state = &_transition_state;
    }

    bool init();
    float read();                // Returns state of transition
    void update( bool hover_mode ); // Iterates the transition counter based on mode.

    static const struct AP_Param::GroupInfo        var_info[];
};

#endif //  __AP_TRANSITION_H__
