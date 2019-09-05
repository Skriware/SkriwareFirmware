#ifndef SKRI_V
#define SKRI_V
#include "OneWire.h"
#include "Filament_Sensor.h"

	#if HAS_HOME_OFFSET
	extern float home_offset_E1;    				//MOVING EXTRUDER HOME OFFSET CHANGE
    extern float home_offset_E0;
    extern byte servo_up_pos;
    extern byte servo_down_pos;
    extern int extruder_change_time_offset;
    extern int up_delay;
    extern byte extruder_type;
    extern float X_up_pos;
    extern float X_down_pos;
    extern float Y_change;
    extern float dY_change;
    extern float dX_change;
    extern bool extruder_up;
    #endif
    #ifdef OPTICAL_SENSOR
    extern Filament_Sensor *fil_sens;					//Filament sensor
    	extern int C_time;
    	extern int NM;
    	extern byte tmp;
    	extern long Fil_sens_check_time;
    	extern byte fil_alarm_counter;
    	extern byte fil_alarm_counter_error_level;
    #endif

     #ifdef EXT_CHEACKSTATION
     	extern float Z_start;
        extern float Z_up;
        extern float Z_down;
     #endif

#endif