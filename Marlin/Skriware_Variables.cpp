	#include "Skriware_Variables.h"
	#include "MarlinConfig.h"
	#if HAS_HOME_OFFSET
	 float home_offset_E1 = 0.0;    				//MOVING EXTRUDER HOME OFFSET CHANGE
     float home_offset_E0 = 0.0;
     byte servo_up_pos = SERVO_POS_UP;
     byte servo_down_pos = SERVO_POS_DOWN;
     int extruder_change_time_offset = EXT_CHANGE_TIME_OFFSET;
     int up_delay = MOTOR_UP_TIME;
     byte extruder_type = 0;
     float X_up_pos  =  0.0;
     float X_down_pos  =  0.0;
     float Y_change  =  0.0;
     float dY_change =  0.0;
     float dX_change =  0.0;
     bool extruder_up = true;
    #endif

    #ifdef OPTICAL_SENSOR
    Filament_Sensor *fil_sens;					//Filament sensor
    int C_time = 0.0;
    int NM =0;
    byte tmp;
    long Fil_sens_check_time = 0.0;
    byte fil_alarm_counter = 0;
    byte fil_alarm_counter_error_level = 4;
    #endif

     #ifdef EXT_CHEACKSTATION
     	float Z_start = 0.0;
        float Z_up = 0.0;
        float Z_down = 0.0;
     #endif