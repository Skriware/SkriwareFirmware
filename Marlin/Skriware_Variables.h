#ifndef SKRI_V
#define SKRI_V
#include "MarlinConfig.h"
#include "OneWire.h"
#include "Filament_Sensor.h"
#include "servo.h"
    
    extern bool Matrix_calibration_corruption;
    extern float std_norm;
    extern byte SG;
    extern byte TMC_checkTime;
    #ifdef MOVING_EXTRUDER
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
            extern OneWire  *ds;
        	extern bool servo_extruder;
            extern byte tmp;
    #endif
    #endif
    #ifdef OPTICAL_SENSOR
            extern Filament_Sensor *fil_sens;					//Filament sensor
        	extern int C_time;
        	extern long Fil_sens_check_time;
        	extern byte fil_alarm_counter;
        	extern byte fil_alarm_counter_error_level;
            extern bool optical_sensor_on;
            extern float sensor_noise_offset;
    #endif

    #ifdef EXT_CHECKSTATION
         	extern float Z_start;
            extern float Z_up;
            extern float Z_down;
            extern int NM;
    #endif

            extern float A_calibration_param;
            extern float B_calibration_param;
            extern float C_calibration_param;
            extern float D_calibration_param;

#ifdef SKRIWARE_FILAMENT_RUNOUT_SENSOR
        	extern bool filament_binary_sensor_E0_on;
        	extern bool filament_binary_sensor_E1_on;
        	extern bool filament_runout_E0;
        	extern bool filament_runout_E1;
        	extern long Last_runout_Signal_E0;
        	extern long Last_runout_Signal_E1;

#endif

#endif