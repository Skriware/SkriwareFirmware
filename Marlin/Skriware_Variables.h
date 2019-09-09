#ifndef SKRI_V
#define SKRI_V
#include "OneWire.h"
#include "Marlin.h"
#include "Arduino.h"
#include "ultralcd.h"
#include "planner.h"
#include "stepper.h"
#include "endstops.h"
#include "temperature.h"
#include "cardreader.h"
#include "configuration_store.h"
#include "language.h"
#include "pins_arduino.h"
#include "math.h"
#include "nozzle.h"
#include "duration_t.h"
#include "types.h"
#include "gcode.h"
#include "Skriware_Variables.h"
#include "Skriware_Functions.h"
#include "Filament_Sensor.h"

#if HAS_SERVOS
  #include "servo.h"
#endif


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
    #endif
    #ifdef OPTICAL_SENSOR
    extern Filament_Sensor *fil_sens;					//Filament sensor
    	extern int C_time;
    	extern int NM;
    	extern byte tmp;
    	extern long Fil_sens_check_time;
    	extern byte fil_alarm_counter;
    	extern byte fil_alarm_counter_error_level;
        extern bool optical_sensor_on;
    #endif

     #ifdef EXT_CHEACKSTATION
     	extern float Z_start;
        extern float Z_up;
        extern float Z_down;
     #endif

#ifdef SKRIWARE_FILAMENT_RUNOUT_SENSOR
	extern bool filament_binary_sensor_E0_on;
	extern bool filament_binary_sensor_E1_on;
	extern bool filament_runout_E0;
	extern bool filament_runout_E1;
	extern long Last_runout_Signal_E0;
	extern long Last_runout_Signal_E1;

#endif

#endif