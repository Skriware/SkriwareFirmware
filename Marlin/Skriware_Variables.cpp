	#include "Skriware_Variables.h"
	#include "MarlinConfig.h"
	
    bool Matrix_calibration_corruption = false; // bilinear_z_offset and g29- implementation

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
    #ifdef MOVING_EXTRUDER      				
	OneWire  *ds;
	bool servo_extruder = false;
	#endif
    #endif

    #ifdef OPTICAL_SENSOR
    Filament_Sensor *fil_sens;					// Optical Filament sensor
    int C_time = 0.0;
    int NM =0;
    byte tmp;
    long Fil_sens_check_time = 0.0;
    byte fil_alarm_counter = 0;
    byte fil_alarm_counter_error_level = OPTICAL_SENSOR_ERROR_LEVEL;
    float sensor_noise_offset = OPTICAL_SENSOR_NOISE_OFFSET;
    bool optical_sensor_on  = true;
    #endif

     #ifdef EXT_CHEACKSTATION
     	float Z_start = 0.0;
        float Z_up = 0.0;
        float Z_down = 0.0;
     #endif
    float A_calibration_param = 0.0;
    float B_calibration_param = 0.0;
    float C_calibration_param = 0.0;
    float D_calibration_param = 0.0;

	#ifdef SKRIWARE_FILAMENT_RUNOUT_SENSOR		//Mechanical runout sensor
 		bool filament_binary_sensor_E0_on = true;
 		bool filament_binary_sensor_E1_on = true;
 		bool filament_runout_E0 = false;
 		bool filament_runout_E1 = false;
 		long Last_runout_Signal_E0 = 0;
 		long Last_runout_Signal_E1 = 0;

	#endif





