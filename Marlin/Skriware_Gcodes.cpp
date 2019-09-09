#include "Skriware_Gcodes.h"

void gcode_M61(){
		#ifdef SKRIWARE_FILAMENT_RUNOUT_SENSOR	
		if (parser.seen('L')) filament_binary_sensor_E0_on = true;
        if (parser.seen('R')) filament_binary_sensor_E1_on = true;
        if (parser.seen('A')){
          filament_binary_sensor_E0_on = true;
          filament_binary_sensor_E1_on = true;
        }
        #endif
        #ifdef OPTICAL_SENSOR
         optical_sensor_on = false;
         SERIAL_ECHOLN("OPTICAL SENSOR OFF");
        #endif
}

void gcode_M62(){
		#ifdef SKRIWARE_FILAMENT_RUNOUT_SENSOR	
		if (parser.seen('L')) filament_binary_sensor_E0_on = false;
        if (parser.seen('R')) filament_binary_sensor_E1_on = false;
        if (parser.seen('A')){
          filament_binary_sensor_E0_on = false;
          filament_binary_sensor_E1_on = false;
        }
         #endif
         #ifdef OPTICAL_SENSOR
         optical_sensor_on = false;
         SERIAL_ECHOLN("OPTICAL SENSOR OFF");
        #endif
}
void gcode_M63(){
	#ifdef SKRIWARE_FILAMENT_RUNOUT_SENSOR
	  	if (parser.seen('L')) filament_runout_E0 = false; 
        if (parser.seen('R')) filament_runout_E1 = false;
        if (parser.seen('A')){
          filament_runout_E1 = false;
          filament_runout_E0 = false;
        }
    #endif
    #ifdef OPTICAL_SENSOR
        fil_alarm_counter = 0;
    #endif
}


void gcode_M64(){

}
void gcode_M65(){

}
void gcode_M66(){

}
void gcode_M67(){

}
void gcode_M68(){

}
void gcode_M69(){

}
void gcode_M70(){

}
void gcode_M71(){

}
void gcode_M72(){

}
void gcode_M73(){

}
void gcode_M74(){

}
void gcode_M75(){

}
void gcode_M76(){

}
void gcode_M77(){

}
void gcode_M78(){

}
void gcode_M79(){

}
void gcode_M80(){

}