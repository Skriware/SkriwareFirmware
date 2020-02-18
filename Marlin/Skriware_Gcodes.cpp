#include "Skriware_Gcodes.h"
void gcode_M57(){
#ifdef EXT_CHECKSTATION
        if (parser.seen('S')) Z_start = parser.value_float();
        if (parser.seen('N')) NM = parser.value_linear_units();
        Z_distance_Test(Z_start,NM);

#else
         if(parser.seen('E')){
            invert_E0();
         }else{
          set_to_print_Z();
         }
         if(parser.seen('S')) stepper.Software_Invert = 1;
         if(parser.seen('R')) stepper.Software_Invert = 0;
#endif
}
void gcode_M58(){
  #ifdef START_GCODE_EXTRUSION_CORRECTION
        #ifdef SKRIWARE_DEBUG
        SERIAL_ECHOLN("Saving Retracts after start gcode:");
        #endif
        for(byte yy = 0; yy < EXTRUDERS; yy++){
        Planner::Retraction_from_start_gcode[yy] = Planner::Retracted_filament[yy];
        Planner::Retract_menagement[yy] = 1;
        #ifdef SKRIWARE_DEBUG
        SERIAL_ECHO("E");
        SERIAL_ECHO(yy*1);
        SERIAL_ECHO(":");
        SERIAL_ECHOLN(Planner::Retracted_filament[yy]);
        #endif
        }  
        Planner::nLayer = 0;
        Planner::last_new_layer_z = 0.0;
  #endif
}
void gcode_M59(){
  #ifdef E_FADE
          Planner::use_e_fade = false;
          for(byte tt = 0; tt < EXTRUDERS; tt++){
            Planner::Retract_menagement[tt] = 0;
            Planner::Retraction_from_start_gcode[tt] = 0.0;
          }
          #ifdef SKRIWARE_DEBUG
          SERIAL_ECHOLN("E Fade disabled");
          #endif
  #endif
}
void gcode_M60(){
  #ifdef E_FADE
    if(planner.z_fade_height != 0.0){
        Planner::use_e_fade = true;
        #ifdef SKRIWARE_DEBUG
        SERIAL_ECHOLN("E Fade enabled");
        #endif
      }
          Planner::dz_gcode = 0.0;
          Planner::de_real = 0.0;
          Planner::de_gcode = 0.0;
          Planner::last_new_layer_z = 0.0;
      for(byte yy = 0; yy < EXTRUDERS; yy++){
          Planner::last_e_gcode[yy] = 0.0;
          Planner::E_fade_applied[yy] = true;
          Planner::Retracted_filament[yy] = 0.0;
          Planner::E_fade_extrusion_difference[yy] = 0.0;
          Planner::e_real[yy] = 0.0;
      }  
      Planner::nLayer = 0; 
    #endif
}
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
         optical_sensor_on = true;
         SERIAL_ECHOLN("OPTICAL SENSOR ON");
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
#ifdef SKRIWARE_FILAMENT_RUNOUT_SENSOR
      if(filament_binary_sensor_E0_on && filament_binary_sensor_E1_on){
            SERIAL_ECHO("L: ");
            if(digitalRead(SKRIWARE_FILAMENT_RUNOUT_SENSOR_PIN_E0) == LOW){ 
              SERIAL_ECHO("0");
             }else{
              SERIAL_ECHO("1");
             } 
            SERIAL_ECHO(" R: ");
            if(digitalRead(SKRIWARE_FILAMENT_RUNOUT_SENSOR_PIN_E1) == LOW){ 
              SERIAL_ECHOLN("0");
             }else{
              SERIAL_ECHOLN("1");
             } 
        }else{
          SERIAL_ECHO("FILAMENT SENSORS OFF");
        }
#endif
}
void gcode_M65(){
    #ifdef OPTICAL_SENSOR 
      if (parser.seen('N')) fil_sens->set_readout_to_mean(parser.value_linear_units());
      if (parser.seen('I'))  fil_sens->set_integration_time(parser.value_linear_units());
      if (parser.seen('M')) fil_sens->set_measurement_time(parser.value_linear_units());
      if (parser.seen('T')) fil_alarm_counter_error_level = parser.value_linear_units();
      if (parser.seen('O')) sensor_noise_offset = parser.value_linear_units();
    #endif
}
void gcode_M66(){
      if(parser.seen('A'))A_calibration_param = parser.value_float();
      if(parser.seen('B'))B_calibration_param = parser.value_float();
      if(parser.seen('C'))C_calibration_param = parser.value_float();
      if(parser.seen('D'))D_calibration_param = parser.value_float();
      if(parser.seen('R')){
        SERIAL_ECHO("A:");
        SERIAL_ECHO(A_calibration_param);
        SERIAL_ECHO("B:");
        SERIAL_ECHO(B_calibration_param);
        SERIAL_ECHO("C:");
        SERIAL_ECHO(C_calibration_param);
        SERIAL_ECHO("D:");
        SERIAL_ECHO(D_calibration_param);
        SERIAL_ECHOLN(" ");
      }
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
  kill("KILL CAUSE MSG");
}
void gcode_M80(){
    #ifdef MOVING_EXTRUDER
        if (parser.seen('U'))Extruder_Up();
        if (parser.seen('D'))Extruder_Down();
        if (parser.seen('W'))servo_up_pos = parser.value_linear_units();
        if (parser.seen('S'))servo_down_pos = parser.value_linear_units();
        if (parser.seen('O'))extruder_change_time_offset = parser.value_linear_units();

          if (parser.seen('N'))X_up_pos = parser.value_float();
          if (parser.seen('M'))Y_change = parser.value_float();
          if (parser.seen('Y'))dY_change = parser.value_float();
          if (parser.seen('X'))dX_change = parser.value_float();
          if (parser.seen('B'))X_down_pos = parser.value_float();

        if (parser.seen('A')){ 
          up_delay = parser.value_linear_units();
          Set_up_Time(up_delay);
        }
        if (parser.seen('T')){ extruder_type= parser.value_linear_units();
          Set_Extruder_Type(extruder_type);
        }
          if (parser.seen('I')){ 
          Set_Extruder_Type(extruder_type);
          Set_up_Time(up_delay);
        }
        #endif
}