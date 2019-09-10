#include "planner.h"
#include "stepper.h"
#include "temperature.h"
#include "ultralcd.h"
#include "language.h"
#include "ubl.h"
#include "gcode.h"

#include "Marlin.h"
//#define DEBUG_E_FADE
/**** For planner.h 

 static float last_z_gcode;
      #ifdef E_FADE        
        static bool use_e_fade;
        static float dz_gcode;
        static float last_e_gcode[EXTRUDERS];
        static float de_real;
        static float de_gcode;
        static float e_real[EXTRUDERS];
        static float last_new_layer_z;
        static float Retracted_filament[EXTRUDERS];
        static float E_fade_extrusion_difference[EXTRUDERS];
        static bool E_fade_applied[EXTRUDERS];
        static int nLayer;
        static bool relative_mode;

      #ifdef START_GCODE_EXTRUSION_CORRECTION
        static float Retraction_from_start_gcode[EXTRUDERS];
      #endif

      #endif

 ****/

#if PLANNER_LEVELING
      float Planner::last_z_gcode = 0.0;
#ifdef E_FADE
      bool  Planner::use_e_fade = false;
      float Planner::dz_gcode = 0.0;
      float Planner::last_e_gcode[] = { 0.0 };
      float Planner::de_real = 0.0;
      float Planner::de_gcode = 0.0;
      float Planner::e_real[] = { 0.0 };
      float Planner::last_new_layer_z = 0.0;
      float Planner::Retracted_filament[] = { 0.0 };
      float Planner::E_fade_extrusion_difference[] = { 0.0 };
      bool  Planner::E_fade_applied[] = { true };
      int   Planner::nLayer = 0; 
      bool  Planner::relative_mode = false;
       #ifdef START_GCODE_EXTRUSION_CORRECTION
      float Planner::Retraction_from_start_gcode[] = {0.0};
      #endif
#endif
#endif

void Planner::efade_and_retract_control_calculation(float &lz, float &e){
	#ifdef E_FADE
	if(use_e_fade){
              de_gcode = e-last_e_gcode[active_extruder];
              if(E_fade_applied[active_extruder] && lz > z_fade_height && de_gcode > 0 && Retracted_filament[active_extruder] == 0.0 &&  e > 0.0){
                E_fade_applied[active_extruder] = false;
                E_fade_extrusion_difference[active_extruder] = e_real[active_extruder] - last_e_gcode[active_extruder];
                SERIAL_ECHOLN("END O E_FADE");
              }
          //if(E_fade_applied[active_extruder]){
          if(de_gcode < 0){
            Retracted_filament[active_extruder] -= de_gcode;    //Retract monitoring
            de_real = de_gcode;
          }else if(de_gcode > 0){
              if(Retracted_filament[active_extruder] > 0.0){        
                      de_real = de_gcode;
                      Retracted_filament[active_extruder] -= de_gcode;
                      if(Retracted_filament[active_extruder] <= 0.0){
                      if(nLayer > 1){
                          de_real -= Retracted_filament[active_extruder];
                          de_real += (1-bilinear_z_offset(tmp)/z_fade_height)*Retracted_filament[active_extruder];
                          if((1-bilinear_z_offset(tmp)/z_fade_height)*dz_gcode > E_FADE_MAX_LAYER_HIGH && dz_gcode < E_FADE_MAX_LAYER_HIGH) de_real += E_FADE_MAX_LAYER_HIGH/dz_gcode*Retracted_filament[active_extruder];
                      }
                        Retracted_filament[active_extruder] = 0.0;
                      }
              }else if(Retracted_filament[active_extruder] == 0.0){
                   if(lz > last_new_layer_z && e > 0.0){
                        dz_gcode = lz - last_new_layer_z;
                        last_new_layer_z = lz;
                        nLayer++;
                       #ifdef DEBUG_E_FADE
                        SERIAL_ECHO("Printing on layer ");
                        SERIAL_ECHOLN(nLayer);
                        SERIAL_ECHO("Z: ");
                        SERIAL_ECHOLN(last_new_layer_z);
                       #endif
                    } 
                    if(nLayer > 1 && e > 0.0){

                      de_real = (1-bilinear_z_offset(tmp)/z_fade_height)*de_gcode;

                      if((1-bilinear_z_offset(tmp)/z_fade_height)*dz_gcode > E_FADE_MAX_LAYER_HIGH && dz_gcode < E_FADE_MAX_LAYER_HIGH)de_real = E_FADE_MAX_LAYER_HIGH/dz_gcode*de_gcode;
                    
                    }else{
                      de_real = de_gcode;
                    }
              }
          }
              if(relative_mode) de_real = de_gcode;
              if(E_fade_applied[active_extruder])e_real[active_extruder] += de_real;
        //}
              last_e_gcode[active_extruder] = e;

  }else if(z_fade_height == 0.0 || E_fade_applied[active_extruder] == false){
              de_gcode = e-last_e_gcode[active_extruder];
          if(de_gcode > 0){
            if(Retracted_filament[active_extruder] > 0.0){
              Retracted_filament[active_extruder] -= de_gcode;
            }else if (Retracted_filament[active_extruder] == 0.0){
              if(lz > last_new_layer_z && e > 0.0){
                dz_gcode = lz - last_new_layer_z;
                last_new_layer_z = lz;
                nLayer++;
                #ifdef DEBUG_E_FADE
                SERIAL_ECHO("Printing on layer ");
                SERIAL_ECHOLN(nLayer);
                SERIAL_ECHO("Z: ");
                SERIAL_ECHOLN(last_new_layer_z);
                #endif
                } 
            }
            if(Retracted_filament[active_extruder] < 0.0)Retracted_filament[active_extruder] = 0.0;
          }else if(de_gcode < 0){
              Retracted_filament[active_extruder] -= de_gcode;
          }
          last_e_gcode[active_extruder] = e;
  }
    #endif
}


void Planner::apply_efade_above_fade_high(float &e){
	 #ifdef E_FADE
          if(use_e_fade){
            #ifdef DEBUG_E_FADE
              SERIAL_ECHO("E_G: ");
              SERIAL_ECHOLN(e);
              SERIAL_ECHO("E_R: ");
            #endif 
            if(use_e_fade){
              if(E_fade_applied[active_extruder]){
                  e = e_real[active_extruder];
              }else{                      
                    e+=E_fade_extrusion_difference[active_extruder];
              }
            }
            #ifdef DEBUG_E_FADE
              SERIAL_ECHOLN(e);
              SERIAL_ECHO("Retraction:");
              SERIAL_ECHOLN(Retracted_filament[active_extruder]);
            #endif
          }
           de_gcode = 0.0;
           de_real = 0.0;
          #endif
}

void Planner::apply_efade_below_fade_high(float &e){
	  #ifdef E_FADE  //ukikoza
      if(use_e_fade && E_fade_applied[active_extruder]){
           
           #ifdef DEBUG_E_FADE
           SERIAL_ECHO("E_G: ");
           SERIAL_ECHOLN(e);
           SERIAL_ECHO("E_R: "); 
           #endif
            e = e_real[active_extruder];
           #ifdef DEBUG_E_FADE
           SERIAL_ECHOLN(e);
           SERIAL_ECHO("Retraction:");
           SERIAL_ECHOLN(Retracted_filament[active_extruder]);
           #endif
      }
      de_gcode = 0.0;
      de_real = 0.0;
      #endif
}