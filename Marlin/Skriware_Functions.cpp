#include "Skriware_Functions.h"

#if IS_KINEMATIC
 #define SYNC_PLAN_POSITION_KINEMATIC() sync_plan_position_kinematic()
#else
 #define SYNC_PLAN_POSITION_KINEMATIC() sync_plan_position()
#endif

extern void prepare_move_to_destination();
extern float destination[XYZE];
extern float home_offset[XYZ];
extern Servo servo[NUM_SERVOS];
extern float current_position[XYZE];
extern float feedrate_mm_s;
extern void sync_plan_position_kinematic();
extern void sync_plan_position();
extern void report_current_position();



void set_to_print_Z(){ 
  destination[Z_AXIS] = Planner::last_new_layer_z;
  prepare_move_to_destination();
}

bool checkTestPin(int pin){
  for(int ii = 0; ii < 10000; ii++){
    if(digitalRead(pin) == HIGH){
      return(false);
    }
  }
  return(true);
}
  void invert_E0(){
          Stepper::E0_inverted = !Stepper::E0_inverted;
          stepper.set_directions();
          if(Stepper::E0_inverted){SERIAL_ECHOLN("E1 INVERTED!");
          }else{
            SERIAL_ECHOLN("E1 AS IN CONFIG FILE!");
          }
  }
#ifdef MOVING_EXTRUDER
void Extruder_Up(){
      if(extruder_type == 3){
        servo[0].write(servo_up_pos);
      }else if(extruder_type == 4 && !extruder_up){
        float tmp_f = feedrate_mm_s;
        float tmp_X = current_position[X_AXIS];
        float tmp_Y = current_position[Y_AXIS];
        feedrate_mm_s = 4200;
         destination[Y_AXIS] = Y_change;
         if(active_extruder == 0){
          destination[X_AXIS] = X_up_pos;
         }else{
          destination[X_AXIS] = X_up_pos + hotend_offset[X_AXIS][active_extruder];
         }
         prepare_move_to_destination();
         stepper.synchronize();
         feedrate_mm_s = 1200;
         destination[Y_AXIS] = current_position[Y_AXIS] - dY_change;
         prepare_move_to_destination();
         stepper.synchronize();
         destination[X_AXIS] = current_position[X_AXIS] + dX_change;
         prepare_move_to_destination();
         stepper.synchronize();
         destination[Y_AXIS] = current_position[Y_AXIS] + dY_change;
         prepare_move_to_destination();
         stepper.synchronize();
         feedrate_mm_s = 4200;
         destination[X_AXIS] = tmp_X;
         destination[Y_AXIS] = tmp_Y;
         prepare_move_to_destination();
         stepper.synchronize();
         feedrate_mm_s = tmp_f;
         extruder_up = true;
      }else if(extruder_type != 0 && extruder_type != 4){
        bool done = false;
      while(!done){
      byte addr[8];
      if ( !ds->search(addr)) {
        ds->reset_search();
        SERIAL_ECHOLN("EXTRUDER BOARD CONNECTION FAIL!");
        refresh_cmd_timeout();
        while (PENDING(millis(), extruder_change_time_offset + previous_cmd_ms)) idle();
      }
      if(addr[0] == 0x68){
        ds->reset();
        ds->select(addr);
        ds->write(0xCC);
        ds->reset_search();
        done = true;
      }else{
        SERIAL_ECHOLN("EXTRUDER BOARD CONNECTION FAIL!");
         refresh_cmd_timeout();
        while (PENDING(millis(), extruder_change_time_offset + previous_cmd_ms)) idle();
      }
    }
     while (PENDING(millis(), extruder_change_time_offset + previous_cmd_ms)) idle();
    }
}

void Extruder_Down(){
     if(extruder_type == 3){
      servo[0].write(servo_down_pos);
     }else if(extruder_type == 4 && extruder_up){
         float tmp_f = feedrate_mm_s;
        float tmp_X = current_position[X_AXIS];
        float tmp_Y = current_position[Y_AXIS];
        feedrate_mm_s = 4200;
         destination[Y_AXIS] = Y_change;
         if(active_extruder == 0){
          destination[X_AXIS] = X_down_pos;
         }else{
          destination[X_AXIS] = X_down_pos + hotend_offset[X_AXIS][active_extruder];
         }
         prepare_move_to_destination();
         stepper.synchronize();
         feedrate_mm_s = 1200;
         destination[Y_AXIS] = current_position[Y_AXIS] - dY_change;
         prepare_move_to_destination();
         stepper.synchronize();
         destination[X_AXIS] = current_position[X_AXIS] - dX_change;
         prepare_move_to_destination();
         stepper.synchronize();
         destination[Y_AXIS] = current_position[Y_AXIS] + dY_change;
         prepare_move_to_destination();
         stepper.synchronize();
         feedrate_mm_s = 4200;
         destination[X_AXIS] = tmp_X;
         destination[Y_AXIS] = tmp_Y;
         prepare_move_to_destination();
         stepper.synchronize();
         feedrate_mm_s = tmp_f;
          extruder_up = false;
     }else if(extruder_type != 0 && extruder_type != 4){
       bool done = false;
    while(!done){
     byte addr[8];
      if ( !ds->search(addr)) {
        ds->reset_search();
        SERIAL_ECHOLN("EXTRUDER BOARD CONNECTION FAIL!");
         refresh_cmd_timeout();
        while (PENDING(millis(), extruder_change_time_offset + previous_cmd_ms)) idle();
      }
      if(addr[0] == 0x68){
        ds->reset();
        ds->select(addr);
        ds->write(0xBB);
        ds->reset_search();
        done = true;
      }else{
        SERIAL_ECHOLN("EXTRUDER BOARD CONNECTION FAIL!");
         refresh_cmd_timeout();
        while (PENDING(millis(), extruder_change_time_offset + previous_cmd_ms)) idle();
      }
    }
     while (PENDING(millis(), extruder_change_time_offset + previous_cmd_ms)) idle();
    }
}

void extruder_swap(uint8_t tmp_extruder,uint8_t active){
   bool extruder_change = tmp_extruder != active;        
      float tmp_Z = current_position[Z_AXIS];
   if(extruder_type != 0 && tmp_extruder == 1 && extruder_change){
        stepper.synchronize();
        destination[Z_AXIS] = tmp_Z+2.0;
        prepare_move_to_destination();
        stepper.synchronize();
        Extruder_Down();
        refresh_cmd_timeout();
        while (PENDING(millis(), extruder_change_time_offset + previous_cmd_ms)) idle();
        destination[Z_AXIS] = tmp_Z;
        prepare_move_to_destination();
        stepper.synchronize();  
        current_position[Z_AXIS] += home_offset_E1 - home_offset[Z_AXIS];
        home_offset[Z_AXIS] = home_offset_E1;
        SYNC_PLAN_POSITION_KINEMATIC();
        report_current_position();
    }
    if(extruder_type != 0 && tmp_extruder == 0 && extruder_change){
       stepper.synchronize();
       Extruder_Up();
       refresh_cmd_timeout();
        while (PENDING(millis(), extruder_change_time_offset + previous_cmd_ms)) idle();
       delay(extruder_change_time_offset);
    
       current_position[Z_AXIS] += home_offset_E0 - home_offset[Z_AXIS];
       home_offset[Z_AXIS] = home_offset_E0;

       SYNC_PLAN_POSITION_KINEMATIC();
       report_current_position();
    }
}


void Z_distance_Test(float Z_start,int N_Cycles){   //Test for moving extruder pozition 
  //type - SERVO = 1, ROTOR = 2, MECHANICAL = 3,
  pinMode(2,INPUT);
  pinMode(15,INPUT);
   Extruder_Down(); 
  for(int c = 0; c < N_Cycles; c++){
     
    destination[X_AXIS] = Z_start;
    prepare_move_to_destination();
    stepper.synchronize();
     Extruder_Up();
    float Z_dist = Z_start;
  while(!checkTestPin(15)){        ///going up with the table, till it touches the nozzle
    destination[X_AXIS] = Z_dist;
    prepare_move_to_destination();
    stepper.synchronize();
    Z_dist += 0.001;
  }
  SERIAL_ECHO(c);
  SERIAL_ECHO("1 [mm*1000]: ");
  SERIAL_ECHOLN(Z_dist*1000);   ////Reporting the distance to PC   
     
    destination[X_AXIS] = Z_start;
    prepare_move_to_destination();
    stepper.synchronize();
     Extruder_Down();
     Z_dist = Z_start;
  while(!checkTestPin(2)){        ///going up with the table, till it touches the nozzle
    destination[X_AXIS] = Z_dist;
    prepare_move_to_destination();
    stepper.synchronize();
    Z_dist += 0.001;
  }                               
  SERIAL_ECHO(c);
  SERIAL_ECHO("2 [mm*1000]: ");
  SERIAL_ECHOLN(Z_dist*1000);   ////Reporting the distance to PC
 }
 
}


void Set_Extruder_Type(byte TYPE){
      if(TYPE == 3){
        servo[0].detach();
        servo[0].attach(4);
        servo_extruder = true;
        Extruder_Up();
      }else if(TYPE != 0 && TYPE != 4){
        servo_extruder = false;
      byte addr[8];
      if ( !ds->search(addr)) {
        SERIAL_ECHOLN("EXTRUDER BOARD CONNECTION FAIL!");
        ds->reset_search();
      return;
      }
      if(addr[0] == 0x68){
        ds->reset();
        ds->select(addr);
        ds->write(0xAA);
        ds->write(TYPE);
        ds->reset_search();
      }else{
        SERIAL_ECHOLN("EXTRUDER BOARD CONNECTION FAIL!");
      }
    }
    extruder_type = TYPE;
}

void Set_up_Time(int time){
     if(extruder_type != 0){
      byte addr[8];
      if ( !ds->search(addr)) {
        ds->reset_search();
      return;
      }
      if(addr[0] == 0x68){
        ds->reset();
        ds->select(addr);
        ds->write(0xDC);
        ds->write(uint8_t(time>>8));
        ds->write(uint8_t(time));
        ds->reset_search();
      }else{
        SERIAL_ECHOLN("EXTRUDER BOARD CONNECTION FAIL!");
      }
    }

}
#endif