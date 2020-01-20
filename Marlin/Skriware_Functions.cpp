#include "Skriware_Functions.h"
#include "Skriware_Variables.h"

#if IS_KINEMATIC
 #define SYNC_PLAN_POSITION_KINEMATIC() sync_plan_position_kinematic()
#else
 #define SYNC_PLAN_POSITION_KINEMATIC() sync_plan_position()
#endif

extern void prepare_move_to_destination();
extern float destination[XYZE];
extern float home_offset[XYZ];
#ifdef MOVING_EXTRUDER
  extern Servo servo[NUM_SERVOS];
#endif
extern float current_position[XYZE];
extern float feedrate_mm_s;
extern void sync_plan_position_kinematic();
extern void sync_plan_position();
extern void report_current_position();



void set_to_print_Z(){ 
  destination[Z_AXIS] = Planner::last_new_layer_z - home_offset[Z_AXIS];
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
        #ifndef EXT_CHECKSTATION 
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
         planner.synchronize();
         feedrate_mm_s = 1200;
         destination[Y_AXIS] = current_position[Y_AXIS] - dY_change;
         prepare_move_to_destination();
         planner.synchronize();
         destination[X_AXIS] = current_position[X_AXIS] + dX_change;
         prepare_move_to_destination();
         planner.synchronize();
         destination[Y_AXIS] = current_position[Y_AXIS] + dY_change;
         prepare_move_to_destination();
         planner.synchronize();
         feedrate_mm_s = 4200;
         destination[X_AXIS] = tmp_X;
         destination[Y_AXIS] = tmp_Y;
         prepare_move_to_destination();
         planner.synchronize();
         feedrate_mm_s = tmp_f;
         #else
         destination[X_AXIS] = X_up_pos;
         prepare_move_to_destination();
         planner.synchronize();
         #endif
         extruder_up = true;
      }else if(extruder_type != 0 && extruder_type != 4){
        bool done = false;
      while(!done){
      byte addr[8];
      if ( !ds->search(addr)) {
        ds->reset_search();
        SERIAL_ECHOLN("EXTRUDER BOARD CONNECTION FAIL!");
        
        while (PENDING(millis(), extruder_change_time_offset )) idle();
      }
      if(addr[0] == 0x68){
        ds->reset();
        ds->select(addr);
        ds->write(0xCC);
        ds->reset_search();
        done = true;
      }else{
        SERIAL_ECHOLN("EXTRUDER BOARD CONNECTION FAIL!");
         
        while (PENDING(millis(), extruder_change_time_offset )) idle();
      }
    }
     while (PENDING(millis(), extruder_change_time_offset )) idle();
    }
}

void Extruder_Down(){
     if(extruder_type == 3){
      servo[0].write(servo_down_pos);
     }else if(extruder_type == 4 && extruder_up){
      #ifndef EXT_CHECKSTATION
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
         planner.synchronize();
         feedrate_mm_s = 1200;
         destination[Y_AXIS] = current_position[Y_AXIS] - dY_change;
         prepare_move_to_destination();
         planner.synchronize();
         destination[X_AXIS] = current_position[X_AXIS] - dX_change;
         prepare_move_to_destination();
         planner.synchronize();
         destination[Y_AXIS] = current_position[Y_AXIS] + dY_change;
         prepare_move_to_destination();
         planner.synchronize();
         feedrate_mm_s = 4200;
         destination[X_AXIS] = tmp_X;
         destination[Y_AXIS] = tmp_Y;
         prepare_move_to_destination();
         planner.synchronize();
         feedrate_mm_s = tmp_f;
         #else
         destination[X_AXIS] = X_down_pos;
         prepare_move_to_destination();
         planner.synchronize();
         #endif
          extruder_up = false;
     }else if(extruder_type != 0 && extruder_type != 4){
       bool done = false;
    while(!done){
     byte addr[8];
      if ( !ds->search(addr)) {
        ds->reset_search();
        SERIAL_ECHOLN("EXTRUDER BOARD CONNECTION FAIL!");
         
        while (PENDING(millis(), extruder_change_time_offset )) idle();
      }
      if(addr[0] == 0x68){
        ds->reset();
        ds->select(addr);
        ds->write(0xBB);
        ds->reset_search();
        done = true;
      }else{
        SERIAL_ECHOLN("EXTRUDER BOARD CONNECTION FAIL!");
         
        while (PENDING(millis(), extruder_change_time_offset )) idle();
      }
    }
     while (PENDING(millis(), extruder_change_time_offset )) idle();
    }
}

void extruder_swap(uint8_t tmp_extruder,uint8_t active){
   bool extruder_change = tmp_extruder != active;        
      float tmp_Z = current_position[Z_AXIS];
   if(extruder_type != 0 && tmp_extruder == 1 && extruder_change){
        planner.synchronize();
        destination[Z_AXIS] = tmp_Z+3.0;
        prepare_move_to_destination();
        planner.synchronize();
        Extruder_Down();
        while (PENDING(millis(), extruder_change_time_offset )) idle();
        current_position[Z_AXIS] = tmp_Z+3.0 + home_offset_E1 - home_offset[Z_AXIS];
        home_offset[Z_AXIS] = home_offset_E1;
        SYNC_PLAN_POSITION_KINEMATIC();
        report_current_position();
    }
    if(extruder_type != 0 && tmp_extruder == 0 && extruder_change){
       planner.synchronize();
       Extruder_Up();
       while (PENDING(millis(), extruder_change_time_offset )) idle();
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
     
    destination[Z_AXIS] = Z_start;
    prepare_move_to_destination();
    planner.synchronize();
     Extruder_Up();
    float Z_dist = Z_start;
  while(!checkTestPin(15)){        ///going up with the table, till it touches the nozzle
    destination[Z_AXIS] = Z_dist;
    prepare_move_to_destination();
    planner.synchronize();
    Z_dist += 0.005;
  }
  SERIAL_ECHO(c);
  SERIAL_ECHO("1 [mm*1000]: ");
  SERIAL_ECHOLN(Z_dist*1000);   ////Reporting the distance to PC   
     
    destination[Z_AXIS] = Z_start;
    prepare_move_to_destination();
    planner.synchronize();
     Extruder_Down();
     Z_dist = Z_start;
  while(!checkTestPin(15)){        ///going up with the table, till it touches the nozzle
    destination[Z_AXIS] = Z_dist;
    prepare_move_to_destination();
    planner.synchronize();
    Z_dist += 0.005;
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

void optical_sensor_check(){
  #ifdef OPTICAL_SENSOR
       if(optical_sensor_on && millis()-Fil_sens_check_time > 500){
        Fil_sens_check_time = millis();
        fil_sens->readData();
          float r_speed = fil_sens->readSpeed_Y();
         #ifdef SENSOR_DEBUG
          SERIAL_ECHO("SENSOR_DEBUG:");
          SERIAL_ECHO(millis());
          SERIAL_ECHO(":");
          SERIAL_ECHO(Stepper::current_extruder_speed);
          SERIAL_ECHO(":");
          SERIAL_ECHOLN(r_speed);
        #endif
        if(active_extruder == 0 && abs(Stepper::current_extruder_speed) > 0.0001 && abs(r_speed) < sensor_noise_offset){
          fil_alarm_counter++;
          if(fil_alarm_counter == fil_alarm_counter_error_level){
          SERIAL_ECHOLN("FILAMENT_RUNOUT_E0");
          fil_alarm_counter = 0;
          }
        }else{
          fil_alarm_counter = 0;
        }
      }
  #endif
}
void binary_sensor_check(){
  #if ENABLED(SKRIWARE_FILAMENT_RUNOUT_SENSOR)
    if(filament_binary_sensor_E0_on && !filament_runout_E0 && digitalRead(SKRIWARE_FILAMENT_RUNOUT_SENSOR_PIN_E0) == LOW){
      if(millis() - Last_runout_Signal_E0 > BINARY_SENSOR_DEBOUNCE_TIME && Last_runout_Signal_E0 != 0){
          SERIAL_ECHOLN("FILAMENT_RUNOUT_E0");
          filament_runout_E0 = true;
          Last_runout_Signal_E0 = 0;
      }else{
          if(Last_runout_Signal_E0 == 0){
          Last_runout_Signal_E0 = millis();
          }
      }
    }else if(digitalRead(SKRIWARE_FILAMENT_RUNOUT_SENSOR_PIN_E0) == HIGH){
          Last_runout_Signal_E0 = 0;
    }
    if(filament_binary_sensor_E1_on && !filament_runout_E1 && digitalRead(SKRIWARE_FILAMENT_RUNOUT_SENSOR_PIN_E1) == LOW){
      if(millis() - Last_runout_Signal_E1 > BINARY_SENSOR_DEBOUNCE_TIME && Last_runout_Signal_E1 != 0){
          SERIAL_ECHOLN("FILAMENT_RUNOUT_E1");
          filament_runout_E1 = true;
          Last_runout_Signal_E1 = 0;
      }else{
          if(Last_runout_Signal_E1 == 0){
          Last_runout_Signal_E1 = millis();
          }
      }
    }else if(digitalRead(SKRIWARE_FILAMENT_RUNOUT_SENSOR_PIN_E0) == HIGH){
          Last_runout_Signal_E1 = 0;
    
    }
   #endif
}

void g92_efade(bool didE){
      #ifdef E_FADE
      if(didE){                 //ukikoza
      Planner::dz_gcode = 0.0;
      Planner::de_real = 0.0;
      Planner::de_gcode = 0.0;
      for(byte yy = 0; yy < EXTRUDERS; yy++){
      Planner::last_e_gcode[yy] = 0.0;
      Planner::e_real[yy] = 0.0;
      Planner::E_fade_extrusion_difference[yy] = 0.0;
      }  
      }
        #endif
}

void g92_retraction_controll(float *v){
 
/*  if(Planner::Retract_menagement[active_extruder] == 2 && *v < 0.0001){
    SERIAL_ECHOLN("RESET RETRACTION!");
    Planner::Retracted_filament[active_extruder] = 0.0;
  }
*/
  if(Planner::Retract_menagement[active_extruder] == 1 && Planner::Retracted_filament[active_extruder] > 0.00001 && *v == 0.0){       //ukikoza
     #ifdef SKRIWARE_DEBUG
      SERIAL_ECHOLN("RETRACTION CONTROLL!");
     #endif
    *v = -Planner::Retracted_filament[active_extruder];
    Planner::last_e_gcode[active_extruder] = *v;
    Planner::e_real[active_extruder] = *v;
    //Planner::Retract_menagement[active_extruder] = 2;
  }
}

void Skriware_Init(){
   #ifdef MOVING_EXTRUDER
    Set_Extruder_Type(extruder_type);         //ukikoza
    Set_up_Time(up_delay);
    #endif
   
   // put your setup code here, to run once:
    #ifdef OPTICAL_SENSOR
    fil_sens = new Filament_Sensor(15);
    fil_sens->Init();
    fil_sens->set_measurement_time(OPTICAL_SENSOR_MEASUREMENT_TIME);
    fil_sens->set_integration_time(OPTICAL_SENSOR_INT_TIME);
    fil_sens->set_readout_to_mean(OPTICAL_SENSOR_N_TO_MEAN);
    fil_sens->set_resolution(0xFF,0xFF);
    if( fil_sens->upload_config()){
      SERIAL_ECHOLN("SENSOR OK!");
     }else{
      SERIAL_ECHOLN("SENSOR_FAIL!");
     }
     #else
     pinMode(FILAMENT_JAM_SENSOR_PIN_E0,INPUT);
     pinMode(FILAMENT_JAM_SENSOR_PIN_E1,INPUT);
     #endif
     pinMode(27,INPUT_PULLUP);
  if(!Stepper::Software_Invert){
    if(checkTestPin(27)){
        Stepper::E0_inverted = 1;
    }else{
        Stepper::E0_inverted = 0;
    }
  }

  pinMode(11,OUTPUT);
  digitalWrite(11,LOW);
  SERIAL_ECHO("E0 INVERT options:");
  SERIAL_ECHO(stepper.E0_inverted);
  SERIAL_ECHOLN(stepper.Software_Invert);
}

void filament_sensor_check(){
  #ifdef OPTICAL_SENSOR
    optical_sensor_check();
  #else
    binary_sensor_check();
  #endif
}

void zero_bed_levelig_grid(){
for(byte yy = 0 ; yy < GRID_MAX_POINTS_Y; yy++){
    for(byte xx = 0; xx < GRID_MAX_POINTS_X; xx++){
        z_values[xx][yy] = 0.0;      
    }
}
bilinear_grid_spacing[X_AXIS] = (RIGHT_PROBE_BED_POSITION - LEFT_PROBE_BED_POSITION) / (GRID_MAX_POINTS_X - 1);
bilinear_grid_spacing[Y_AXIS] = (BACK_PROBE_BED_POSITION - FRONT_PROBE_BED_POSITION) / (GRID_MAX_POINTS_X - 1);
}

void setZ_Offset_TMC(){
  
  destination[Z_AXIS] = 15.0;
  prepare_move_to_destination();
  planner.synchronize();
  digitalWrite(Z_ENABLE_PIN, LOW);
  stepperZ.coolstep_min_speed(1024UL * 1024UL - 1UL);
  stepperZ.stealthChop(1);
  stepperZ.diag1_stall(1);
  stepperZ.sg_stall_value(SG);
  int TT = 5;
  long st_mean = 0;
  int last_mean = 0;
  long st_mean_sum =0;
  long Mean_of_Means = 0;
  long SqareSum = 0;
  long Standard_dev_of_means = 0;
  int N = 0;
  int NM = 0;
  stepperZ.shaft_dir(1);
  long Start_time = millis();
  long step_taken = 0;
  while(true){
  digitalWrite(Z_STEP_PIN, HIGH);
  delayMicroseconds(400);
  digitalWrite(Z_STEP_PIN, LOW);
  delayMicroseconds(400);
  step_taken++;
  if(step_taken%400 == 0){
    thermalManager.print_heaterstates();
     SERIAL_EOL();
  }
  uint32_t ms = millis();
  if(step_taken > 400){
  static uint32_t last_time = 0;
  if((ms - last_time) >10){
   st_mean_sum +=stepperZ.sg_result();
   N++;
   TT+=1; 
   if(TT >100){
   // SERIAL_ECHOLN(st_mean);
   // SERIAL_ECHOLN("ok");
    TT =0;
   }
  }
  if ((ms - last_time) > 50) {
    last_mean =st_mean;
    st_mean = st_mean_sum/N;
    st_mean_sum = 0;
    N =0;
    /*SERIAL_ECHO("M: ");
    SERIAL_ECHO(Mean_of_Means/NM);
    SERIAL_ECHO(" SD: ");
    SERIAL_ECHO(Standard_dev_of_means);
    SERIAL_ECHO(" R: ");
    SERIAL_ECHOLN(st_mean);
    */
    long MTC = (Mean_of_Means/NM - st_mean);
    if(MTC > 10 && MTC > Standard_dev_of_means +long(std_norm*Standard_dev_of_means) && step_taken > 400*5){
      SERIAL_ECHO("Z hit: ");
      SERIAL_ECHO(step_taken);
      SERIAL_ECHO(" ");
      float Z_off = (float)step_taken/400 - 15.0;
      SERIAL_ECHOLN(Z_off);
      break;
  }
  SqareSum += st_mean*st_mean;
  Mean_of_Means += st_mean;
  NM++;
  Standard_dev_of_means = long(sqrt(SqareSum/NM -Mean_of_Means/NM*Mean_of_Means/NM));
  last_time = millis();
  }
  }
}
  stepperZ.shaft_dir(0);
  for(int step_back = 0; step_back < step_taken;step_back++){
  digitalWrite(Z_STEP_PIN, HIGH);
  delayMicroseconds(400);
  digitalWrite(Z_STEP_PIN, LOW);
  delayMicroseconds(400);
  if(step_back%400 == 0){
    idle();
  }
  }

}

/************* Correction for ABL *************************
void bilinear_line_to_destination(float fr_mm_s){       
     int cx1 = CELL_INDEX(X, current_position[X_AXIS]),
        cy1 = CELL_INDEX(Y, current_position[Y_AXIS]),
        cx2 = CELL_INDEX(X, destination[X_AXIS]),
        cy2 = CELL_INDEX(Y, destination[Y_AXIS]);
        cx1 = constrain(cx1, 0, ABL_BG_POINTS_X - 2);
        cy1 = constrain(cy1, 0, ABL_BG_POINTS_Y - 2);
        cx2 = constrain(cx2, 0, ABL_BG_POINTS_X - 2);
        cy2 = constrain(cy2, 0, ABL_BG_POINTS_Y - 2);

   #define LINE_SEGMENT_END(A) (current_position[A ##_AXIS] + (destination[A ##_AXIS] - current_position[A ##_AXIS]) * normalized_dist)


      if (cx1 == cx2 && cy1 == cy2) {
      // Start and end on same mesh square
        line_to_destination(fr_mm_s);
        set_current_to_destination();
        return;
      }else{
        #ifdef DEBUG_SPLIT
      SERIAL_ECHO("NEW GCODE: ");
      SERIAL_ECHO(" X: ");
      SERIAL_ECHO(destination[X_AXIS]);
      SERIAL_ECHO(" Y: ");
      SERIAL_ECHO(destination[Y_AXIS]);
      SERIAL_ECHO(" Z: ");
      SERIAL_ECHO(destination[Z_AXIS]);
      SERIAL_ECHO(" E: ");
      SERIAL_ECHOLN(destination[E_AXIS]);
      #endif
        int X_grid = cx2-cx1;    
        int Y_grid = cy2-cy1;      //calculate how many gid sections line corsses.

        int X_dir = (cx1 != cx2)? (cx2-cx1)/abs(cx2-cx1) : 0;
        int Y_dir = (cy1 != cy2)? (cy2-cy1)/abs(cy2-cy1) : 0;

        float final_X_destination = destination[X_AXIS];
        float final_Y_destination = destination[Y_AXIS];
        float start_position_X = current_position[X_AXIS];
        float start_position_Y = current_position[Y_AXIS];
        float E_movement = destination[E_AXIS] - current_position[E_AXIS];
        float Z_movement = destination[Z_AXIS] - current_position[Z_AXIS];
        #ifdef DEBUG_SPLIT
        SERIAL_ECHO("Extrusion: ");
        SERIAL_ECHOLN(E_movement);
        #endif
        float normalized_dist_X,normalized_dist_Y,normalized_dist;
        normalized_dist = 0.0;
        float gcode_distance = sqrt((destination[X_AXIS] - current_position[X_AXIS])*(destination[X_AXIS] - current_position[X_AXIS]) + (destination[Y_AXIS] - current_position[Y_AXIS])*(destination[Y_AXIS] - current_position[Y_AXIS]));
        normalized_dist_Y = (final_Y_destination != start_position_Y) ? (final_X_destination - start_position_X)/(final_Y_destination - start_position_Y) : 0;
        normalized_dist_X = (final_X_destination != start_position_X) ? (final_Y_destination - start_position_Y)/(final_X_destination - start_position_X) : 0;
        for(int yy = 0; yy < abs(X_grid)+abs(Y_grid)+1; yy++){ //for loop containing line spliting algorithm

          float destination_X[XYZE],destination_Y[XYZE],distance_to_point[XYZE];
          destination_Y[X_AXIS] = destination_Y[Y_AXIS] = 0.0;
          destination_X[X_AXIS] = destination_X[Y_AXIS] = 0.0;
          distance_to_point[X_AXIS] = 2*Y_MAX_POS;
          distance_to_point[Y_AXIS] = 2*Y_MAX_POS;
         #ifdef DEBUG_SPLIT
          SERIAL_ECHO("CX1: ");
          SERIAL_ECHO(cx1);
          SERIAL_ECHO("CY1: ");
          SERIAL_ECHO(cy1);
        #endif
          if(yy == 0 && X_dir == -1) cx1++; //mange starting edge of the negative direction
          if(yy == 0 && Y_dir == -1) cy1++;
          int gcx = (cx1 != cx2)?cx1 + X_dir: cx1;
          int gcy = (cy1 != cy2)?cy1 + Y_dir: cy1;
         
        // calculating destinations to nearest points with grid crossing:
          if(cx2 != cx1){
            destination_X[X_AXIS] = LOGICAL_X_POSITION(bilinear_start[X_AXIS] + ABL_BG_SPACING(X_AXIS) * gcx);
            destination_X[Y_AXIS] = current_position[Y_AXIS] + normalized_dist_X*(destination_X[X_AXIS]-current_position[X_AXIS]);
            distance_to_point[X_AXIS] = sqrt((destination_X[X_AXIS] - current_position[X_AXIS])*(destination_X[X_AXIS] - current_position[X_AXIS]) + (destination_X[Y_AXIS] - current_position[Y_AXIS])*(destination_X[Y_AXIS] - current_position[Y_AXIS]));
          }
          if(cy2 != cy1){
            destination_Y[Y_AXIS] = LOGICAL_Y_POSITION(bilinear_start[Y_AXIS] + ABL_BG_SPACING(Y_AXIS) * gcy);
            destination_Y[X_AXIS] = current_position[X_AXIS] + normalized_dist_Y*(destination_Y[Y_AXIS] - current_position[Y_AXIS]);
            distance_to_point[Y_AXIS] = sqrt((destination_Y[X_AXIS] - current_position[X_AXIS])*(destination_Y[X_AXIS] - current_position[X_AXIS]) + (destination_Y[Y_AXIS] - current_position[Y_AXIS])*(destination_Y[Y_AXIS] - current_position[Y_AXIS]));
          }
          
          if(cx1 == cx2 && cy1 == cy2){
            destination[X_AXIS] = final_X_destination;
            destination[Y_AXIS] = final_Y_destination;
            normalized_dist = distance_to_point[Y_AXIS] = sqrt((destination[X_AXIS] - current_position[X_AXIS])*(destination[X_AXIS] - current_position[X_AXIS]) + (destination[Y_AXIS] - current_position[Y_AXIS])*(destination[Y_AXIS] - current_position[Y_AXIS]))/gcode_distance;
          }else{
            if(distance_to_point[X_AXIS] >= distance_to_point[Y_AXIS]){
              destination[X_AXIS] = destination_Y[X_AXIS];
              destination[Y_AXIS] = destination_Y[Y_AXIS];
              cy1 += Y_dir;
              if(cy1-cy2 == 1 && Y_dir == -1) cy1--; 
              normalized_dist = distance_to_point[Y_AXIS]/gcode_distance;  //set for Z and E scaling
            }else if(distance_to_point[X_AXIS] < distance_to_point[Y_AXIS]){
              destination[X_AXIS] = destination_X[X_AXIS];
              destination[Y_AXIS] = destination_X[Y_AXIS];
              cx1 += X_dir;
              if(cx1-cx2 == 1 && X_dir == -1) cx1--;
              normalized_dist = distance_to_point[X_AXIS]/gcode_distance;  //set for Z and E scaling
            }

          }
          destination[Z_AXIS] = current_position[Z_AXIS] + Z_movement*normalized_dist;
          destination[E_AXIS] = current_position[E_AXIS] + E_movement*normalized_dist;
          #ifdef DEBUG_SPLIT
          SERIAL_ECHO("SPLIT ");
          SERIAL_ECHO(yy);
          SERIAL_ECHOLN(":  ");
          #endif
          line_to_destination(fr_mm_s);
          set_current_to_destination();
        }


      }

  }
  ******************/
