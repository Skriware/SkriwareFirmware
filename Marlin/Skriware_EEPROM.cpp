#include "Marlin.h"
#include "planner.h"
#include "stepper.h"
#include "Skriware_Variables.h"
#include "configuration_store.h"
#include "Skriware_Functions.h"


#define EEPROM_WRITE(VAR) write_data(*eeprom_index, (uint8_t*)&VAR, sizeof(VAR), working_crc)
#define EEPROM_READ(VAR) read_data(*eeprom_index, (uint8_t*)&VAR, sizeof(VAR), working_crc)
 
bool MarlinSettings::older_eeprom_config = false;

void MarlinSettings::save_eeprom_sk2(uint16_t *working_crc,int *eeprom_index){

    //V56
    EEPROM_WRITE(Stepper::Software_Invert);
    EEPROM_WRITE(Stepper::E0_inverted);
    EEPROM_WRITE(A_calibration_param);
    EEPROM_WRITE(B_calibration_param);
    EEPROM_WRITE(C_calibration_param);
    EEPROM_WRITE(D_calibration_param);
    //V57
    #ifdef MOVING_EXTRUDER
    EEPROM_WRITE(home_offset_E1);
    EEPROM_WRITE(home_offset_E0);
    EEPROM_WRITE(extruder_change_time_offset);
    EEPROM_WRITE(servo_up_pos);
    EEPROM_WRITE(servo_down_pos);
    EEPROM_WRITE(up_delay);
    EEPROM_WRITE(extruder_type);
    EEPROM_WRITE(X_up_pos);
    EEPROM_WRITE(X_down_pos);
    EEPROM_WRITE(Y_change);
    EEPROM_WRITE(dY_change);
    EEPROM_WRITE(dX_change);
    //V58
    EEPROM_WRITE(extruder_servo_pin);
    #endif
    #ifdef OPTICAL_SENSOR
    EEPROM_WRITE(sensor_noise_offset);
    #endif


}

void MarlinSettings::load_eeprom_sk2(uint16_t *working_crc,int *eeprom_index, char version[4]){
 int sk_eeprom_verison = (version[2]-'0')+10*(version[1]-'0');  

if(sk_eeprom_verison > 55){
    EEPROM_READ(Stepper::Software_Invert);
    EEPROM_READ(Stepper::E0_inverted);
    if(Stepper::Software_Invert){
        Stepper::E0_inverted = true;
    }else{
        if(checkTestPin(27)){
            Stepper::E0_inverted = true;
        }else{
            Stepper::E0_inverted = false;
        }
    }
    EEPROM_READ(A_calibration_param);
    EEPROM_READ(B_calibration_param);
    EEPROM_READ(C_calibration_param);
    EEPROM_READ(D_calibration_param);               //1.2.0 version
}
if(sk_eeprom_verison > 56){             //For new versions
    #ifdef MOVING_EXTRUDER
    EEPROM_READ(home_offset_E1);
    EEPROM_READ(home_offset_E0);
    EEPROM_READ(extruder_change_time_offset);
    EEPROM_READ(servo_up_pos);
    EEPROM_READ(servo_down_pos);
    EEPROM_READ(up_delay);
    EEPROM_READ(extruder_type);
    EEPROM_READ(X_up_pos);
    EEPROM_READ(X_down_pos);
    EEPROM_READ(Y_change);
    EEPROM_READ(dY_change);
    EEPROM_READ(dX_change);

    #endif
    #ifdef OPTICAL_SENSOR
    EEPROM_READ(sensor_noise_offset);
    #endif
    
}

if(sk_eeprom_verison > 57){
    EEPROM_READ(extruder_servo_pin);
}

}

void MarlinSettings::reset_eeprom_sk2(){
	
    #ifdef MOVING_EXTRUDER
        home_offset_E1 = 0.0;
        home_offset_E0 = 0.0;
        servo_up_pos = SERVO_POS_UP;
        servo_down_pos = SERVO_POS_DOWN;
        extruder_change_time_offset = EXT_CHANGE_TIME_OFFSET;
        up_delay = MOTOR_UP_TIME;
        extruder_type = DEF_EXTRDER_TYPE;
        X_up_pos = 0.0;
        X_down_pos = 0.0;
        Y_change = 0.0;
        dY_change = 0.0;
        dX_change = 0.0;
    #endif

    #ifdef OPTICAL_SENSOR
        sensor_noise_offset = OPTICAL_SENSOR_NOISE_OFFSET;
    #endif
    A_calibration_param = 0.0;
    B_calibration_param = 0.0;
    C_calibration_param = 0.0;
    D_calibration_param = 0.0;
}

 bool MarlinSettings::nan_Matrix_Test(float z_values[GRID_MAX_POINTS_X][GRID_MAX_POINTS_Y]){
    bool nanTest = true;                                      
          for(byte yy = 0 ; yy < GRID_MAX_POINTS_Y; yy++){
            for(byte xx = 0; xx < GRID_MAX_POINTS_X; xx++){
              if(isnan(z_values[xx][yy])){
                nanTest = false;
                break;
              }
            }
          }
          if(!nanTest){
            SERIAL_ECHOLN("ERROR: Calibration data corrupted!");
          }else{
            SERIAL_ECHOLN("Bed matrix loaded, with no errors.");
          }
          return(nanTest);
}