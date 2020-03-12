#ifndef SK_F
#define SK_F
#include "Skriware_Variables.h"
#include "stepper.h"
#include "planner.h"
#define SKRIWARE_DEBUG

void set_to_print_Z();
bool checkTestPin(int pin);
void invert_E0();
void Extruder_Up();
void Extruder_Down();
void Z_distance_Test(float Z_start,int N_Cycles);
void Set_Extruder_Type(byte TYPE);
void Set_up_Time(int time);
void extruder_swap(uint8_t tmp_extruder,uint8_t active);
void optical_sensor_check();
void binary_sensor_check();
void filament_sensor_check();
void g92_efade(bool didE);
void g92_retraction_controll(float *v);
void Skriware_Init();
void zero_bed_levelig_grid();

#endif