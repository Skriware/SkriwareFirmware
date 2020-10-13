#ifndef SCS
#define SCS
#include "MarlinConfig.h"
#define ADDR        0b1010000
extern uint8_t capdac;

byte write16(uint8_t address, uint8_t reg, uint16_t data);

uint16_t read16(uint8_t address, uint8_t reg);

void cap_sensor_init();

void set_capdac(uint8_t new_cpd);

uint16_t read_capacity_from_Channel(uint8_t channel);

#endif