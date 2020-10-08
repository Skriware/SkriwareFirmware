#include "Skriware_capacity_sensor_utilities.h"
#include "Wire.h"

uint8_t capdac = 16;

void write16(uint8_t address, uint8_t reg, uint16_t data)
{
  Wire.beginTransmission(address);
  Wire.write(reg); //send address
  Wire.write( (uint8_t) (data >> 8));
  Wire.write( (uint8_t) data);
  Wire.endTransmission();  
}

uint16_t read16(uint8_t address, uint8_t reg)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();
  uint16_t data;
  Wire.beginTransmission(address);
  Wire.requestFrom(address, (uint8_t)2);
  data = Wire.read();
  data <<= 8;
  data |= Wire.read();
  Wire.endTransmission();
  return data;
}

void cap_sensor_init(){
	Wire.begin();
	write16(ADDR, 0x0C, 0b1000110100000000); // 400 S/s, repeat measurements, reset
  	delay(1000);
  write16(ADDR, 0x08, 0b0001000000000000|((uint16_t)capdac) << 5); // Measurement 1: CHA CIN1; CHB CAPDAC
  write16(ADDR, 0x09, 0b0011000000000000|((uint16_t)capdac) << 5); // Measurement 2: CHA CIN2; CHB CAPDAC 
  write16(ADDR, 0x0A, 0b0101000000000000|((uint16_t)capdac) << 5); // Measurement 3: CHA CIN3; CHB CAPDAC
  write16(ADDR, 0x0B, 0b0111000000000000|((uint16_t)capdac) << 5); // Measurement 4: CHA CIN4; CHB CAPDAC
  write16(ADDR, 0x0C, 0b0000010111110000); // 100 S/s, repeat measurements, start immidietely
} 


void set_capdac(uint8_t new_cpd){
	  write16(ADDR, 0x0C, 0b0000010000000000); // 100 S/s, repeat measurements, stop
      capdac = new_cpd;
      write16(ADDR, 0x08, 0b0001000000000000|((uint16_t)capdac) << 5); // Measurement 1: CHA CIN1; CHB CAPDAC
      write16(ADDR, 0x09, 0b0011000000000000|((uint16_t)capdac) << 5); // Measurement 2: CHA CIN2; CHB CAPDAC 
      write16(ADDR, 0x0A, 0b0101000000000000|((uint16_t)capdac) << 5); // Measurement 3: CHA CIN3; CHB CAPDAC
      write16(ADDR, 0x0B, 0b0111000000000000|((uint16_t)capdac) << 5); // Measurement 4: CHA CIN4; CHB CAPDAC
      write16(ADDR, 0x0C, 0b0000010111110000); // 100 S/s, repeat measurements, start immidietely
}

int32_t read_capacity_from_Channel(uint8_t channel){
  uint16_t msb = read16(ADDR, channel);
  int32_t capacitance = ((int32_t)457) * ((int32_t)msb); //in attofarads
  capacitance /= 1000;   //in femtofarads
  return(capacitance);
}

