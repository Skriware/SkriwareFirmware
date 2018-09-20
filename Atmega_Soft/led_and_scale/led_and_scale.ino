/* Code for Skriware Model 2 Atmega328.  
 *  
 * Atmega currently handles 3 tasks: 
 *  - Weight scale measurement using HX711 ADC
 *  - Controll RGB LED lights: 
 *        1. turn on/off, 
 *        2. fade, 
 *        3. brighten, 
 *        4. breath
 *        5. blink
 *  - Controle power for main Skriware Model 2 printer CPU. 
 *  
 *  MCU is onnected to RPi via I2C interface. 
 *  Communication follows the frame format below:
 *  | 1 byte | 1 byte | 1 byte | 1 byte | 1 byte | 1 byte   |
 *  |--------|--------|--------|--------|--------|----------|
 *  |function|  mode  |    R   |   G    |   B    | checksum |
 *  
 *  Function can be: 
 *  0xDD for Power Off
 *  0xF0 for Cancel Power Off order
 *  0x0F for weight measurement from scale 1
 *  0xCF for weight measurement from scale 2 // measurements are done as an avreage over 5 mesurements in which we exclude the highest and lowest values.
 *  0xCC for lights predefined actions
 *  0xDF for software version request
 *  0x0C for left LED panel direct controll 
 *  0xC0 for right LED panel direct controll 
 *  0xA0 for left LED panel configuration
 *  0x0A for right LED panel configuration
 *  0xAA for confirming configuration from 0xA0 and 0x0A commands and sending them to LED panels
 *  0x0B for predefined LED light up to RGB on both pads from 0 to NLEDS
 *  0xB0 for predefined LED light up to RGB on both pads from N to NLEDS  
 *  0x0D for predefined LED light up to RGB on Left pad from N to NLEDS 
 *  0xD0 for predefined LED light up to RGB on Left pad from N to NLEDS 
 *  0x0E for predefined LED light up to RGB on Right pad from N to NLEDS 
 *  0xE0 for predefined LED light up to RGB on Right pad from N to NLEDS 
 *  0xC1 for center LED bar light up to RGB
 *  0x1C for center LED bar light down
 *  When controlling lights with 0xCC one must specify mode and RGB colour.
 *  Available modes:
 *    - 0x00 turn the lights on
 *    - 0x01 turn the lgihts off
 *    - 0x02 brighten
 *    - 0x03 fade
 *    - 0x04 breath
 *    - 0x05 flash 
 *    
 *  When directryl controling left or right LED panel (0xC0 or 0x0C): 
 *  mode  -> number of the LED in chain
 *  R,G,B -> Color of the LED 
 *  
 *  When configuring  left or right LED panel (0xA0 or 0x0A): 
 *  mode  -> number of the LED in chain
 *  R,G,B -> Color of the LED 
 *  Configuration will be shown on both panels after sending acceptation command(0xAA)
 *  
 *  When using LED up to(0xB0 or 0x0B):
 *  mode  -> number of the LED in chain from which(0x0B) or to which(0xB0) leds will be set to RGB
 *  R,G,B -> Color of the LEDs 
 *  
 *  After weight measurement has been requested master
 *  can read the weight using I2C read mode. Result is sent
 *  as an array of chars. 
 *  
 */
#include <Wire.h>
#include "Adafruit_NeoPixel.h"
#include "HX711.h"

volatile bool breathing = false;
volatile bool flashing = false;
volatile byte Rbreath;
volatile byte Gbreath;
volatile byte Bbreath;

//#define DEBUG -//debug mode for Serial communication.
// Specify LED RGB outputs
#define   SOFT_VERSION  1
#define   SlaveFlagPin  A3
#define   SCALE1_DT     9
#define   SCALE2_DT     10
#define   SCALE1_SCK    13
#define   SCALE2_SCK    13
#define   POWERPin  8
#define   PowerButtonInterruptPin  2
#define   LEDPIN 6
#define   NUMPIXELS 39
#define   NLED_LEFT 12
#define   NLED_CENTER 15
#define   NLED_RIGHT 12

bool MKSPower = false;
long int LastClick = 0;
bool waiting_for_response = false;
int Clicks = 0;
long LeftRead;   
long RightRead;  

HX711 LeftScale;
HX711 RightScale;
char out_buffer[32];
byte frame[6];

bool softwareVersionRequest = false;
int softwareVersion = SOFT_VERSION;
uint8_t softwareVersionCode[16];

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, LEDPIN, NEO_GRB + NEO_KHZ800);

void setSoftwareVersionCode(){
softwareVersionCode[0] = softwareVersion;
 for(int yy = 1 ; yy < 16 ; yy++){
  uint8_t tmp = (uint8_t)(((softwareVersion+yy)*(softwareVersion+yy)) % 255);
  softwareVersionCode[yy] = tmp;
 }


}
void setup() {
 
  pixels.begin();
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent);
  LeftScale.begin(SCALE1_DT, SCALE1_SCK, 64);
  RightScale.begin(SCALE2_DT, SCALE2_SCK, 64);
  
  #ifdef DEBUG
  Serial.begin(9600);           // start serial for output
  #endif
  pinMode(POWERPin, OUTPUT);
  digitalWrite(POWERPin,HIGH);
  pinMode(SlaveFlagPin,OUTPUT);
  digitalWrite(SlaveFlagPin,LOW);
  pinMode(PowerButtonInterruptPin,INPUT);
  setSoftwareVersionCode();
  
}

void loop() {
  
  if(breathing) {
    attachInterrupt(digitalPinToInterrupt(PowerButtonInterruptPin),PowerButtonPressed,HIGH);
    breath();
    detachInterrupt(digitalPinToInterrupt(PowerButtonInterruptPin));
  }

  if(flashing){
    attachInterrupt(digitalPinToInterrupt(PowerButtonInterruptPin),PowerButtonPressed,HIGH);
    flash();
    detachInterrupt(digitalPinToInterrupt(PowerButtonInterruptPin));
  }

  if(MKSPower){
  LeftRead   = measure_weight1();
  if(digitalRead(PowerButtonInterruptPin) == HIGH)PowerButtonPressed();
  RightRead  = measure_weight2();
  }
  if(digitalRead(PowerButtonInterruptPin) == HIGH)PowerButtonPressed();
}

void handle_message(byte frame[5]) {
  byte checksum = 0;
  for(int kk = 0; kk < 5; kk++) checksum = frame[kk] ^ checksum;
  checksum += 4;
  if(checksum  == frame[5]){
  switch (frame[0]) {
    case 0xDF: 
        softwareVersionRequest = true;
      break; 
    case 0xAF: 
        sprintf(out_buffer,"%ld",LeftRead);
      break; 
    case 0xCF:
        sprintf(out_buffer,"%ld",RightRead);
      break;
    case 0xCC: 
      control_lights(frame[1], frame[2], frame[3], frame[4]);
      break;
    case 0xDD:
        centerLightDown();
      for(int i = NLED_LEFT ; i > -1; i--){
        delay(1500);
         lightsUpToDOWN(2,i,frame[2],frame[3],frame[4]);
         lightsUpToDOWN(0,i,frame[2],frame[3],frame[4]);
      }
      digitalWrite(POWERPin,HIGH);
      digitalWrite(SlaveFlagPin,LOW);
      MKSPower = false;
      flashing = false;
      breathing = false;
      break;
    case 0xF0:
      digitalWrite(SlaveFlagPin,HIGH);
      break;
    case 0x0C:
      setLED(0,frame[1], frame[2], frame[3], frame[4]);
      showLED();
      break;
    case 0xC0:
      setLED(2,frame[1], frame[2], frame[3], frame[4]);
      showLED();
      break;
    case 0x0A:
      setLED(2,frame[1], frame[2], frame[3], frame[4]);
      break;
    case 0xA0:
      setLED(0,frame[1], frame[2], frame[3], frame[4]);
      break;
    case 0xAA:
      showLED();
      break;
    case 0xB0:
      lightsUpToUP(2,frame[1], frame[2], frame[3], frame[4]);
      lightsUpToUP(0,frame[1], frame[2], frame[3], frame[4]);
      break;
    case 0x0B:
      lightsUpToDOWN(2,frame[1], frame[2], frame[3], frame[4]);
      lightsUpToDOWN(0,frame[1], frame[2], frame[3], frame[4]);
      break;
    case 0xD0:
      lightsUpToUP(2,frame[1], frame[2], frame[3], frame[4]);
      break;
    case 0x0D:
      lightsUpToDOWN(2,frame[1], frame[2], frame[3], frame[4]);
      break;
    case 0xE0:
      lightsUpToUP(0,frame[1], frame[2], frame[3], frame[4]);
      break;
    case 0x0E:
      lightsUpToDOWN(0,frame[1], frame[2], frame[3], frame[4]);
      break;
    case 0xC1:
      centerLightUp(frame[2], frame[3], frame[4]);
      break;
    case 0x1C:
      centerLightDown();
      break;
    default: 
      #ifdef DEGUB
      Serial.println("I2C error");
      #endif
      break;
  }

}
}

void centerLightUp(byte R, byte G, byte B){
  lightsUpToUP(1,NLED_CENTER,R,G,B);
}

void centerLightDown(){
  lightsUpToUP(1,NLED_CENTER,0,0,0);
}

void control_lights(byte mode, byte R, byte G, byte B) {
  #ifdef DEBUG
  Serial.print("Mode is: "); Serial.println(mode);
  #endif
  breathing = 0;
  flashing = 0;
  switch (mode) {
    case 0x00:
      lights_up(R, G, B);
      break;
    case 0x01:
      lights_down();
      break;
    case 0x02:
      brighten(R, G, B);
      break;
    case 0x03:
      fade(R, G, B);
      break;
    case 0x04:
      Rbreath = R;
      Gbreath = G;
      Bbreath = B;
      breathing = !breathing;
      break;
    case 0x05:
      Rbreath = R;
      Gbreath = G;
      Bbreath = B;
      flashing = !flashing;
      break;
    case 0x06:
        for(int i = 0 ; i <NLED_LEFT+1; i++){
        delay(650);
         lightsUpToDOWN(2,i,R,G,B);
         lightsUpToDOWN(0,i,R,G,B);
      }
      centerLightUp(R,G,B);
    break;
  }
}

long measure_weight1() {
  long WL = 0;
  long weightLT[5];
  int i = 0;
  while(true){
    if(digitalRead(PowerButtonInterruptPin) == HIGH)PowerButtonPressed();
    if(LeftScale.is_ready()) {
      weightLT[i] = LeftScale.read_average(2);
      i++;
    }
    if(i == 5)break;
  }
  
  long lmax = weightLT[0];
  long lmin = weightLT[0];
  int maxid = 0;
  int minid = 0;

  for(int jj = 1; jj < 5; jj++){
   if(weightLT[jj] > lmax){
    lmax = weightLT[jj];
    maxid = jj;
   }
   if(weightLT[jj] < lmin){
    lmin = weightLT[jj];
    minid = jj;
   }
  }


  for(byte kk = 0; kk <5 ; kk++){
    if(kk != maxid && kk != minid) WL += weightLT[kk];
  }

  return(WL/3);
}

long measure_weight2() {
  long WR = 0;
  long weightRT[5];
  int i = 0;
  while(true){
    if(digitalRead(PowerButtonInterruptPin) == HIGH)PowerButtonPressed();
    if(RightScale.is_ready()) {
      weightRT[i] = RightScale.read_average(2);
      i++;
    }
    if(i == 5)break;
  }
  
  long lmax = weightRT[0];
  long lmin = weightRT[0];
  int maxid = 0;
  int minid = 0;

  for(int jj = 1; jj < 5; jj++){
   if(weightRT[jj] > lmax){
    lmax = weightRT[jj];
    maxid = jj;
   }
   if(weightRT[jj] < lmin){
    lmin = weightRT[jj];
    minid = jj;
   }
  }

  for(byte kk = 0; kk <5 ; kk++){
    if(kk != maxid && kk != minid) WR += weightRT[kk];
  }
  

  return(WR/3);
}

// function that executes whenever data is received from master
void receiveEvent(int howMany) {
  int counter = 0;
  while (Wire.available()) { // loop through all but the last
    frame[counter] = Wire.read();
    #ifdef DEBUG
    Serial.print(counter); Serial.print(" : ");Serial.println(frame[counter]);
    #endif
    counter++;
  }
  handle_message(frame);
}

// function that executes whenever data is requested by master
void requestEvent() {
  if(softwareVersionRequest){
    for(int yy = 0 ; yy < 16 ; yy++)Wire.write(softwareVersionCode[yy]);
    softwareVersionRequest = false;
  }else{
    Wire.write(out_buffer);
  }
}

void fade(byte R, byte G, byte B) {
  
  byte Rstep = R/20 + 1;
  byte Gstep = G/20 + 1;
  byte Bstep = B/20 + 1;

  if(R == 0) Rstep = 0;
  if(G == 0) Gstep = 0;
  if(B == 0) Bstep = 0;

  for (int f = 20; f > -1; f--) {
    for(int i = 0 ; i < NUMPIXELS; i++){
      pixels.setPixelColor(i, pixels.Color(f*Rstep,
                                                  f*Gstep,
                                                  f*Bstep));     
    }
  delay(200);
  pixels.show();

  }
  
}

void brighten(byte R, byte G, byte B) {
  // brighten from 0 to RGB in 20 points steps
  byte Rstep = R/20 + 1;
  byte Gstep = G/20 + 1;
  byte Bstep = B/20 + 1;

  if(R == 0) Rstep = 0;
  if(G == 0) Gstep = 0;
  if(B == 0) Bstep = 0;

  for (int f = 0; f <= 20; f++) {
    for(int i = 0 ; i < NUMPIXELS; i++){
      pixels.setPixelColor(i, pixels.Color(f*Rstep,
                                           f*Gstep,
                                           f*Bstep)); 
           
    }
  delay(200);
  pixels.show();
    }

}

void breath() {
  brighten(Rbreath, Gbreath, Bbreath);
  fade(Rbreath, Gbreath, Bbreath);
}

void flash() {
    lights_up(Rbreath, Gbreath, Bbreath);
    delay(300);
    lights_down();
    delay(300);
}

void lights_up(byte R, byte G, byte B) {
    for(int i = 0 ; i < NUMPIXELS; i++){
      pixels.setPixelColor(i, pixels.Color(R,G,B));                                              
    }
    pixels.show(); 
}

void lights_down() {
   for(int i = 0 ; i < NUMPIXELS; i++){
      pixels.setPixelColor(i, pixels.Color(0,0,0));                                              
    }

    pixels.show();
}

void setLED(byte LEDbar,byte LEDid, byte R, byte G,byte B){
  switch (LEDbar){
    case 0:
      pixels.setPixelColor(LEDid, pixels.Color(R,G,B)); 
      break;
    case 1:
      pixels.setPixelColor(LEDid+NLED_LEFT-1, pixels.Color(R,G,B)); 
      break;
    case 2:
      pixels.setPixelColor(NUMPIXELS-LEDid, pixels.Color(R,G,B)); 
      break;
  }
}

void showLED(){
  pixels.show();
}

void lightsUpToDOWN(byte LEDbar, byte n, byte R, byte G, byte B){
    switch (LEDbar){
    case 0:
      for(int i = 0 ; i < NLED_LEFT; i++){
            if( i < n){ 
              pixels.setPixelColor(i, pixels.Color(R,G,B));                                              
            }else{
              pixels.setPixelColor(i, pixels.Color(0,0,0)); 
             }
      }
      pixels.show();
      break;
    case 1:
      for(int i = NLED_LEFT ; i < NLED_LEFT+NLED_CENTER; i++){
            if( i < n+NLED_LEFT){ 
              pixels.setPixelColor(i, pixels.Color(R,G,B));                                              
            }else{
              pixels.setPixelColor(i, pixels.Color(0,0,0)); 
             }
      }
      pixels.show();
      break;
    case 2:
       for(int i = NUMPIXELS-1 ; i > NLED_CENTER+NLED_LEFT-1; i--){
            if( i > NUMPIXELS-n-1){ 
              pixels.setPixelColor(i, pixels.Color(R,G,B));                                              
            }else{
              pixels.setPixelColor(i, pixels.Color(0,0,0)); 
             }
      }
      pixels.show();
      break;
  }
}

void lightsUpToUP(byte LEDbar,byte n, byte R, byte G, byte B){
   switch (LEDbar){
    case 0:
      for(int i = NLED_LEFT+1 ; i > -1 ; i--){
            if( i > NLED_LEFT-n-1){ 
              pixels.setPixelColor(i, pixels.Color(R,G,B));                                              
            }else{
              pixels.setPixelColor(i, pixels.Color(0,0,0)); 
             }
      }
      pixels.show();
      break;
    case 1:
      for(int i = NLED_LEFT+NLED_CENTER-1; i > NLED_LEFT-1; i--){
            if( i > NLED_LEFT+NLED_CENTER-n-1){ 
              pixels.setPixelColor(i, pixels.Color(R,G,B));                                              
            }else{
              pixels.setPixelColor(i, pixels.Color(0,0,0)); 
             }
      }
      pixels.show();
      break;
    case 2:
       for(int i =  NLED_LEFT+NLED_CENTER ; i < NUMPIXELS; i++){
            if( i < NLED_LEFT+NLED_CENTER+n-1){ 
              pixels.setPixelColor(i, pixels.Color(R,G,B));                                              
            }else{
              pixels.setPixelColor(i, pixels.Color(0,0,0)); 
             }
      }
      pixels.show();
      break;
  
}

}

void HelloWorld(){
  lights_up(255,255,255);
  for(int i = NLED_LEFT ; i > -1; i--){
        delay(100);
         lightsUpToUP(2,i,255,255,255);
         lightsUpToUP(0,i,255,255,255);
      }
  for(int jj = 1; jj <8; jj++){
        delay(100);
        setLED(1,jj,0,0,0);
        setLED(1,NLED_CENTER-jj+1,0,0,0);
        showLED();
  }

  for(int kk = 1; kk < 8; kk++){
    delay(100);
    lights_down();
    setLED(1,8+kk,255,255,255);
    setLED(1,8-kk,255,255,255);
    showLED();
  }

  for(int jj = NLED_LEFT; jj > 0; jj--){
    delay(100);
    lights_down();
    setLED(0,jj-1,255,255,255);
    setLED(2,jj,255,255,255);
    showLED();
  }
}




void PowerButtonPressed(){
  if((millis() - LastClick) > 500){ 
    LastClick = millis();
    Clicks = 0;
    waiting_for_response = false;
  }else{
    Clicks++;
    delay(10);
    LastClick = millis();
  }
  if(Clicks > 20){
     if(!MKSPower){
      digitalWrite(POWERPin,LOW);
      digitalWrite(SlaveFlagPin,HIGH);
      HelloWorld();
      MKSPower = true;
    }else if(!waiting_for_response){
      digitalWrite(SlaveFlagPin,LOW);
      delay(100);
      digitalWrite(SlaveFlagPin,HIGH);
      waiting_for_response = true;
    }
    }
  if(Clicks > 220){
    lights_down();
    MKSPower = false;
    breathing = false;
    flashing = false;
    Clicks = 0;
    digitalWrite(POWERPin,HIGH);
    digitalWrite(SlaveFlagPin,LOW);
    delay(2500);
    }
}





