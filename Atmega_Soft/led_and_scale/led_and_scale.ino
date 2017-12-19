/* Code for Skriware Model 2 Atmega328. 
TO DO:

//Odczyt z obudwu wag.
//POWER 
//LED 
//TESTY 

//CZUJNIK FILA???
 
 *  
 * Atmega currently handles 2 tasks: 
 *  - Weight scale measurement using HX711 ADC
 *  - Controll RGB LED lights: 
 *        1. turn on/off, 
 *        2. fade, 
 *        3. brighten, 
 *        4. breath
 *        5. blink
 *  
 *  MCU is onnected to RPi via I2C interface. 
 *  Communication follows the frame format below:
 *  | 1 byte | 1 byte | 1 byte | 1 byte | 1 byte | 1 byte   |
 *  |--------|--------|--------|--------|--------|----------|
 *  |function|  mode  |    R   |   G    |   B    | checksum |
 *  
 *  Function can be: 
 *  0xFF for Power Off
 *  0xF0 for Cancel Power Off order
 *  0x0F for weight measurement from scale 1
 *  0xCF for weight measurement from scale 2 // measurements are done as an avreage over 5 mesurements in which we exclude the highest and lowest values.
 *  0xCC for lights predefined actions
 *  
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
#include <Adafruit_NeoPixel.h>
#include "HX711.h"

volatile bool breathing = false;
volatile bool flashing = false;
volatile byte Rbreath;
volatile byte Gbreath;
volatile byte Bbreath;

// Specify LED RGB outputs

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

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, LEDPIN, NEO_GRB + NEO_KHZ800);

void setup() {
  pixels.begin();
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent);
  LeftScale.begin(SCALE1_DT, SCALE1_SCK, 64);
  RightScale.begin(SCALE2_DT, SCALE2_SCK, 64);
  //attachInterrupt(digitalPinToInterrupt(PowerButtonInterruptPin),PowerButtonPressed,HIGH);
  
  Serial.begin(9600);           // start serial for output
  pinMode(POWERPin, OUTPUT);
  digitalWrite(POWERPin,HIGH);
  pinMode(SlaveFlagPin,OUTPUT);
  pinMode(PowerButtonInterruptPin,INPUT);

  
  //lights_down();
  
  //showLED();
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

  if(digitalRead(PowerButtonInterruptPin) == HIGH)PowerButtonPressed();
}

void handle_message(byte frame[5]) {
  switch (frame[0]) {
    case 0xAF: 
        attachInterrupt(digitalPinToInterrupt(PowerButtonInterruptPin),PowerButtonPressed,HIGH);
        LeftRead   = measure_weight1();
        detachInterrupt(digitalPinToInterrupt(PowerButtonInterruptPin));
        sprintf(out_buffer,"%ld",LeftRead);
      break; 
    case 0xCF:
         attachInterrupt(digitalPinToInterrupt(PowerButtonInterruptPin),PowerButtonPressed,HIGH);
        RightRead  = measure_weight2();
         detachInterrupt(digitalPinToInterrupt(PowerButtonInterruptPin));
        sprintf(out_buffer,"%ld",RightRead);
      break;
    case 0xCC: 
      control_lights(frame[1], frame[2], frame[3], frame[4]);
      break;
    case 0xFF:
        centerLightDown();
      for(int i = NLED_LEFT ; i > -1; i--){
        delay(350);
         lightsUpToDOWN(2,i,frame[2],frame[3],frame[4]);
         lightsUpToDOWN(0,i,frame[2],frame[3],frame[4]);
      }
      delay(15000);
      digitalWrite(POWERPin,HIGH);
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
      Serial.println("I2C error");
      break;
  }
}

void centerLightUp(byte R, byte G, byte B){
  lightsUpToUP(1,NLED_CENTER,R,G,B);
}

void centerLightDown(){
  lightsUpToUP(1,NLED_CENTER,0,0,0);
}

void control_lights(byte mode, byte R, byte G, byte B) {
  Serial.print("Mode is: "); Serial.println(mode);
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
  //Serial.println("Weight measure");
  long WL = 0;;
  long weightLT[5];
  int i = 0;
  while(true){
    if(LeftScale.is_ready()) {
      weightLT[i] = LeftScale.read_average(5);
      if(digitalRead(PowerButtonInterruptPin) == HIGH)PowerButtonPressed();
      //Serial.print(i);
      //Serial.print(" ");
      //Serial.println(weightLT[i]);
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

  //Serial.print("MAX:");
  //Serial.println(weightLT[maxid]);

  //Serial.print("MIN:");
  //Serial.println(weightLT[minid]);

  for(byte kk = 0; kk <5 ; kk++){
    if(kk != maxid && kk != minid) WL += weightLT[kk];
  }

  //Serial.print("Final:");
  //Serial.println(WL/3);
  return(WL/3);
}

long measure_weight2() {
 //Serial.println("Weight measure");
  long WR = 0;
  long weightRT[5];
  int i = 0;
  while(true){
    if(RightScale.is_ready()) {
      weightRT[i] = RightScale.read_average(5);
      if(digitalRead(PowerButtonInterruptPin) == HIGH)PowerButtonPressed();
      //Serial.print(i);
      //Serial.print(" ");
      //Serial.println(weightRT[i]);
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

  //Serial.print("MAX:");
  //Serial.println(weightRT[maxid]);

  //Serial.print("MIN:");
  //Serial.println(weightRT[minid]);

  for(byte kk = 0; kk <5 ; kk++){
    if(kk != maxid && kk != minid) WR += weightRT[kk];
  }

  //Serial.print("Final:");
  //Serial.println(WR/3);
  return(WR/3);
}



// function that executes whenever data is received from master
void receiveEvent(int howMany) {
  int counter = 0;
  while (Wire.available()) { // loop through all but the last
    frame[counter] = Wire.read();
    Serial.print(counter); Serial.print(" : ");Serial.println(frame[counter]);
    counter++;
    //if(counter > 5)break;
  }
  //if(frame[0]^frame[1]^frame[2]^frame[3]^frame
  handle_message(frame);
}

// function that executes whenever data is requested by master
void requestEvent() {
  int count = Wire.write(out_buffer);
  //Serial.print(" "); Serial.print(count); Serial.println(" bytes sent");
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
  // brighten from 0 to RGB in 5 points steps
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
      pixels.setPixelColor(LEDid+1, pixels.Color(R,G,B)); 
      break;
    case 1:
      pixels.setPixelColor(LEDid+NLED_LEFT+1, pixels.Color(R,G,B)); 
      break;
    case 2:
      pixels.setPixelColor(NUMPIXELS-LEDid-1, pixels.Color(R,G,B)); 
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

void PowerButtonPressed(){
  Serial.println("Interrupt!");
  if((millis() - LastClick) > 500){ 
    LastClick = millis();
    Clicks = 0;
    waiting_for_response = false;
  }else{
    Clicks++;
    delay(10);
    LastClick = millis();
    Serial.println(Clicks);
  }
  if(Clicks > 20){
     if(!MKSPower){
      digitalWrite(POWERPin,LOW);
      lights_down();
      digitalWrite(SlaveFlagPin,HIGH);
      for(int kk = 0; kk < 20 ; kk++){          // to reduce the effect that leds behave strange when getting power.
        delay(100);
        lights_down();
      }
      MKSPower = true;
    }else if(!waiting_for_response){
      digitalWrite(SlaveFlagPin,LOW);
      delay(100);
      digitalWrite(SlaveFlagPin,HIGH);
      waiting_for_response = true;
    }
      //Sendinf to MKS turn off message 
    }
  if(Clicks > 220){
    lights_down();
    MKSPower = false;
    breathing = false;
    flashing = false;
    digitalWrite(SlaveFlagPin,HIGH);
    Clicks = 0;
    digitalWrite(POWERPin,HIGH);
    delay(2500);
    }
}





