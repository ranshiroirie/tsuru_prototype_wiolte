/*  ********************************************* 
 *  SparkFun_MAG3110_Basic
 *  Triple Axis Magnetometer Breakout - MAG3110 
 *  Hook Up Guide Example 
 *  
 *  Utilizing Sparkfun's MAG3110 Library
 *  A basic sketch that reads x y and z readings
 *  from the MAG3110 sensor
 *  
 *  George B. on behalf of SparkFun Electronics
 *  Created: Sep 22, 2016
 *  Updated: n/a
 *  
 *  Development Environment Specifics:
 *  Arduino 1.6.7
 *  
 *  Hardware Specifications:
 *  SparkFun MAG3110
 *  Bi-directional Logic Level Converter
 *  Arduino Micro
 *  
 *  This code is beerware; if you see me (or any other SparkFun employee) at the
 *  local, and you've found our code helpful, please buy us a round!
 *  Distributed as-is; no warranty is given.
 *  *********************************************/
#include <WioLTEforArduino.h>
#include <stdio.h>
#include <SparkFun_MAG3110.h>

WioLTE Wio;

MAG3110 mag = MAG3110(); //Instantiate MAG3110

void setup() {
  SerialUSB.println("初期化中");
  Wio.Init();

  SerialUSB.println("###電源供給開始");
  Wio.PowerSupplyGrove(true);
  
  SerialUSB.begin(9600);

  mag.initialize(); //Initializes the mag sensor
  mag.start();      //Puts the sensor in active mode
}

void loop() {

  int x, y, z;
  //Only read data when it's ready
  if(mag.dataReady()) {
    //Read the data
    mag.readMag(&x, &y, &z);
  
    SerialUSB.print("X: ");
    SerialUSB.print(x);
    SerialUSB.print(", Y: ");
    SerialUSB.print(y);
    SerialUSB.print(", Z: ");
    SerialUSB.println(z);
  
    SerialUSB.println("--------");
  }
}
