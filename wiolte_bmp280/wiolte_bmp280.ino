/*
   bmp280_example.ino
   Example sketch for BMP280

   Copyright (c) 2016 seeed technology inc.
   Website    : www.seeedstudio.com
   Author     : Lambor, CHN
   Create Time:
   Change Log :

   The MIT License (MIT)

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
   THE SOFTWARE.
*/
#include <WioLTEforArduino.h>
#include <stdio.h>
#include <Seeed_BMP280.h>
#include <Wire.h>

WioLTE Wio;

BMP280 bmp280;

void setup()
{
  SerialUSB.println("初期化中");
  Wio.Init();

  SerialUSB.println("###電源供給開始");
  Wio.PowerSupplyGrove(true);

  if (!Wio.TurnOnOrReset()) {
    SerialUSB.println("### エラー! ###");
    return;
  }
  
  SerialUSB.begin(9600);
  
  if (!bmp280.init()) {
    SerialUSB.println("Device not connected or broken!");
  }
}

void loop()
{
  float pressure;

  //get and print temperatures
  SerialUSB.print("Temp: ");
  SerialUSB.print(bmp280.getTemperature());
  SerialUSB.println("C"); // The unit for  Celsius because original arduino don't support speical symbols

  //get and print atmospheric pressure data
  SerialUSB.print("Pressure: ");
  SerialUSB.print(pressure = bmp280.getPressure());
  SerialUSB.println("Pa");

  //get and print altitude data
  SerialUSB.print("Altitude: ");
  SerialUSB.print(bmp280.calcAltitude(pressure));
  SerialUSB.println("m");

  SerialUSB.println("\n");//add a line between output of different times.

  delay(1000);
}
