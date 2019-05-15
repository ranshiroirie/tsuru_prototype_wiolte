#include <WioLTEforArduino.h>
#include <Wire.h>
#include "BMI088.h"

float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
//int16_t temp = 0;

WioLTE Wio;

void setup()
{
  Wio.Init();
  Wio.PowerSupplyLTE(true);
  Wio.PowerSupplyGrove(true);

  Wire.begin();
  SerialUSB.begin(115200);

  delay(1000);
  SerialUSB.println("BMI088 Raw Data");

  while (1)
  {
    if (bmi088.isConnection())
    {
      bmi088.initialize();
      SerialUSB.println("BMI088 is connected");
      break;
    }
    else SerialUSB.println("BMI088 is not connected");

    delay(2000);
  }
}

void loop()
{
  bmi088.getAcceleration(&ax, &ay, &az);
  bmi088.getGyroscope(&gx, &gy, &gz);
  //  temp = bmi088.getTemperature();

  SerialUSB.print(ax);
  SerialUSB.print(",");
  SerialUSB.print(ay);
  SerialUSB.print(",");
  SerialUSB.print(az);
  SerialUSB.print(",");

  SerialUSB.print(gx);
  SerialUSB.print(",");
  SerialUSB.print(gy);
  SerialUSB.print(",");
  SerialUSB.print(gz);
  SerialUSB.print(",");

  //  Serial.print(temp);

  SerialUSB.println();

  delay(50);
}
