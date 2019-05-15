#include <WioLTEforArduino.h>
#include <BMI088.h>

Bmi088Gyro gyro(Wire, 0x68);

Bmi088Accel accel(Wire, 0x18);
WioLTE Wio;

void setup(void)
{
  int status;

  Wio.Init();
  Wio.PowerSupplyLTE(true);
  Wio.PowerSupplyGrove(true);

  delay(1000);
  SerialUSB.println("BMI088 Raw Data");

  status = accel.begin();
  if (status < 0) {
    Serial.println("Accel Initialization Error");
    Serial.println(status);
    while (1) {}
  }
  status = gyro.begin();
  if (status < 0) {
    Serial.println("Gyro Initialization Error");
    Serial.println(status);
    while (1) {}
  }
}

void loop(void)
{
  /* read the accel */
  accel.readSensor();
  /* read the gyro */
  gyro.readSensor();
  /* print the data */
  Serial.print(accel.getAccelX_mss());
  Serial.print("\t");
  Serial.print(accel.getAccelY_mss());
  Serial.print("\t");
  Serial.print(accel.getAccelZ_mss());
  Serial.print("\t");
  Serial.print(gyro.getGyroX_rads());
  Serial.print("\t");
  Serial.print(gyro.getGyroY_rads());
  Serial.print("\t");
  Serial.print(gyro.getGyroZ_rads());
  Serial.print("\t");
  Serial.print(accel.getTemperature_C());
  Serial.print("\n");
  /* delay to help with printing */
  delay(20);
}
