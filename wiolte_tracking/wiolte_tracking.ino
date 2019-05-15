/*
  ＜取得情報＞
  ・IMEI（Wio LTE本体）
  ・取得日時（GPSセンサー）
  ・緯度＆経度（GPSセンサー）
  ・速度（GPSセンサー）
  ・進路方向（GPSセンサー）
  ・温度＆湿度（温度湿度センサー）
  ・加速度（６軸加速度ジャイロセンサー）
  ・角加速度（６軸加速度ジャイロセンサー）
  ・気圧（気圧センサー）（予定）
  ・地磁気（磁力センサー）（予定）

  ＜使用部品・センサー＞
  ・Wio LTE
  ・SORACOM Air SIM（予定）
  ・温度湿度センサー(DHT11)
  ・GPSセンサー(E-1612-UB)
  ・６軸加速度ジャイロセンサー(BMI011)
  ・気圧センサー(BMP280)（予定）
  ・三軸デジタル磁力センサ(MAG3110)（予定）

  ＜デバイス側からAPI Gatewayへのデータ送信形式（JSON）＞
  {
  "TrackingSensor":
    {
      "DeviceID":"IMEI",
      "SensorContents":
        {
          "Timestamp":"YYYY-MM-DDTHH:MM:SS",
          "GPS":
            {
              "latitude_mdeg":緯度,
              "longitude_mdeg":経度,
              "speed":速度,
              "course":進路方向
            },
          "Temp":温度,
          "Humid":湿度,
          "Acceleration":
            {
              "ax":X加速度,
              "ay":Y加速度,
              "az":Z加速度
            },
          "Pressure":気圧,
          "Magnetism":地磁気,
          "Gyro":
            {
              "gx":X角加速度,
              "gy":Y角加速度,
              "gz":Z角加速度
            }
        }
    }
  }

  ＜API GatewayのURL＞
  https://mrmpxsc84b.execute-api.ap-northeast-1.amazonaws.com/dev/test
*/

#include <WioLTEforArduino.h>
#include <SparkFun_MAG3110.h>
#include <MicroNMEA.h>
#include <stdio.h>
#include <BMI088.h>
#include <Seeed_BMP280.h>
#include <Wire.h>

#define APN               "iijmio.jp"
#define USERNAME          "mio@iij"
#define PASSWORD          "iij"

#define BASEURL       "https://mrmpxsc84b.execute-api.ap-northeast-1.amazonaws.com/dev/test"

#define SENSOR_PIN    (WIOLTE_D38)

WioLTE Wio;

//磁力センサーの設定
MAG3110 mag = MAG3110(); //Instantiate MAG3110

//気圧センサーの設定
BMP280 bmp280;

//加速度センサーの設定
float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;

//GPSの設定
HardwareSerial& gps = Serial;
char nmeaBuffer[85];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));
bool ledState = LOW;

void led(bool state)
{
  if (state)  Wio.LedSetRGB(0, 255, 0);
  else       Wio.LedSetRGB(255, 0, 0);
}

void gpsHardwareReset()
{
  while (gps.available())
    gps.read();

  Wio.PowerSupplyGrove(false);
  delay(500);
  Wio.PowerSupplyGrove(true);

  while (1) {
    while (gps.available()) {
      char c = gps.read();
      if (nmea.process(c))  return;

    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////
//
char IMEI[20]; //SIMの電話番号を一時保存するための配列

void setup()
{
  Wio.Init();
  Wio.PowerSupplyLTE(true);
  Wio.PowerSupplyGrove(true);

  SerialUSB.println("###電源投入");
  if (!Wio.TurnOnOrReset()) {
    SerialUSB.println("### エラー! ###");
    return;
  }

  //GPSの設定
  gps.begin(9600);

  led(ledState);

  SerialUSB.println("GPSリセット中 ...");
  gpsHardwareReset();
  SerialUSB.println("... 完了");

  MicroNMEA::sendSentence(gps, "$PORZB");
  MicroNMEA::sendSentence(gps, "$PORZB,RMC,1,GGA,1");
  MicroNMEA::sendSentence(gps, "$PNVGNME,2,9,1");

  //温度湿度センサーの設定
  TemperatureAndHumidityBegin(SENSOR_PIN);

  //加速度ジャイロセンサーの設定
  Wire.begin();
  delay(3000);
  while (1)
  {
    if (bmi088.isConnection())
    {
      bmi088.initialize();
      SerialUSB.println("BMI088接続完了");
      break;
    }
    else SerialUSB.println("BMI088接続されていません");

    delay(2000);
  }

  //磁力センサーの設定
  mag.initialize(); //Initializes the mag sensor
  mag.start();      //Puts the sensor in active mode

  //気圧センサーの設定
  if (!bmp280.init()) {
    SerialUSB.println("Device not connected or broken!");
  }

  //電話番号の取得
  SerialUSB.println(Wio.GetIMEI(IMEI, sizeof(IMEI)));
  SerialUSB.println(IMEI);
  SerialUSB.println("###\""APN"\"へ接続中...");
  delay(5000);
  if (!Wio.Activate(APN, USERNAME, PASSWORD)) {
    SerialUSB.println("### エラー!モバイルネットワーク回線への接続に失敗しました。APN設定を再度確認して、やり直して下さい。 ###");
    return;
  }
  SerialUSB.println("###オンライン");
}

void loop()
{
  SerialUSB.println("取得開始");

  //磁力の取得
  int mx, my, mz;
  char magnet[256];
  if (mag.dataReady()) {
    mag.readMag(&mx, &my, &mz);
    sprintf(magnet, "{\"mx\":\"%d\",\"my\":\"%d\",\"mz\":\"%d\"}", mx, my, mz);
  }

  //気圧の取得
  float pressure = bmp280.getPressure();
  SerialUSB.print("気圧:  ");
  SerialUSB.println(pressure);

  //加速度ジャイロの取得
  bmi088.getAcceleration(&ax, &ay, &az);
  bmi088.getGyroscope(&gx, &gy, &gz);
  char accel[256];
  sprintf(accel, "{\"ax\":\"%.02f\",\"ay\":\"%.02f\",\"az\":\"%.02f\"}", ax, ay, az);
  char gyro[256];
  sprintf(gyro, "{\"gx\":\"%.02f\",\"gy\":\"%.02f\",\"gz\":\"%.02f\"}", gx, gy, gz);
  //  SerialUSB.print(accel); SerialUSB.println(gyro);

  //温度湿度の取得
  float temp;
  float humi;
  if (!TemperatureAndHumidityRead(&temp, &humi)) {
    SerialUSB.println("ERROR!");
    goto err;
  }

  if (nmea.isValid()) {
    ledState = true;

    //    //日時の取得
    char datetime[256];
    sprintf(datetime, "%04d-%02d-%02d %02d:%02d:%02d", int(nmea.getYear()), int(nmea.getMonth()), int(nmea.getDay()), int(nmea.getHour()), int(nmea.getMinute()), int(nmea.getSecond()));

    //    //位置情報の取得
    long latitude_mdeg = nmea.getLatitude();
    long longitude_mdeg = nmea.getLongitude();
    long speeds = nmea.getSpeed();
    long course = nmea.getCourse();

    char GPS[256];
    sprintf(GPS, "{\"latitude_mdeg\":\"%.06f\",\"longitude_mdeg\":\"%.06f\",\"speed\":\"%.02f\",\"course\":\"%.02f\"}", float(latitude_mdeg) / 1000000., float(longitude_mdeg) / 1000000., float(speeds) / 10000., float(course) / 10000.);

    sending(IMEI, datetime, GPS, temp, humi, accel, pressure, magnet, gyro);
    nmea.clear();
  }
  else
  {
    ledState = false;
  }

  led(ledState);

  while (gps.available()) {
    char c = gps.read();
    if (!ledState) SerialUSB.print(c);
    nmea.process(c);
  }

err:
  delay(3000);
}

//データの送信
void sending(char* IMEI, char* datetime, char* GPS, float temp, float humid, char* accel, float pressure, char* magnet, char* gyro) {
  int status; //POSTした時のレスポンス値を保存するための変数
  char sendingdata[1024];
  SerialUSB.println("### POST中");
  SerialUSB.print("Post:");
  sprintf(sendingdata, "{\"TrackingSensor\":{\"IMEI\":\"%s\",\"SensorContents\":{\"Timestamp\":\"%s\",\"GPS\":%s,\"Temp\":\"%.02f\",\"Humid\":\"%.02f\",\"Acceleration\":%s,\"Pressure\":\"%.02f\",\"Magnetism\":%s,\"Gyro\":%s}}}", IMEI, datetime, GPS, temp, humid, accel, pressure, magnet, gyro);
  SerialUSB.print(sendingdata);
  if (!Wio.HttpPost(BASEURL, sendingdata, & status)) { //POSTを送信
    SerialUSB.println("### エラー! ###"); //以下POST失敗時の処理
    goto err;
  }
  SerialUSB.print("Status:");
  SerialUSB.println(status);
  SerialUSB.println("");
err:
  SerialUSB.println("### インターバル中");
}

////////////////////////////////////////////////////////////////////////////////////////
//温度湿度センサーの設定

int TemperatureAndHumidityPin;

void TemperatureAndHumidityBegin(int pin)
{
  TemperatureAndHumidityPin = pin;
  DHT11Init(TemperatureAndHumidityPin);
}

bool TemperatureAndHumidityRead(float* temperature, float* humidity)
{
  byte data[5];

  DHT11Start(TemperatureAndHumidityPin);
  for (int i = 0; i < 5; i++) data[i] = DHT11ReadByte(TemperatureAndHumidityPin);
  DHT11Finish(TemperatureAndHumidityPin);

  if (!DHT11Check(data, sizeof (data))) return false;
  if (data[1] >= 10) return false;
  if (data[3] >= 10) return false;

  *humidity = (float)data[0] + (float)data[1] / 10.0f;
  *temperature = (float)data[2] + (float)data[3] / 10.0f;

  return true;
}

////////////////////////////////////////////////////////////////////////////////////////
//

void DHT11Init(int pin)
{
  digitalWrite(pin, HIGH);
  pinMode(pin, OUTPUT);
}

void DHT11Start(int pin)
{
  // Host the start of signal
  digitalWrite(pin, LOW);
  delay(18);

  // Pulled up to wait for
  pinMode(pin, INPUT);
  while (!digitalRead(pin)) ;

  // Response signal
  while (digitalRead(pin)) ;

  // Pulled ready to output
  while (!digitalRead(pin)) ;
}

byte DHT11ReadByte(int pin)
{
  byte data = 0;

  for (int i = 0; i < 8; i++) {
    while (digitalRead(pin)) ;

    while (!digitalRead(pin)) ;
    unsigned long start = micros();

    while (digitalRead(pin)) ;
    unsigned long finish = micros();

    if ((unsigned long)(finish - start) > 50) data |= 1 << (7 - i);
  }

  return data;
}

void DHT11Finish(int pin)
{
  // Releases the bus
  while (!digitalRead(pin)) ;
  digitalWrite(pin, HIGH);
  pinMode(pin, OUTPUT);
}

bool DHT11Check(const byte* data, int dataSize)
{
  if (dataSize != 5) return false;

  byte sum = 0;
  for (int i = 0; i < dataSize - 1; i++) {
    sum += data[i];
  }

  return data[dataSize - 1] == sum;
}

////////////////////////////////////////////////////////////////////////////////////////
