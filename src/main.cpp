#include <Arduino.h>
#include "api.hpp"

#define THERM_PIN 32

/*
#define TMP_PIN_1 23
#define HUM_PIN_1 13

OneWire oneWire(TMP_PIN_1);

DallasTemperature tmp_sensor(&oneWire);
Adafruit_BMP280   bar_sensor;
DHT               hum_sensor(HUM_PIN_1, DHT11);
Adafruit_MPU6050  gyr_sensor;

String neoexample = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";

void calibrate_gyro();

void parseGPS(const String& s) {

  float time;
  float latitute;
  float longitude;
  int fix_quality;
  int number_of_satellites;
  float horizontal_precision;
  float altitude;

  sscanf(s.c_str(), "$GPGGA,%f,%f,N,%f,E,%d,%d,%f,%f", &time, &latitute, &longitude, &fix_quality, &number_of_satellites, &horizontal_precision, &altitude);

  Serial.print("GMP :");
  Serial.printf(" time = %f, lat = %f, lon = %f, fix = %d, #sat = %d, pre = %f, alt = %f\n", time, latitute, longitude, fix_quality, number_of_satellites, horizontal_precision, altitude);
}

void setup() {


  Serial.begin(9600);
  Serial2.begin(9600);

  tmp_sensor.begin();

  if (!bar_sensor.begin()) {
    Serial.println("Count not find BMP280 sensor");
  }
  bar_sensor.setSampling(Adafruit_BMP280::MODE_NORMAL,    
                         Adafruit_BMP280::SAMPLING_X2,    
                         Adafruit_BMP280::SAMPLING_X16,   
                         Adafruit_BMP280::FILTER_X16,     
                         Adafruit_BMP280::STANDBY_MS_500);

  hum_sensor.begin();

  if (!gyr_sensor.begin()) {
    for (;;) {
      Serial.println("panic: gyr_sensor");
      Serial.flush();
    }
  }

  gyr_sensor.setAccelerometerRange(MPU6050_RANGE_8_G);
  gyr_sensor.setGyroRange(MPU6050_RANGE_1000_DEG);
  gyr_sensor.setFilterBandwidth(MPU6050_BAND_21_HZ);
  calibrate_gyro();
  // gyr_sensor.setgyr_sensorConfig(2);
  // gyr_sensor.setAccConfig(2);
  //gyr_sensor.calcOffsets();

  Serial.println("cansus inited");
}
*/

API api;

void setup() {
  Serial.begin(9600);
  api.init();
}

void loop() {

  api.gps_update();

  Serial.printf("TMP DS18: %f\n", api.tmp_ds18());
  Serial.printf("TMP DHT : %f\n", api.tmp_dht());
  Serial.printf("TMP BMP : %f\n", api.tmp_bmp());
  // Serial.printf("TMP GYR : %f\n", api.tmp_gyr());
  Serial.printf("HUM DHT : %f\n", api.hum_dht());
  Serial.printf("BAR BMP : %f\n", api.bar_bmp());
  Serial.printf("GPS TIME: %f\n", api.gps_time());
  Serial.printf("GPS_LAT : %f\n", api.gps_latitute());
  Serial.printf("GPS_LON : %f\n", api.gps_longitude());
  Serial.printf("GPS_QUA : %d\n", api.gps_quality());
  Serial.printf("GPS_NUM : %d\n", api.gps_number_of_satellites());
  Serial.printf("GPS_PREC: %f\n", api.gps_horizontal_precision());
  Serial.printf("GPS_ALT : %f\n", api.gps_altitude());
  Serial.printf("----------\n");

  api.lora_send();

  delay(300);

  return;

  // while (Serial2.available() > 0) {
  //   String msg = Serial2.readStringUntil('\n');

  //   if (msg.indexOf("$GPGGA") == -1)
  //     continue;

  //   parseGPS(msg);
  // }

  // Serial.print("HUM : ");
  // Serial.println(hum_sensor.readHumidity());

  // Serial.print("TMP2: ");
  // Serial.println(hum_sensor.readTemperature());

  // tmp_sensor.requestTemperatures();

  // Serial.print("TMP1: ");
  // Serial.println(tmp_sensor.getTempCByIndex(0));

  // Serial.print("BAR :");
  // Serial.print(bar_sensor.readPressure() / 100);
  // Serial.println(" hPa");

  // gyr_sensor.
  //  sensors_event_t a, g, tempz;
  // gyr_sensor.getEvent(&a, &g, &tempz);

  /* Print out the values */
  // Serial.print("Acceleration X: ");
  // Serial.print(a.acceleration.x);
  // Serial.print(", Y: ");
  // Serial.print(a.acceleration.y);
  // Serial.print(", Z: ");
  // Serial.print(a.acceleration.z);
  // Serial.println(" m/s^2");

  // Serial.print("Rotation X: ");
  // Serial.print(g.gyro.x);
  // Serial.print(", Y: ");
  // Serial.print(g.gyro.y);
  // Serial.print(", Z: ");
  // Serial.print(g.gyro.z);
  // Serial.println(" rad/s");

  // Serial.print("Temperature: ");
  // Serial.print(temp.temperature);
  // Serial.println(" degC");
  
  // float elapsed = (millis() - last_time) / 1000;
  // last_time = millis();

  // integral_x += g.gyro.x;
  // integral_y += g.gyro.y;
  // integral_z += g.gyro.z;

  // integral_x += (g.gyro.x-x_avg) * elapsed;
  // integral_y += (g.gyro.y-y_avg) * elapsed;
  // integral_z += (g.gyro.z-z_avg) * elapsed;
  // integral_x += g.gyro.x/(float)time_passed;
  // integral_y += g.gyro.y/(float)time_passed;
  // integral_z += g.gyro.z/(float)time_passed;

  // printf("integrated: %f %f %f\n", integral_x, integral_y, integral_z);

  //Serial.printf("%f %f %f\n", g.gyro.heading, g.gyro.pitch, g.gyro.roll)
  
  const double BETA               = 3270.0;
  const double MAX_ADC            = 4096.0;
  const double RESITOR2           = 4704.0;
  const double THERM_NOMINAL      = 1000.0;
  const double THERM_NOMINAL_TEMP = 25.0+273.15;

  double volt_avg = analogRead(THERM_PIN);
  // for(int i=0; i<10; i++)
  //   volt_avg += ;
  // volt_avg /= 10; 

  double therm_resistance = RESITOR2 * ( (MAX_ADC / volt_avg) - 1);
  double temp = (BETA * THERM_NOMINAL_TEMP) / 
           (BETA + (THERM_NOMINAL_TEMP * log(therm_resistance / THERM_NOMINAL)));

  Serial.printf("TERM %lf %lf %lf\n", temp, therm_resistance, volt_avg);

  // Serial.println("");
  delay(100);
  // sensors_event_t a, g, temp;
  // gyr_sensor.getEvent(&a, &g, &temp);
}