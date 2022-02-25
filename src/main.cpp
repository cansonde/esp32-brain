#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

#include "api.hpp"

#define THERM_PIN 32
#define SD_CS_PIN 27
#define BUZZ_PIN  15

File cansat_data;
API api;

void setup() {
  Serial.begin(9600);

  // if (!SD.begin(SD_CS_PIN)) {
  //   Serial.println("SD card inicialization failed");
  //   for (;;) {
  //     Serial.println("Cannot init SD card\n");
  //   }
  // }

  // 2,1 kHz


  ledcAttachPin(BUZZ_PIN, 0);
  
  // String filename = "cansat";
  // int file_number = 0;

  // // dont overwrite existing files
  // while (SD.exists((char*) (void*) (filename + String(file_number) + ".dat").c_str())) {
  //   file_number++;
  // }

  // filename = filename + String(file_number) + ".dat";

  // Serial.println("Opening: " + filename);

  // cansat_data = SD.open("/cansat0.dat", FILE_WRITE);
  // // cansat_data = SD.open((char*) (void*) filename.c_str(), FILE_WRITE);

  // if (!cansat_data) {
  //   Serial.println("Openinng file in SD card failed");
  //   cansat_data.close();
  //   for (;;) {
  //     Serial.println("Cannot open a file: " + filename);
  //   }
  // }

  // api.init();
}

void loop() {

  ledcWriteTone(0, 2100);
  delay(500);
  ledcWriteTone(0, 1000);
  delay(500);

  // ledcWriteNote(0, NOTE_C, 4);
  // delay(500);
  // ledcWriteNote(0, NOTE_D, 4);
  // delay(500);
  // ledcWriteNote(0, NOTE_E, 4);
  // delay(500);
  // ledcWriteNote(0, NOTE_F, 4);
  // delay(500);
  // ledcWriteNote(0, NOTE_G, 4);
  // delay(500);
  // ledcWriteNote(0, NOTE_A, 4);
  // delay(500);
  // ledcWriteNote(0, NOTE_B, 4);
  // delay(500);
  // ledcWriteNote(0, NOTE_C, 5);
  // delay(500);


  return;

  const int len = 4;
  
  const unsigned char msg[len] = {'s', 'u', 's', '\n'};

  cansat_data.write(msg, len);  

  Serial.print("writing to ");
  Serial.println(cansat_data.name());

  cansat_data.flush();
  // api.gps_update();

  // Serial.printf("TMP DS18: %f\n", api.tmp_ds18());
  // Serial.printf("TMP DHT : %f\n", api.tmp_dht());
  // Serial.printf("TMP BMP : %f\n", api.tmp_bmp());
  // // Serial.printf("TMP GYR : %f\n", api.tmp_gyr());
  // Serial.printf("HUM DHT : %f\n", api.hum_dht());
  // Serial.printf("BAR BMP : %f\n", api.bar_bmp());
  // Serial.printf("GPS TIME: %f\n", api.gps_time());
  // Serial.printf("GPS_LAT : %f\n", api.gps_latitute());
  // Serial.printf("GPS_LON : %f\n", api.gps_longitude());
  // Serial.printf("GPS_QUA : %d\n", api.gps_quality());
  // Serial.printf("GPS_NUM : %d\n", api.gps_number_of_satellites());
  // Serial.printf("GPS_PREC: %f\n", api.gps_horizontal_precision());
  // Serial.printf("GPS_ALT : %f\n", api.gps_altitude());
  // Serial.printf("----------\n");

  // api.lora_send();

  delay(100000);

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