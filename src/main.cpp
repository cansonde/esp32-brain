#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

#include "api.hpp"
#include "packet.h"

#define THERM_PIN 32
#define SD_CS_PIN 27
#define BUZZ_PIN  15

File cansat_data;
API api;

void setup() {
  Serial.begin(9600);
  delay(3000);
  ledcAttachPin(BUZZ_PIN, 0);

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card inicialization failed");
    delay(1000);
  }
  
  String filename = "cansat";
  int file_number = 0;
  // dont overwrite existing files
  while (SD.exists((char*) (void*) ("/" + filename + String(file_number) + ".dat").c_str())) {
    file_number++;
  }

  filename = "/" + filename + String(file_number) + ".dat";
  Serial.println("Opening: " + filename);
  cansat_data = SD.open((char*) (void*) filename.c_str(), FILE_WRITE);
  if (!cansat_data) {
    Serial.println("Openinng file in SD card failed");
    cansat_data.close();
    delay(100);
  }

  api.init();
}

void sd_save_frame(String data);

unsigned int ms_1000_timer = 0;
unsigned int ms_100_timer = 0;

bool tone = false;
void loop() {
  bool ms_100_ev_en = (millis()-ms_100_timer) > 100;
  bool ms_1000_ev_en = (millis()-ms_1000_timer) > 1000;
  if(!ms_100_ev_en)
    return; // continue
  
  // if(ms_1000_ev_en) {
  //   if(tone)
  //     ledcWriteTone(0, 2100);
  //   else
  //     ledcWriteTone(0, 1000);
  //   tone ^= 1;
  // }

  if(ms_1000_ev_en) {
    api.gps_update();
    api.particle_update();
  
    Serial.printf("TMP DS18: %f\n", api.tmp_ds18());
    Serial.printf("TMP DHT : %f\n", api.tmp_dht());
    Serial.printf("TMP BMP : %f\n", api.tmp_bmp());
    Serial.printf("TMP GYR : %f\n", api.tmp_gyr());
    Serial.printf("HUM DHT : %f\n", api.hum_dht());
    Serial.printf("BAR BMP : %f\n", api.bar_bmp());
    Serial.printf("GPS TIME: %f\n", api.gps_time());
    Serial.printf("GPS_LAT : %f\n", api.gps_latitute());
    Serial.printf("GPS_LON : %f\n", api.gps_longitude());
    Serial.printf("GPS_QUA : %d\n", api.gps_quality());
    Serial.printf("GPS_NUM : %d\n", api.gps_number_of_satellites());
    Serial.printf("GPS_PREC: %f\n", api.gps_horizontal_precision());
    Serial.printf("GPS_ALT : %f\n", api.gps_altitude());
    Serial.printf("PAR     : %d\n", api.par_cnt());
    Serial.printf("BAT_VOLT: %f\n", api.bat_volt());
    float x,y,z;
    api.gyr_acc(x,y,z);
    Serial.printf("ACC : %f %f %f\n", x, y, z);
    Serial.printf("----------\n");
  }

  String frs = getframe(api, ms_1000_ev_en); // also sends frame
  if(ms_1000_ev_en) {
    api.lora_send(frs);
  }
  if(ms_100_ev_en) {
    sd_save_frame(frs);
  }

  if(ms_1000_ev_en)
    ms_1000_timer = millis();
  if(ms_100_ev_en)
    ms_100_timer = millis();

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
 /* 
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
  */
  // sensors_event_t a, g, temp;
  // gyr_sensor.getEvent(&a, &g, &temp);
}

void sd_save_frame(String s) {
  Serial.println("sd");
  String hexstr = "";

  for(int i=0; i<s.length(); i++) {
    int lower = s[i]&0x0F;
    int upper = (s[i]&0xF0)>>4;
    char lc = (lower < 10 ? '0'+lower : 'a'+(lower-10));
    char uc = (upper < 10 ? '0'+upper : 'a'+(upper-10));
    hexstr += uc;
    hexstr += lc;  
  }
  hexstr += '\n';
  //cansat_data.write((const uint8_t*)hexstr.c_str(), hexstr.length());
  cansat_data.print(hexstr);
  cansat_data.flush();
}