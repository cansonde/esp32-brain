#pragma once

#include <DallasTemperature.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <DHT.h>
#include <LoRa.h>

#define TMP_PIN_1       4
#define HUM_PIN_1       13
#define THE_PIN_1       32
#define LORA_RST_PIN    12
#define LORA_DIO0_PIN   35
#define LORA_NSS_PIN    5
#define SER_RX_PIN      33
#define SER_GPS_RX_PIN  16
#define BAT_PIN         25

/* additional pins
    SPI 19 18 23
    I2C 22 21
    UART 16(GPS) 33(PART)

SD_CS_PIN 2ddrd7
BUZZ_PIN  15
BAT_PIN 25
*/


class API {

public:

    API() : m_one(TMP_PIN_1), m_tmp(&m_one), m_hum(HUM_PIN_1, DHT11), m_ser(1), m_gser(2) { }
    
    void init() {
        //m_gser.begin(9600);
        m_ser.begin(9600, SERIAL_8N1, SER_RX_PIN, -1);
        m_gser.begin(9600, SERIAL_8N1, SER_GPS_RX_PIN, -1);

        m_tmp.begin();
        
        if (!m_bar.begin()) {
            Serial.println("[!] Count not find BMP280 sensor");
            delay(1000);
        }

        m_bar.setSampling(Adafruit_BMP280::MODE_NORMAL,
                          Adafruit_BMP280::SAMPLING_X2,
                          Adafruit_BMP280::SAMPLING_X16,
                          Adafruit_BMP280::FILTER_X16,
                          Adafruit_BMP280::STANDBY_MS_500);
        m_hum.begin();
        m_gyr.begin();
        m_gyr.setAccelerometerRange(MPU6050_RANGE_8_G);

        lora_init();

        while(m_gser.available())
            m_gser.read();
    }

    float tmp_ds18() {
        m_tmp.requestTemperatures();
        return m_tmp.getTempCByIndex(0);
    }

    float tmp_dht() {
        return m_hum.readTemperature();
    }

    float tmp_bmp() {
        return m_bar.readTemperature();
    }

    float tmp_gyr() {
        static sensors_event_t tmp;
        m_gyr.getTemperatureSensor()->getEvent(&tmp);
        return tmp.temperature;
    }

    float hum_dht() {
        return m_hum.readHumidity();
    }

    float bar_bmp() {
        return m_bar.readPressure() / 100;
    }

    float gps_time() {
        return m_gps_time;
    }

    float gps_latitute() {
        return m_gps_latitute;
    }

    float gps_longitude() {
        return m_gps_longitude;
    }

    int gps_quality() {
        return m_gps_fix_quality;
    }

    int gps_number_of_satellites() {
        return m_gps_number_of_satellites;
    }

    float gps_horizontal_precision() {
        return m_gps_horizontal_precision;
    }

    float gps_altitude() {
        return m_gps_altitude;
    }

    void gps_update() {
        Serial.println("Updating");
        //Serial.flush();
        
        unsigned int time_wait = millis();
        while (m_gser.available() > 0) {
            m_gser.setTimeout(20);
            
            String msg = m_gser.readStringUntil('\n');
            if(millis() - time_wait > 2000)
                return;

            int index = msg.indexOf("$GPGGA");

            if (index == -1) {
                continue;
            }
            
            Serial.println("Reading updates\n");

            sscanf(msg.c_str() + index, "$GPGGA,%f,%f,N,%f,E,%d,%d,%f,%f,%f",
                &m_gps_time,
                &m_gps_latitute,
                &m_gps_longitude,
                &m_gps_fix_quality,
                &m_gps_number_of_satellites,
                &m_gps_horizontal_precision,
                &m_gps_altitude,
                &m_gps_geoid_height
            );
        }
    }

    void gyr_acc(float &x, float &y, float &z) {
        static sensors_event_t acc;
        m_gyr.getAccelerometerSensor()->getEvent(&acc);
        x = acc.orientation.x;
        y = acc.orientation.y;
        z = acc.orientation.z;
    }

    void lora_init() {
        const long FREQ = 433.800e6; // channel 30
        
        LoRa.setPins(LORA_NSS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);

        if(!LoRa.begin(FREQ)) {
            Serial.println("[!] LoRa begin() failed!");
            delay(1000);
        }

        // Calculated for ~500 bps (sf11) or ~900 bps (sf10) 
        LoRa.setCodingRate4(5);
        LoRa.setSpreadingFactor(10);
        LoRa.setSignalBandwidth(125000);
       
        LoRa.setTxPower(20); // MAX POWER (20 dBm - 100mW)
    }

    static const size_t FRAME_SIZE = 291/8;
    unsigned char frame [FRAME_SIZE];
    void lora_send(String data) {
        // FIXME: test frame, replaace with pointer passed to function
        for(int i=0; i<FRAME_SIZE; i++)
            frame[i] = 0; 
        for(int i=0; i<min(FRAME_SIZE, data.length()); i++)
            frame[i] = data[i];

        unsigned int time_start = millis();
        while(!LoRa.beginPacket()){
            if(millis() - time_start > 1000) {
                return;
            }
            
            Serial.println("[!] LoRa currently TXing - blocking wait...");
        }
        
        LoRa.write(frame, FRAME_SIZE);
 
        LoRa.endPacket(true); // true - async mode (don't wait for TX end)
        Serial.println("LoRa packet sent!");
    }

    void particle_update() {
        if(!m_ser.available())
            return;
        
        String line_read = "";
        unsigned int time_wait = millis();
        while(m_ser.available()) {
            m_ser.setTimeout(10);
            
            if(millis() - time_wait > 200)
                return;

            String new_line  = m_ser.readStringUntil('\n');
            if(new_line.indexOf('x') == -1)
                break; // this is the last line that is not valid
            
            line_read = new_line;
        }

        if(line_read != "") {
            m_particle_count = atoi(line_read.substring(0, line_read.length()-2).c_str());
        }
    }

    int par_cnt() {
        return m_particle_count;
    }

    float bat_volt() {
        return ((analogRead(BAT_PIN) * 3.3)/4095)*3.f;
    }

private:

    OneWire           m_one;
    DallasTemperature m_tmp; // ds18b20
    Adafruit_BMP280   m_bar; // bmp280
    DHT               m_hum; // dht
    Adafruit_MPU6050  m_gyr; // mpu6050
    HardwareSerial    m_ser, m_gser; // hardware serial 1

    float             m_gps_time;
    float             m_gps_latitute;
    float             m_gps_longitude;
    int               m_gps_fix_quality;
    int               m_gps_number_of_satellites;
    float             m_gps_horizontal_precision;
    float             m_gps_altitude;
    float             m_gps_geoid_height;
    int               m_particle_count;

};