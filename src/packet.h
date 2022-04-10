#pragma once

#include <Arduino.h>
#include "api.hpp"
const int sizes[24] = {8,16,15,15,12,4,9,9,9,9,7,8,32,32,9,8,8,8,8,8,8,8,16};
unsigned int values[24];

// | const 8b | frame nr 16b | gps x-15b (map 15b, 16-20)  y-15b, (map 15b, 48-52) h-12b (raw) , meta(num sat)-4b   
//| -20, 43 9b temp *4 (map 9b, -20, 40) | 7b hum (raw) | map(2.5-4.5,8b) bat voltage | 
//32b particle counter * 2 (size_sum, last size???) | pressure  (raw 9b off +500)| 8b * 6 gyro | 8b state | 16b CRC  
String ret;
int CRC()
{
    return 0;
}
unsigned char pack=0;
int packsize=0;

int g_frame = 0;
unsigned int map(float original_value, float minim, float maxim,int bits)
{
    if(original_value < minim) original_value = minim;
    if(original_value > maxim) original_value = maxim;
    original_value-=minim;
    maxim -= minim;
    unsigned int maxval = (1<<bits)-1;
    float ret = (original_value/maxim)*(float)maxval;
    return (unsigned int) ret;
}

void updatepack(bool bit)
{
    // Serial.print(bit);
    // Serial.print(' ');
    // Serial.print(pack);
    // Serial.print('\n');
    if(packsize>=7)
    {
        pack+=bit;
        ret+=(char)pack;
        //cerr<<bitset<8>(pack)<<"\n";
        // Serial.println(pack);
        pack=0;
        packsize=0;
        return;
    }
    pack+=bit;
    pack*=2u;
    packsize++;
}
void send()
{

    values[0]='S';//|const 8b|
    for (int i = 0; i < 24; i++) // 25 ?
    {
//        Serial.println(sizes[i]);
        for (int j = sizes[i]-1; j >= 0 ; j--)
        {
            updatepack((values[i] & (1<<j)));
        }
    }

    for(int i=0; i<7; i++)
        updatepack(0); // pad with zeroes
    packsize = 0;
    pack = 0;
}

String getframe(API &api, bool inc_r_frame)
{
    // for (int i = 1; i < 25; i++)   
    // cin>>values[i];
    if(inc_r_frame)
        g_frame++;
    values[1] = g_frame;

    values[2] = map(api.gps_latitute(),48,52,sizes[2]);
    values[3] = map(api.gps_longitude(),16,20,sizes[3]);
    values[4] = (unsigned int)  api.gps_altitude();
    values[5] = api.gps_number_of_satellites();   
    values[6] = map(api.tmp_dht(),-20,40,sizes[6]);
    values[7] = map(api.tmp_ds18(),-20,40,sizes[7]);
    values[8] = 0; // map(api.tmp_thermistor(),-20,40,sizes[8]); //TODO thermistor temp
    values[9] = map(api.tmp_bmp(),-20,40,sizes[9]);
    values[10] = (unsigned int) api.hum_dht();
    values[11] = map(api.bat_volt(),2.5f,4.5f,sizes[11]); //TODO battery voltage
    
    //TODO particles
    values[12] = api.par_cnt();
    values[13] = api.gps_time();
    //<><><><><><

    values[14] = (unsigned int)api.bar_bmp() - 500;// 500 offset

    //TODO gyro
    values[15] = 0;
    values[16] = 0;
    values[17] = 0;
    
    float x, y, z;
    api.gyr_acc(x, y, z);
    values[18] = map(x, -80, 80, sizes[18]);
    values[19] = map(y, -80, 80, sizes[19]);
    values[20] = map(z, -80, 80, sizes[20]);
    //<><><><><><
    
    values[21] = 0; //no dbi
    values[22] = api.gps_quality(); //no state
    values[23] = CRC(); 

    ret = "";
    send();
    return ret;
    // int frameNr, X_gps, Y_gps, h_gps, meta, tempDHT11, tempDS18B20,  tempThermistor, humidity,
    // batteryVoltage,  smallParticles,  mediumParticles,  bigParticles, pressure;
    // int Xgyro, Ygyro, Zgyro, Xacceleration, Yacceleration, Zacceleration, dbi,  state;
    // int airflow,crc=0;
}