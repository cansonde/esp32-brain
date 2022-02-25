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
    original_value-=minim;
    maxim -= minim;
    unsigned int maxval = (1<<bits)-1;
    float ret = (original_value/maxim)*(float)maxval;
    return (unsigned int) ret;
}

void updatepack(bool bit)
{
    if(packsize>=7)
    {
        pack+=bit;
        ret+=String(pack);
        //cerr<<bitset<8>(pack)<<"\n";
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

    for (int i = 0; i < 25; i++)
    {
        for (int j = 0; j < sizes[i]; j++)
        {
            updatepack((values[i] & (1<<j)));
        }
    }
}

void getframe(API &api)
{
    // for (int i = 1; i < 25; i++)   
    // cin>>values[i];
    values[1] = g_frame;
    g_frame++;
    api.gps_update();
    values[2] = map(api.gps_latitute(),16,20,sizes[2]);//WARNING nwm czy nie zamienic longitude i latitude bo w ramce jest co innego i co innego ustalilismy 
    values[3] = map(api.gps_longitude(),48,52,sizes[3]);
    values[4] = (unsigned int)  api.gps_altitude();
    values[5] = api.gps_number_of_satellites();   
    values[6] = map(api.tmp_dht(),-20,40,sizes[6]);
    values[7] = map(api.tmp_ds18(),-20,40,sizes[7]);
    values[8] = map(api.tmp_thermistor(),-20,40,sizes[8]); //TODO thermistor temp
    values[9] = map(api.tmp_280(),-20,40,sizes[9]);//TODO
    values[10] = (unsigned int) api.hum_dht();
    values[11] = map(api.bat_vol(),2.5f,4.5f,sizes[11]); //TODO battery voltage
   
    //TODO particles
    values[12] = 
    values[13] = 
    //<><><><><><

    values[14] = (unsigned int)api.bar_bmp() - 500;// 500 offset

    //TODO gyro
    values[15] = 
    values[16] = 
    values[17] = 
    
    values[18] = 
    values[19] = 
    values[20] = 
    //<><><><><><
    
    values[21] = api.dbi(); //TODO dbi
    values[22] = api.state(); //TODO state
    values[24] = CRC(); 



    send();
    // int frameNr, X_gps, Y_gps, h_gps, meta, tempDHT11, tempDS18B20,  tempThermistor, humidity,
    // batteryVoltage,  smallParticles,  mediumParticles,  bigParticles, pressure;
    // int Xgyro, Ygyro, Zgyro, Xacceleration, Yacceleration, Zacceleration, dbi,  state;
    // int airflow,crc=0;
}