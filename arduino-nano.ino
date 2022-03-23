#include <Wire.h>
#include "JY901.h"

#define I2C_ADDR 0x21
//#define I2C_ADDR 0x22

int counts = 0;
double sum[9]= {0,0,0,0,0,0,0,0,0};
int buffer_size = 11;// jolla big

//dummy
float Acc[3]; 
float Gyro[3]; 
float Ang[3]; 
    
void getSensorData()
{
    for(int i=0; i<3; i++){
      Acc[i]=JY901.stcAcc.a[i]/32768*16;
      Gyro[i]=JY901.stcGyro.w[i]/32768*2000;
      Ang[i]=JY901.stcAngle.Angle[0]/32768*180;
      sum[i]+=Acc[i];
      sum[i+3]+=Gyro[i];
      sum[i+6]+=Ang[i];
      }
    counts++;
    Serial.print("ACC_x : ");
    Serial.print(Acc[0]);
    Serial.print("  ACC_y : ");
    Serial.print(Acc[1]);
    Serial.print("  ACC_z : ");
    Serial.print(Acc[2]);
    Serial.print("\n");
    Serial.print("Gyro_x : ");
    Serial.print(Gyro[0]);
    Serial.print("  Gyro_y : ");
    Serial.print(Gyro[1]);
    Serial.print("  Gyro_z : ");
    Serial.print(Gyro[2]);
    Serial.print("\n");
    Serial.print("Ang_x : ");
    Serial.print(Ang[0]);
    Serial.print("  Ang_y : ");
    Serial.print(Ang[1]);
    Serial.print("  Ang_z : ");
    Serial.print(Ang[2]);
    Serial.print("\n");
}

void setup()
{
    Serial.begin(9600);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    Wire.begin(I2C_ADDR);
    Wire.onRequest(sendToMaster);
}

void loop()
{
    getSensorData();
    delay(500);
}

void serialEvent() 
{
  while (Serial.available()) 
  {
    JY901.CopeSerialData(Serial.read()); //Call JY901 data cope function
  }
}

uint8_t calc_checksum(uint8_t* ptr, int len)
{
    uint8_t checksum = 0;
    for (int i = 0; i < len; i++)
        checksum += ptr[i];
    return checksum;
}

// need to change buffer
void sendToMaster()
{   
    // if uint8_t cover ok buf size small
    uint16_t data[9];
    for(int i=0;i<9;i++){
      data[i] = sum[i] / counts;
      sum[i] = 0;
      }
    counts = 0;
    uint8_t buf[buffer_size]; // jolla big needed
    //divide data to buf[0] and buf[1] --> front 8 bit on buf[0] back 8 bit on buf[1]
    buf[0] = (data[0] >> 8) & 0xFF; // move 8 bits right and do 'and' with 11111111why?
    buf[1] = data[0] & 0xFF; // 11111111(=0xFF)
    buf[9] = calc_checksum(buf, buffer_size-1);
    for (int i = 0; i < buffer_size; i++)
        Wire.write(buf[i]);
}
