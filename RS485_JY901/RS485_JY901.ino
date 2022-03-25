#include <Wire.h>
#include "JY901.h"
#include <SoftwareSerial.h>
SoftwareSerial rs485(2, 3); //(Rx, Tx)

#define MODBUS_ADDR 0x50
#define READ_REG 0x03
#define WRITE_REG 0x06


byte unlockMaster[] = {MODBUS_ADDR, WRITE_REG, 0x00, 0x69, 0xB5, 0x88, 0x22, 0xA1};
byte setNormal[]={MODBUS_ADDR, WRITE_REG, 0x00, 0x01, 0x00, 0x00, 0xD5, 0x8B};
byte saveConfig[]={MODBUS_ADDR, WRITE_REG, 0x00, 0x00, 0x00, 0x00, 0x84, 0x4B};

byte readAngle[] = {MODBUS_ADDR, READ_REG, 0x00, 0x3d, 0x00, 0x03, 0x99, 0x86};
byte readAcc[] = {MODBUS_ADDR, READ_REG, 0x00, 0x34, 0x00, 0x03, 0x49, 0x84};
byte recData[11];

void sendCommand(byte command[]){
  rs485.write(command, 8);
  for(int i = 0;i<8;i++){
    Serial.print(command[i],HEX);
    Serial.print(",");
    }
  Serial.println();
  }

void calibrateAcc(){
  byte accCalmode[]={MODBUS_ADDR, WRITE_REG, 0x00, 0x01, 0x00, 0x01, 0x14, 0x4B};
  Serial.println("---------- Calibration Init ----------");
  sendCommand(accCalmode);
  delay(7000);
  sendCommand(setNormal);
  delay(1000);
  sendCommand(saveConfig);
  }

void rs485_send(byte command, byte message[]){
  byte data[9];
  data[0] = MODBUS_ADDR;
  data[1] = command;
  for(int i =2;i<10;i++){
    data[i] = message[i-2];
  }

  for(int i = 0;i<10;i++){
    Serial.print(data[i]);
    Serial.print(",");
  }
  Serial.println();
  rs485.write(data,10);
}

int rs485_receive(byte recv[]){
  Serial.println("rcv data init");
  unsigned long t = millis(); 
  while(1){
    if(millis() - t > 2000){
      return -1;
      break;
    }
    if(rs485.available()){
      Serial.println("rs485 available");
      rs485.readBytes(recv,11);
      return 0;
      break;
    }
  }
}


void setup() 
{
  Serial.begin(9600);
  rs485.begin(9600);
  Serial.println("---------- Serial Initiated ----------");
  rs485.write(unlockMaster, 8);
  for(int i = 0;i<8;i++){
    Serial.print(unlockMaster[i],HEX);
    Serial.print(",");
  }
  Serial.println();
  calibrateAcc();
  Serial.println("---------- Calibration Done ----------");
}


void loop() 
{ 
  rs485.write(readAngle, 8);
  for(int i = 0;i<8;i++){
    Serial.print(readAngle[i],HEX);
    Serial.print(",");
  }
  Serial.println();

  if(rs485_receive(recData) != -1){
    Serial.println("data recieved!");
    for(int i = 0;i<11;i++){
      Serial.print(recData[i]);
      Serial.print(",");
      }
      Serial.println();
  }
   else{
    Serial.println("no resp");
    Serial.println();    
    } 
   delay(5000);
}
