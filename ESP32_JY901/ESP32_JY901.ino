#include <HardwareSerial.h> //for rs485 comm
#define RXD2 5
#define TXD2 17
HardwareSerial rs485(2); // rxtx mode 2 of 0,1,2
#include "JY901.h"

const byte MODBUS_ADDR=0x50;
const byte READ_REG=0x03;
const byte WRITE_REG=0x06;


const byte unlockMaster[] = {MODBUS_ADDR, WRITE_REG, 0x00, 0x69, 0xB5, 0x88, 0x22, 0xA1};
const byte accCalmode[]={MODBUS_ADDR, WRITE_REG, 0x00, 0x01, 0x00, 0x01, 0x14, 0x4B};
const byte setNormal[]={MODBUS_ADDR, WRITE_REG, 0x00, 0x01, 0x00, 0x00, 0xD5, 0x8B};
const byte saveConfig[]={MODBUS_ADDR, WRITE_REG, 0x00, 0x00, 0x00, 0x00, 0x84, 0x4B};

const byte readAngle[] = {0x50, 0x03, 0x00, 0x3d, 0x00, 0x03, 0x99, 0x86};
const byte readAcc[] = {0x50, 0x03, 0x00, 0x34, 0x00, 0x03, 0x49, 0x84};
const byte readAngVel[] = {0x50, 0x03, 0x00, 0x37, 0x00, 0x03, 0xB9, 0x84};
byte recData[12];

void sendCommand(const byte command[]){
  for(int i = 0;i<8;i++){
    Serial.print(command[i],HEX);
    if(i != 7){
      Serial.print(",");
      }
    }
  rs485.write(command, 8);
  Serial.println();
  rs485.flush();
  }

void calibrateAcc(){
  Serial.println("---------- Calibration Init ----------");
  sendCommand(accCalmode);
  delay(5000);
  sendCommand(setNormal);
  delay(3000);
  sendCommand(saveConfig);
  delay(2000);
  }

void rs485_send(byte command, byte message[]){
  byte data[9];
  data[0] = MODBUS_ADDR;
  data[1] = command;
  for(int i =2;i<9;i++){
    data[i] = message[i-2];
  }

  for(int i = 0;i<10;i++){
    Serial.print(data[i]);
    if(i != 9){
      Serial.print(",");
      }
  }
  Serial.println();
  rs485.write(data,10);
}

int rs485_receive(byte recv[]){
//  Serial.println("rcv data init");
  unsigned long t = millis(); 
  while(1){
    if(millis() - t > 10000){
      return -1;
      break;
    }
//    if(rs485.available()){
//      rs485.readBytes(recv, 11);
//      }
    for (int i = 0; (rs485.available() > 0) && (i < 11); i++) {
      recv[i] = rs485.read();
      if (i == 10)
      Serial.println("read 12 byte");
    }
    Serial.println("reading completed");
    return 0;
    break;
  }
}

void printData(){
  float data_x = (((recData[3]<<8)|recData[4])*16*9.81)/32768;
  float data_y = (((recData[5]<<8)|recData[6])*16*9.81)/32768;
  float data_z = (((recData[7]<<8)|recData[8])*16*9.81)/32768;
  Serial.print(data_x);Serial.print("   "); Serial.print(data_y);Serial.print("   "); Serial.println(data_z);
  }

void setup() 
{
  Serial.begin(115200);
  rs485.begin(9600, SERIAL_8N1, RXD2, TXD2);
  rs485.flush();
  
  Serial.println("---------- Serial Initiated ----------");
  rs485.write(unlockMaster, 8);
  for(int i = 0;i<8;i++){
    Serial.print(unlockMaster[i],HEX);
    Serial.print(",");
  }
  Serial.println();
  rs485.flush();
  //calibrateAcc();
  Serial.println("---------- Calibration Done ----------");
}


void loop() 
{ 
  Serial.println();
  rs485.write(readAcc, 8);
  Serial.println("readAcc");
  delay(3000);
  rs485.flush();

  if(rs485_receive(recData) != -1){
    //Serial.println("data recieved!");
    for(int i = 0;i<11;i++){
      Serial.print(recData[i],HEX);
      Serial.print(",");
      }
      Serial.println();
    }
   else{
    Serial.println("no resp");
    Serial.println();    
    } 
//   printData();
   rs485.flush();
   delay(5000);
   
  rs485.write(readAngle, 8);
  Serial.println("readAng");
  delay(3000);
  rs485.flush();

  if(rs485_receive(recData) != -1){
    Serial.println("data recieved!");
    for(int i = 0;i<11;i++){
      Serial.print(recData[i],HEX);
      Serial.print(",");
      }
      Serial.println();
    }
   else{
    Serial.println("no resp");
    Serial.println();    
    } 
//   printData();
   rs485.flush();
}
