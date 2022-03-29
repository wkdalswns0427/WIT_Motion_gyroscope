#include <HardwareSerial.h> //for rs485 comm
#define RXD2 5
#define TXD2 17
HardwareSerial rs485(2); // rxtx mode 2 of 0,1,2

byte prev_ADDR = 0x50;
byte new_ADDR = 0x51;
byte WRITE_REG = 0x06;

byte unlockMaster[] = {0x50, 0x06, 0x00, 0x69, 0xB5, 0x88, 0x22, 0xA1};
byte changeADDR[] = {0x50, 0x06, 0x00, 0x1a, 0x00, new_ADDR, 0x64, 0x70};
byte saveConfig[]={0x51, 0x06, 0x00, 0x00, 0x00, 0x00, 0x84, 0x4B};

byte accCalmode[]={0x51, 0x06, 0x00, 0x01, 0x00, 0x01, 0x14, 0x4B};
byte setNormal[]={0x51, 0x06, 0x00, 0x01, 0x00, 0x00, 0xD5, 0x8B};

byte readAngle[] = {0x51, 0x03, 0x00, 0x3d, 0x00, 0x03, 0x99, 0x86};
byte readAcc[] = {0x51, 0x03, 0x00, 0x34, 0x00, 0x03, 0x49, 0x84};
byte readAngVel[] = {0x51, 0x03, 0x00, 0x37, 0x00, 0x03, 0xB9, 0x84};
byte recData[12];
byte trashBuffer[70];

int flag = 0;

void sendCommand(byte command[8], int prt){
  byte data[10];
  for(int i=0;i<8;i++){
    data[i]=command[i];
    }
  if(prt==1){
    for(int i = 0;i<8;i++){
      Serial.print(command[i],HEX);
      if(i != 7){
        Serial.print(",");
        }
      }
    }
  rs485.write(data, 8);
  Serial.println();
  rs485.flush();
  }

void calibrateAcc(){
  Serial.println("---------- Calibration Init ----------");
  sendCommand(unlockMaster,1);
  delay(500);
  sendCommand(accCalmode,1);
  delay(5000);
  sendCommand(setNormal,1);
  delay(3000);
  sendCommand(saveConfig,1);
  delay(2000);
  }

void changeAddress(){
  Serial.println("change modbus address 0x50 --> 0x51");
  sendCommand(unlockMaster,1);
  delay(500);
  sendCommand(changeADDR,1);
  delay(500);
  sendCommand(saveConfig,1);
  delay(1000);
  
  }

int rs485_receive(byte recv[], int num){
//  Serial.println("rcv data init");
  unsigned long t = millis(); 
  while(1){
    if(millis() - t > 10000){
      return -1;
      break;
    }
    for (int i = 0; (rs485.available() > 0) && (i < num); i++) {
      recv[i] = rs485.read();
    }
    return 0;
    break;
  }
}

void printData(){
  float data_x = (((recData[3]<<8)|recData[4])*16*9.81)/32768;
  float data_y = (((recData[5]<<8)|recData[6])*16*9.81)/32768;
  float data_z = (((recData[7]<<8)|recData[8])*16*9.81)/32768 - 9.81;
  Serial.print(data_x);Serial.print("   "); Serial.print(data_y);Serial.print("   "); Serial.println(data_z);
  }

void setup() 
{
  Serial.begin(115200);
  rs485.begin(9600, SERIAL_8N1, RXD2, TXD2);
  rs485.flush();
  
  Serial.println("---------- Serial Initiated ----------");
  Serial.println();
  //changeAddress();
  //Serial.println("---------- Address Changed ----------");
  calibrateAcc();
  Serial.println("---------- Calibration done ----------");
  
  if(rs485_receive(trashBuffer, 63) != -1){
    //Serial.println("data recieved!");
    for(int i = 0;i<63;i++){
      Serial.print(trashBuffer[i],HEX);
      Serial.print(",");
      }
      Serial.println();
    }
}


void loop() 
{ 
  if(++flag==1){
    Serial.println("WT901C485 read");
    }
  sendCommand(readAcc,0);
  Serial.println("Acceleration");
  delay(2000);
  rs485.flush();

  if(rs485_receive(recData, 11) != -1){
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
  printData();
  rs485.flush();
  delay(1000);

  sendCommand(readAngle,0);
  Serial.println("Angle");
  delay(2000);
  rs485.flush();

  if(rs485_receive(recData, 11) != -1){
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
  printData();
  delay(1000);
  rs485.flush();
}
