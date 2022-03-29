#include <HardwareSerial.h> //for rs485 comm
#define RXD2 5
#define TXD2 17
HardwareSerial rs485(2); // rxtx mode 2 of 0,1,2

//byte MODBUS_ADDR=0x50;
//byte READ_REG=0x03;
//byte WRITE_REG=0x06;


byte unlockMaster[] = {0x50, 0x06, 0x00, 0x69, 0xB5, 0x88, 0x22, 0xA1};
byte accCalmode[]={0x50, 0x06, 0x00, 0x01, 0x00, 0x01, 0x14, 0x4B};
byte magCalmode[]={0x50, 0x06, 0x00, 0x01, 0x00, 0x07, 0x94, 0x49};
byte setNormal[]={0x50, 0x06, 0x00, 0x01, 0x00, 0x00, 0xD5, 0x8B};
byte saveConfig[]={0x50, 0x06, 0x00, 0x00, 0x00, 0x00, 0x84, 0x4B};

byte readAngle[] = {0x50, 0x03, 0x00, 0x3d, 0x00, 0x03, 0x99, 0x86};
byte readAcc[] = {0x50, 0x03, 0x00, 0x34, 0x00, 0x03, 0x49, 0x84};
byte readAngVel[] = {0x50, 0x03, 0x00, 0x37, 0x00, 0x03, 0xB9, 0x84};
byte recData[12];
byte trashBuffer[100];

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
  Serial.println("---------- Acceleration Calibration Init ----------");
  sendCommand(unlockMaster,1);
  delay(500);
  sendCommand(accCalmode,1);
  delay(5000);
  sendCommand(setNormal,1);
  delay(1000);
  sendCommand(saveConfig,1);
  delay(1000);
  }
void calibrateMag(){
  Serial.println("---------- Magnetic Calibration Init ----------");
  sendCommand(unlockMaster,1);
  delay(500);
  sendCommand(magCalmode,1);
  Serial.println("---------- Slowly rotate in 3 axis ----------");
  delay(5000);
  Serial.println("---------- May stop now :) ----------");
  sendCommand(setNormal,1);
  delay(1000);
  sendCommand(saveConfig,1);
  delay(1000);  
  }
void rs485_send(byte command, byte message[]){
  byte data[9];
  data[0] = 0x50;
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

void printAccel(){
  float data_x = ((recData[3]<<8)|recData[4])/32768*16*9.81;
  float data_y = ((recData[5]<<8)|recData[6])/32768*16*9.81;
  float data_z = ((recData[7]<<8)|recData[8])/32768*16*9.81 - 9.81;
  Serial.print(data_x);Serial.print("   "); Serial.print(data_y);Serial.print("   "); Serial.println(data_z);
  }

void printAngle(){
  float data_x = ((recData[3]<<8)|recData[4])/32768*180;
  float data_y = ((recData[5]<<8)|recData[6])/32768*180;
  float data_z = ((recData[7]<<8)|recData[8])/32768*180;
  Serial.print(data_x);Serial.print("   "); Serial.print(data_y);Serial.print("   "); Serial.println(data_z);
  }

void setup() 
{
  Serial.begin(115200);
  rs485.begin(9600, SERIAL_8N1, RXD2, TXD2);
  rs485.flush();
  
  Serial.println("--------------- Serial Initiated ---------------");
  calibrateAcc();
  calibrateMag();
  Serial.println("--------------- Calibration Done ---------------");
  
  if(rs485_receive(trashBuffer, 71) != -1){
    //Serial.println("data recieved!");
    for(int i = 0;i<71;i++){
      Serial.print(trashBuffer[i],HEX);
      Serial.print(",");
      }
      Serial.println();
    }
}


void loop() 
{ 
  rs485.flush();
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
  printAccel();
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
  printAngle();
  delay(1000);
}
