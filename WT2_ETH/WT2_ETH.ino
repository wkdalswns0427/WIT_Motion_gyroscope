#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
const char* ssid = "yout SSID";
const char* password = "your PW";
const uint16_t port = 6040;//임시
const char* host = "3.38.162.240"; // new RDS
WiFiClient client;

#include <HardwareSerial.h> //for rs485 comm
#define RXD2 5
#define TXD2 17
HardwareSerial rs485(2); // rxtx mode 2 of 0,1,2

/*
#include <Arduino.h> //for ethernet
#include <ETH.h>     //for ethernet
#define ETH_POWER_PIN   16
#define ETH_MDC_PIN     23
#define ETH_MDIO_PIN    18
#define ETH_ADDR        1
#define ETH_CLK_MODE    ETH_CLOCK_GPIO0_IN
#define ETH_TYPE        ETH_PHY_LAN8720
static bool eth_connected = false;
#include <HTTPClient.h>
#include <string.h>
*/

#include <math.h>
byte unlockMaster1[]={0x50, 0x06, 0x00, 0x69, 0xB5, 0x88, 0x22, 0xA1};
byte changeADDR1[] = {0x50, 0x06, 0x00, 0x1A, 0x00, 0x51, 0x64, 0x70};
byte accCalmode1[]={0x50, 0x06, 0x00, 0x01, 0x00, 0x01, 0x14, 0x4B};
byte magCalmode1[]={0x50, 0x06, 0x00, 0x01, 0x00, 0x07, 0x94, 0x49};
byte setNormal1[]={0x50, 0x06, 0x00, 0x01, 0x00, 0x00, 0xD5, 0x8B};
byte saveConfig1[]={0x50, 0x06, 0x00, 0x00, 0x00, 0x00, 0x84, 0x4B};
byte readAngle1[] = {0x50, 0x03, 0x00, 0x3d, 0x00, 0x03, 0x99, 0x86};
byte readAcc1[] = {0x50, 0x03, 0x00, 0x34, 0x00, 0x03, 0x49, 0x84};
byte readAngVel1[] = {0x50, 0x03, 0x00, 0x37, 0x00, 0x03, 0xB9, 0x84};

byte unlockMaster2[] = {0x51, 0x06, 0x00, 0x69, 0xB5, 0x88, 0x23, 0x70};
byte saveConfig2[]={0x51, 0x06, 0x00, 0x00, 0x00, 0x00, 0x85, 0x9A};
byte accCalmode2[]={0x51, 0x06, 0x00, 0x01, 0x00, 0x01, 0x15, 0x9A};
byte magCalmode2[]={0x51, 0x06, 0x00, 0x01, 0x00, 0x07, 0x95, 0x98};
byte setNormal2[]={0x51, 0x06, 0x00, 0x01, 0x00, 0x00, 0xD4, 0x5A};
byte readAngle2[] = {0x51, 0x03, 0x00, 0x3D, 0x00, 0x03, 0x98, 0x57};
byte readAcc2[] = {0x51, 0x03, 0x00, 0x34, 0x00, 0x03, 0x48, 0x55};
byte readAngVel2[] = {0x51, 0x03, 0x00, 0x37, 0x00, 0x03, 0xB8, 0x55};

byte recData1[12];
byte recData2[12];
byte trashBuffer[100];

static byte accDiff[] = {0,0,0};
static byte angDiff[] = {0,0,0};
static byte angvelDiff[] = {0,0,0};

int flag = 0;

/*
void WiFiEvent(WiFiEvent_t event)
{

  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("ETH Started");
      //set eth hostname here
      ETH.setHostname("esp32-ethernet");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("ETH Connected");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.print("ETH MAC: ");
      Serial.print(ETH.macAddress());
      Serial.print(", IPv4: ");
      Serial.print(ETH.localIP());
      if (ETH.fullDuplex()) {
        Serial.print(", FULL_DUPLEX");
      }
      Serial.print(", ");
      Serial.print(ETH.linkSpeed());
      Serial.println("Mbps");
      eth_connected = true;
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH Disconnected");
      eth_connected = false;
      break;
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("ETH Stopped");
      eth_connected = false;
      break;
    default:
      break;
  }
}
*/

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

void calibrateAcc(int type){
    if(type==1){
      Serial.println("---------- Acceleration Calibration Init ----------");
      sendCommand(unlockMaster1,1);
      delay(500);
      sendCommand(accCalmode1,1);
      delay(5000);
      sendCommand(setNormal1,1);
      delay(1000);
      sendCommand(saveConfig1,1);
      delay(1000);
    }
    else if(type==2){
      Serial.println("---------- Acceleration Calibration Init ----------");
      sendCommand(unlockMaster2,1);
      delay(500);
      sendCommand(accCalmode2,1);
      delay(5000);
      sendCommand(setNormal2,1);
      delay(1000);
      sendCommand(saveConfig2,1);
      delay(1000);
    }
  }
  
void calibrateMag(int type){
    if(type==1){
      Serial.println("---------- Magnetic Calibration Init ----------");
      sendCommand(unlockMaster1,1);
      delay(500);
      sendCommand(magCalmode1,1);
      Serial.println("---------- Slowly rotate in 3 axis ----------");
      delay(5000);
      Serial.println("---------- May stop now :) ----------");
      sendCommand(setNormal1,1);
      delay(1000);
      sendCommand(saveConfig1,1);
      delay(1000);  
    }
    else if(type==2){
      Serial.println("---------- Magnetic Calibration Init ----------");
      sendCommand(unlockMaster2,1);
      delay(500);
      sendCommand(magCalmode2,1);
      Serial.println("---------- Slowly rotate in 3 axis ----------");
      delay(5000);
      Serial.println("---------- May stop now :) ----------");
      sendCommand(setNormal2,1);
      delay(1000);
      sendCommand(saveConfig2,1);
      delay(1000);  
      }
  }
  

int rs485_receive(byte recv[], int num){
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
  
void readAcceleration(int type){
  if(type==1){
    sendCommand(readAcc1,0);
    Serial.println("Acceleration");
    delay(2000);
  
    if(rs485_receive(recData1, 11) != -1){
      for(int i = 0;i<11;i++){
        Serial.print(recData1[i],HEX);
        Serial.print(",");
        }
        Serial.println();
      }
     else{
      Serial.println("no resp");
      Serial.println();    
      } 
    printAccel(recData1);
    rs485.flush();
    delay(1000);
    }
    else if(type==2){
    sendCommand(readAcc2,0);
    Serial.println("Acceleration");
    delay(2000);
  
    if(rs485_receive(recData2, 11) != -1){
      for(int i = 0;i<11;i++){
        Serial.print(recData2[i],HEX);
        Serial.print(",");
        }
        Serial.println();
      }
     else{
      Serial.println("no resp");
      Serial.println();    
      } 
    printAccel(recData2);
    rs485.flush();
    delay(1000);
    }
  }

void readSensorAngle(int type){
  if(type==1){
    sendCommand(readAngle1,0);
    Serial.println("Angle");
    delay(2000);
  
    if(rs485_receive(recData1, 11) != -1){
      for(int i = 0;i<11;i++){
        Serial.print(recData1[i],HEX);
        Serial.print(",");
        }
        Serial.println();
      }
     else{
      Serial.println("no resp");
      Serial.println();    
      } 
    printAngle(recData1);
    rs485.flush();
    delay(1000);
    }
    else if(type==2){
    sendCommand(readAngle2,0);
    Serial.println("Angle");
    delay(2000);
  
    if(rs485_receive(recData2, 11) != -1){
      for(int i = 0;i<11;i++){
        Serial.print(recData2[i],HEX);
        Serial.print(",");
        }
        Serial.println();
      }
     else{
      Serial.println("no resp");
      Serial.println();    
      } 
    printAngle(recData2);
    rs485.flush();
    delay(1000);
    }
  }

void readAngularVelocity(int type){
  if(type==1){
    sendCommand(readAngVel1,0);
    Serial.println("Angular Velocity");
    delay(2000);
  
    if(rs485_receive(recData1, 11) != -1){
      for(int i = 0;i<11;i++){
        Serial.print(recData1[i],HEX);
        Serial.print(",");
        }
        Serial.println();
      }
     else{
      Serial.println("no resp");
      Serial.println();    
      } 
    printAngVel(recData1);
    rs485.flush();
    delay(1000);
    }
    else if(type==2){
    sendCommand(readAngVel2,0);
    Serial.println("Angular Velocity");
    delay(2000);
  
    if(rs485_receive(recData2, 11) != -1){
      for(int i = 0;i<11;i++){
        Serial.print(recData2[i],HEX);
        Serial.print(",");
        }
        Serial.println();
      }
     else{
      Serial.println("no resp");
      Serial.println();    
      } 
    printAngVel(recData2);
    rs485.flush();
    delay(1000);
    }
  }

void printAccel(byte rec[]){
  float data_x = ((rec[3]<<8)|rec[4])/(32768/16);
  float data_y = ((rec[5]<<8)|rec[6])/(32768/16);
  float data_z = ((rec[7]<<8)|rec[8])/(32768/16);
  Serial.print(data_x);Serial.print("   "); Serial.print(data_y);Serial.print("   "); Serial.println(data_z);
  }

void printAngle(byte rec[]){
  float data_x = ((rec[3]<<8)|rec[4])/(32768/180);
  float data_y = ((rec[5]<<8)|rec[6])/(32768/180);
  float data_z = ((rec[7]<<8)|rec[8])/(32768/180);
  Serial.print(data_x);Serial.print("   "); Serial.print(data_y);Serial.print("   "); Serial.println(data_z);
  }

void printAngVel(byte rec[]){
  float data_x = ((rec[3]<<8)|rec[4])/(32768/2000);
  float data_y = ((rec[5]<<8)|rec[6])/(32768/2000);
  float data_z = ((rec[7]<<8)|rec[8])/(32768/2000);
  Serial.print(data_x);Serial.print("   "); Serial.print(data_y);Serial.print("   "); Serial.println(data_z);
  }

byte* calculateDiff(byte arr[], int type){
  for(int i=0; i<3; i++){
    arr[i]=abs(((recData1[2*i+3]<<8)|recData1[4])-((recData2[2*i+4]<<8)|recData2[4]));
    if(type==1){
        arr[i] = arr[i]/(32768/16);
      }
    else if(type==2){
        arr[i] = arr[i]/(32768/180);
      }
    else if(type==3){
        arr[i] = arr[i]/(32768/2000);
      }
    }
  Serial.print(arr[0]);Serial.print("   "); Serial.print(arr[1]);Serial.print("   "); Serial.println(arr[2]);
  return arr;
  }

void setup() 
{ 
  Serial.begin(9600);
  rs485.begin(9600, SERIAL_8N1, RXD2, TXD2);
  rs485.flush();
  // wifi
  /*
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.println("...");
    }
  Serial.print("WiFi connected with IP : ");
  Serial.println(WiFi.localIP());
  */
  // ethernet
  /*
  WiFi.onEvent(WiFiEvent);
  ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE);
  delay(10000);
  */
  
  Serial.println("--------------- Serial Initiated ---------------");
  calibrateAcc(1);
  calibrateAcc(2);
  //calibrateMag(1);
  //calibrateMag(2);
  Serial.println("--------------- Calibration Done ---------------");
  
  if(rs485_receive(trashBuffer, 100) != -1){
    for(int i = 0;i<100;i++){
      Serial.print(trashBuffer[i],HEX);
      Serial.print(",");
      }
      Serial.println();
    }
}


void loop() 
{ 
  int trial = 1;
  if(!client.connect(host, port)){
    Serial.println("Connection to Host Failed");
    delay(1000);
    trial++;
    return;
    }
  rs485.flush();
  if(++flag==1){
    Serial.println("WT901C485 read");
    }
  
  readAcceleration(1);
  readAcceleration(2);
  calculateDiff(accDiff, 1);
  
  readSensorAngle(1);
  readSensorAngle(2);
  calculateDiff(angDiff, 2);
  
  readAngularVelocity(1);
  readAngularVelocity(2);
  calculateDiff(angvelDiff, 3);
  delay(500);
}
