# WITMOTION MotionSensor Firmware
---
JY901 applications on
- ESP32 - WT32ETH01 model with ethernet

provided code for arduino IDE doesn't seem to work so wrote a whole new C/C++ code with MODBUS protocol

---
Communcation Protocol : UART RS485
---
compatible with Wit-motion devices working with RS485 

(WT901C485, HWT901C485,SINDT485 ... etc)

* change modbus protocol in the manual is wrong! double-check crc before uploading
---
### ISSUE
sensing and posting two sensors with single ESP32 had time issues... about 3~4 packets a sec

--> divided into two processors
