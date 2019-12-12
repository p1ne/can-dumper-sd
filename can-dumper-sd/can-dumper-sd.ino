#include <Arduino.h>
#include <SPI.h>
#include <avr/pgmspace.h>
#include "mcp_can.h"
#include "mcp_can_dfs.h"
#include <SD.h>
#include "CANMessage.h"

const int CAN_SPI_CS_PIN = 10;
const int SD_SPI_CS_PIN = 9;

MCP_CAN CAN(CAN_SPI_CS_PIN);
unsigned char flagRecv = 0;
unsigned char len = 0;
unsigned char buf[8];
String str;
File dataFile;
bool printed = false;
int id;
unsigned long time;
unsigned long prevTime;
int misfired[4] = { 0,0,0,0 };
int lastRPM = 0;

CANMessage msg;

//CANMessage msgBuffer[64];
//int bufferPos = 0;

void attachCAN()
{
//  #if defined(__AVR_ATmega32U4__) // Arduino Pro Micro
   pinMode(7, INPUT);
   attachInterrupt(digitalPinToInterrupt(7), MCP2515_ISR, FALLING); // start interrupt
//  #else // Other Arduinos (Nano in my case)
//    pinMode(2, INPUT);
//    attachInterrupt(digitalPinToInterrupt(2), MCP2515_ISR, FALLING); // start interrupt
//  #endif
}

void setup()
{
  Serial.begin(9600);
  delay(2000);

  for (int i=0;i<4;i++) {
    misfired[i] = 0;
  }

  pinMode(A2, OUTPUT);

  Serial.println("Init");
START_INIT:
  if(CAN_OK == CAN.begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ)) {
    Serial.println(F("CAN ok!"));
  } else {
    Serial.println(F("CAN fail"));
    delay(100);
    goto START_INIT;
  }

/*  if (!SD.begin(SD_SPI_CS_PIN)) {
    Serial.println("Card failed, or not present");
    return;
  }
*/

  Serial.println("card initialized.");
  File dataFile = SD.open("can-bus.txt", FILE_WRITE);
  if (dataFile) {            
    dataFile.println("---");
  }
  dataFile.close();   

  attachCAN();

  CAN.setMode(MCP_LOOPBACK);

  CAN.init_Mask(0, CAN_STDID, 0x07FF0000);   // there are 2 mask in mcp2515, you need to set both of them
  CAN.init_Mask(1, CAN_STDID, 0x07FF0000);
  for (int i=0;i<5;i++)
    CAN.init_Filt(i, CAN_STDID, 0x07e80000);   // Reply data

  CAN.setMode(MCP_NORMAL);

//  bufferPos = 0;
  str = "";
  time = millis();
  prevTime = time;
}

void MCP2515_ISR()
{
  flagRecv = 1;
}

/*
void writeSD()
{
//  String str = "";
  str = "";

  for (int i=0;i<64;i++) {
    str=str+String(msgBuffer[i].started) + "\t" +
                String(msgBuffer[i].header, HEX) + "\t" +
                String(msgBuffer[i].len) + "\t" +
                String(msgBuffer[i].data[0], HEX) + "\t" +
                String(msgBuffer[i].data[1], HEX) + "\t" +
                String(msgBuffer[i].data[2], HEX) + "\t" +
                String(msgBuffer[i].data[3], HEX) + "\t" +
                String(msgBuffer[i].data[4], HEX) + "\t" +
                String(msgBuffer[i].data[5], HEX) + "\t" +
                String(msgBuffer[i].data[6], HEX) + "\t" +
                String(msgBuffer[i].data[7], HEX) + "\r\n";
  }

  File dataFile = SD.open("can-bus.txt", FILE_WRITE);
  if (dataFile) {            
      dataFile.print(str);
      dataFile.close();
  }
}
*/

void loop()
{
  time = millis();

  if (time - prevTime > 200) {
    // misfires
    msg.set( 0, 0, 0, 0x7E0, 8, 0x03, 0x22, 0x29, 0x1D, 0x55, 0x55, 0x55, 0x55 );
    CAN.sendMsgBuf(msg.header, 0, msg.len, msg.data);
    delay(20);
    msg.set( 0, 0, 0, 0x7E0, 8, 0x03, 0x22, 0x29, 0x1E, 0x55, 0x55, 0x55, 0x55 );
    CAN.sendMsgBuf(msg.header, 0, msg.len, msg.data);
    delay(20);
    msg.set( 0, 0, 0, 0x7E0, 8, 0x03, 0x22, 0x29, 0x1F, 0x55, 0x55, 0x55, 0x55 );
    CAN.sendMsgBuf(msg.header, 0, msg.len, msg.data);
    delay(20);
    msg.set( 0, 0, 0, 0x7E0, 8, 0x03, 0x22, 0x29, 0x20, 0x55, 0x55, 0x55, 0x55 );
    CAN.sendMsgBuf(msg.header, 0, msg.len, msg.data);
    delay(20);
    // RPM
    msg.set( 0, 0, 0, 0x7E0, 8, 0x03, 0x22, 0x20, 0x6F, 0x55, 0x55, 0x55, 0x55 );
    CAN.sendMsgBuf(msg.header, 0, msg.len, msg.data);
    delay(20);

    prevTime = time;
  }

  if(flagRecv)
  {
    flagRecv = 0;
    while (CAN_MSGAVAIL == CAN.checkReceive())
    {
      // read data,  len: data length, buf: data buf
      CAN.readMsgBuf(&len, buf);
      id = CAN.getCanId();

      String str = "";

      if ((id == 0x7E8) && (buf[0] == 0x05) && (buf[1] == 0x62) && (buf[2] == 0x20) && (buf[3] == 0x6F)) { // current RPM
            lastRPM = buf[4]*256+buf[5];
      }

      if (  ((id == 0x7E8) && (buf[0] == 0x05) && (buf[1] == 0x62) && (buf[2] == 0x20) && (buf[3] >= 0x0A) && (buf[3] <= 0x0D)) ||
            ((id == 0x7E8) && (buf[0] == 0x05) && (buf[1] == 0x62) && (buf[2] == 0x29) && (buf[3] >= 0x1D) && (buf[3] <= 0x20))
          ) {
        
        int canValue = buf[4]*256+buf[5];

        if (canValue != 0) {

          if ((buf[2] == 0x20) && (buf[3] >= 0x0A) && (buf[3] <= 0x0D)) {         // retards
            str = str + "ret\t" + String(buf[3]-0x9) + "\t" + String((float)(signed int)(canValue) / 100.0);;

          } else if ((buf[2] == 0x29) && (canValue > 0)) {  // misfire
            int misfires = canValue;

            if (misfires > misfired[buf[3]-0x1D]) {
              misfired[buf[3]-0x1D] = misfires;
              str = str + "misfire\t" + String(buf[3]-0x1C) + "\t" + String(misfires);
              tone(A2,1000,500);
            }
          } else if ((buf[2] == 0x29) && (canValue == 0)) {  // misfire
            misfired[buf[3]-0x1D] = 0;
          }  

          if (str != "") {
            if (!dataFile) {
              if (!SD.begin(SD_SPI_CS_PIN)) {
                return;
              }
              dataFile  = SD.open("can-bus.txt", FILE_WRITE);
              printed = false;
            }

            if (dataFile) {            
              unsigned long allSeconds=time/1000;
              int runHours= allSeconds/3600;
              int secsRemaining=allSeconds%3600;
              int runMinutes=secsRemaining/60;
              int runSeconds=secsRemaining%60;
              str = String(time) + "\t" + String(runHours) + ":" + String(runMinutes) + ":" + String(runSeconds) + "\t" + String(lastRPM) + "\t" + str;
              dataFile.println(str);
              printed = true;
            }
          }

        } else if ((canValue == 0) && (printed)) {
          if (dataFile) {
            dataFile.close();
          }
          printed = false;
        }
      }
    }
  }
}
