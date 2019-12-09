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
byte misfired[4] = { 0,0,0,0 };
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
      msg.set( 0, 0, 0, 0x7E0, 8, 0x03, 0x22, 0x29, 0x1E, 0x55, 0x55, 0x55, 0x55 );
      CAN.sendMsgBuf(msg.header, 0, msg.len, msg.data);
      msg.set( 0, 0, 0, 0x7E0, 8, 0x03, 0x22, 0x29, 0x1F, 0x55, 0x55, 0x55, 0x55 );
      CAN.sendMsgBuf(msg.header, 0, msg.len, msg.data);
      msg.set( 0, 0, 0, 0x7E0, 8, 0x03, 0x22, 0x29, 0x20, 0x55, 0x55, 0x55, 0x55 );
      CAN.sendMsgBuf(msg.header, 0, msg.len, msg.data);
      // RPM
      msg.set( 0, 0, 0, 0x7E0, 8, 0x03, 0x22, 0x20, 0x6F, 0x55, 0x55, 0x55, 0x55 );
      CAN.sendMsgBuf(msg.header, 0, msg.len, msg.data);

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

            if ( (id     == 0x7E8) &&
                 (buf[0] == 0x05) &&
                 (buf[1] == 0x62) &&

                 ( ( (buf[2] == 0x20) &&
                      ( ((buf[3] >= 0x0A) &&  // retards
                        (buf[3] <= 0x0D)) ||
                        (buf[3] == 0x6F)       // rpm
                      )
                   ) || (
                     (buf[2] == 0x29) &&
                     (buf[3] >= 0x1D) &&
                     (buf[3] <= 0x20)
                   )
                 )
               ) {
              
              if ((buf[4] != 0) ||
                  (buf[5] != 0)
                 ) {
               //msgBuffer[bufferPos].set(millis(),0,0,CAN.getCanId(),len,buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);
               //bufferPos++;
               //if (bufferPos == 64) {
               //   bufferPos = 0;
               //   writeSD();
               //}
/*
                 String str=String(time) + "\t" +
                  String(id, HEX) + "\t" +
                  String(len) + "\t" +
                  String(buf[0], HEX) + "\t" +
                  String(buf[1], HEX) + "\t" +
                  String(buf[2], HEX) + "\t" +
                  String(buf[3], HEX) + "\t" +
                  String(buf[4], HEX) + "\t" +
                  String(buf[5], HEX) + "\t" +
                  String(buf[6], HEX) + "\t" +
                  String(buf[7], HEX);
  */
                  unsigned long allSeconds=time/1000;
                  int runHours= allSeconds/3600;
                  int secsRemaining=allSeconds%3600;
                  int runMinutes=secsRemaining/60;
                  int runSeconds=secsRemaining%60;

                  String str = "";

                  if ((buf[2] == 0x20) && (buf[3] >= 0x0A) && (buf[3] <= 0x0D)) {         // retards
                    str = String(time) + "\t" + String(runHours) + ":" + String(runMinutes) + ":" + String(runSeconds) + "\t" + String(lastRPM) + "\t";
                    str = str + "ret\t";

                    switch (buf[3]) {
                      case 0xA:
                        str = str + "1\t";
                        break;
                      case 0xB:
                        str = str + "2\t";
                        break;
                      case 0xC:
                        str = str + "3\t";
                        break;
                      case 0xD:
                        str = str + "4\t";
                        break;
                    }

                    signed int retValueInt = buf[4]*256+buf[5];
                    float retValue = retValueInt / 100;
                    str = str + String(retValue);


                  } else if ((buf[2] == 0x20) && (buf[3] == 0x6F)) {         // rpm
                    lastRPM = buf[4]*256+buf[5];
                  } else if ((buf[2] == 0x29) && (buf[4] > 0)) {  // misfire
                    str = String(time) + "\t" + String(runHours) + ":" + String(runMinutes) + ":" + String(runSeconds) + "\t" + String(lastRPM) + "\t";

                    switch (buf[3]) {
                      case 0x1D:
                        if (buf[4] > misfired[0]) {
                          misfired[0] = buf[4];
                          str = str + "misfire\t1\t" + String(buf[4]);
                          tone(A2,1000,500);
                        }
                        break;
                      case 0x1E:
                        if (buf[4] > misfired[1]) {
                          misfired[1] = buf[4];
                          str = str + "misfire\t2\t" + String(buf[4]);
                          tone(A2,1000,500);
                        }
                        break;
                      case 0x1F:
                        if (buf[4] > misfired[2]) {
                          misfired[2] = buf[4];
                          str = str + "misfire\t3\t" + String(buf[4]);
                          tone(A2,1000,500);
                        }
                        break;
                      case 0x20:
                        if (buf[4] > misfired[3]) {
                          misfired[3] = buf[4];
                          str = str + "misfire\t4\t" + String(buf[4]);
                          tone(A2,1000,500);
                        }
                        break;
                      }
                    } else if ((buf[2] == 0x29) && (buf[4] == 0)) {  // misfire
                    switch (buf[3]) {
                      case 0x1D:
                        misfired[0] = 0;
                        break;
                      case 0x1E:
                        misfired[1] = 0;
                        break;
                      case 0x1F:
                        misfired[2] = 0;
                        break;
                      case 0x20:
                        misfired[3] = 0;
                        break;
                    }
                  }  

                  if (!dataFile) {
                    if (!SD.begin(SD_SPI_CS_PIN)) {
                      return;
                    }
                    dataFile  = SD.open("can-bus.txt", FILE_WRITE);
                    printed = false;
                  }

                  if (dataFile) {            
                    dataFile.println(str);
                    printed = true;
                  }
                } else if ((buf[4] == 0) &&
                          (buf[5] == 0) &&
                          (printed == true)
                         ) {
                  dataFile.close();
                  printed = false;
                }
            }
        }
    }
}
