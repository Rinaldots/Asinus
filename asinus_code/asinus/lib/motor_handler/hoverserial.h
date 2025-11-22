// hoverserial.h v20231224
/*
// Variables todo
uint8_t upperLEDMaster = 0;
uint8_t lowerLEDMaster = 0;
uint8_t mosfetOutMaster = 0;
uint8_t upperLEDSlave = 0;
uint8_t lowerLEDSlave = 0;
uint8_t mosfetOutSlave = 0;
uint8_t beepsBackwards = 0;
uint8_t activateWeakening = 0;
*/

// Simple include guard to avoid double-definitions when header is included multiple times
#ifndef HOVERSERIAL_H
#define HOVERSERIAL_H

#include "util.h"

// Allow baud and pin types to differ (e.g., baud: long, pins: int)
template <typename O, typename IB, typename IP>
void HoverSetupEsp32(O& oSerial, IB iBaud, IP gpio_RX, IP gpio_TX)
{
  // Starts the serial connection using the baud, protocol, GPIO RX, GPIO TX.
  // These are the GPIO numbers; not necessarily the pin number printed on the PCB.
  oSerial.begin(iBaud, SERIAL_8N1, gpio_RX, gpio_TX);
}

inline uint16_t CalcCRC(uint8_t *ptr, int count)
{
  uint16_t  crc;
  uint8_t i;
  crc = 0;
  while (--count >= 0)
  {
    crc = crc ^ (uint16_t) *ptr++ << 8;
    i = 8;
    do
    {
      if (crc & 0x8000)
      {
        crc = crc << 1 ^ 0x1021;
      }
      else
      {
        crc = crc << 1;
      }
    } while(--i);
  }
  return (crc);
}


#define START_FRAME         0xABCD       // [-] Start frme definition for reliable serial communication

#ifdef REMOTE_UARTBUS
  typedef struct __attribute__((packed, aligned(1))) {
     uint16_t cStart = START_FRAME;    //  = '/';
     uint8_t iSlave;    //  the slave id this message is sent from
     int16_t iSpeed;   // 100* km/h
     uint16_t iVolt;    // 100* V
     int16_t iAmp;   // 100* A
     int32_t iOdom;    // hall steps
     uint16_t checksum;
  } SerialHover2Server;

  //typedef struct{   // new version
  //   uint16_t cStart = START_FRAME;   // new version
  typedef struct __attribute__((packed, aligned(1))) {  // old version
     uint8_t  cStart = '/';
     uint8_t  iDataType = 0;    //  unique id for this data struct
     uint8_t  iSlave;       //  contains the slave id this message is intended for
     int16_t  iSpeed = 0;
     uint8_t  wState = 0;   // 1=ledGreen, 2=ledOrange, 4=ledRed, 8=ledUp, 16=ledDown   , 32=Battery3Led, 64=Disable, 128=ShutOff
     uint16_t checksum;
  } SerialServer2Hover;
  typedef struct __attribute__((packed, aligned(1))) {  // old version
     uint8_t  cStart = '/';
     uint8_t  iDataType = 1;    //  unique id for this data struct
     uint8_t  iSlave;       //  contains the slave id this message is intended for
     int16_t  iSpeed = 0;
     int16_t  iSteer = 0;
     uint8_t  wState = 0;   // 1=ledGreen, 2=ledOrange, 4=ledRed, 8=ledUp, 16=ledDown   , 32=Battery3Led, 64=Disable, 128=ShutOff
     uint8_t  wStateSlave = 0;   // 1=ledGreen, 2=ledOrange, 4=ledRed, 8=ledUp, 16=ledDown   , 32=Battery3Led, 64=Disable, 128=ShutOff
     uint16_t checksum;
  } SerialServer2HoverMaster;

  typedef struct __attribute__((packed, aligned(1))) {  //
    uint8_t  cStart = '/';
    uint8_t  iDataType = 2;    //  
    uint8_t  iSlave;       //  contains the slave id this message is intended for
      float fBattFull;
      float fBattEmpty;
      uint8_t iDriveMode; // MM32 0=COM_VOLTAGE, 1=COM_SPEED, 2=SINE_VOLTAGE, 3=SINE_SPEED
    uint8_t iSlaveNew; // if >= 0 contains the new slave id saved in eeprom
    uint16_t checksum;
  } SerialServer2HoverConfig;

  template <typename O,typename D> void HoverSendData(O& oSerial, D& oData)
  {
    oData.checksum = CalcCRC((uint8_t*)&oData, sizeof(oData)-2); // first bytes except crc
    oSerial.write((uint8_t*) &oData, sizeof(oData));
    //DEBUGN(oData.iSlave, sizeof(oData));
  }

  template <typename O> void HoverSendConfig(O& oSerial, uint8_t iSlave, uint8_t iDriveMode, float fBattFull=0.0, float fBattEmpty=-100.0)
  {
    //DEBUGT("iSteer",iSteer
    SerialServer2HoverConfig oData;
    oData.iSlave    = iSlave;
    oData.fBattFull = fBattFull;
    oData.fBattEmpty = fBattEmpty;
    oData.iDriveMode = iDriveMode;
    oData.iSlaveNew = -1; // no change
    oData.checksum = CalcCRC((uint8_t*)&oData, sizeof(SerialServer2HoverConfig)-2); // first bytes except crc
    oSerial.write((uint8_t*) &oData, sizeof(SerialServer2HoverConfig));
  }


  template <typename O,typename I> void HoverSend(O& oSerial, uint8_t iSlave, I iSpeed, uint8_t  wState=32)
  {
    //DEBUGT("iSteer",iSteer);DEBUGN("iSpeed",iSpeed);
    SerialServer2Hover oData;
    oData.iSlave    = iSlave;
    oData.iSpeed    = (int16_t)iSpeed;
    oData.wState    = wState;
    oData.checksum = CalcCRC((uint8_t*)&oData, sizeof(SerialServer2Hover)-2); // first bytes except crc
    oSerial.write((uint8_t*) &oData, sizeof(SerialServer2Hover));
    //DebugOut((uint8_t*) &oData, sizeof(oData));
  }

  inline void HoverLog(SerialHover2Server& oData)
  {
    DEBUGT("iSlave",oData.iSlave);
    DEBUGT("iOdom",oData.iOdom);
    DEBUGT("\tiSpeed",(float)oData.iSpeed/100.0);
    DEBUGT("\tiAmp",(float)oData.iAmp/100.0);
    DEBUGN("\tiVolt",(float)oData.iVolt/100.0);


  }

  inline void HoverUpdate(SerialHover2Server& oData)
  {
    asinusManager.updateMotorByIndex(oData.iSlave, oData.iOdom, oData.iSpeed, oData.iVolt, millis());
  } 


inline void DebugOut(uint8_t aBuffer[], uint8_t iSize)
{
  for (int i=0; i<iSize; i++)
  {
    uint8_t c = aBuffer[i];
    Serial.print((c < 16) ? " 0" : " ");Serial.print(c,HEX);
  }
  Serial.println();
}

//boolean Receive(Serial& oSerial, SerialFeedback& Feedback)
template <typename O,typename OF> boolean Receive(O& oSerial, OF& Feedback)
{
  int iTooMuch = oSerial.available() - sizeof(SerialHover2Server) + 1;
  int8_t bFirst = 1;
  while (iTooMuch >= bFirst )
  {
    byte c = oSerial.read();  // Read the incoming byte
    iTooMuch--;

    if (bFirst) // test first START byte
    {
      if (c == (byte)START_FRAME) //if (c == 0xCD)
      {
        bFirst = 0;
      }
    }
    else  // test second START byte
    {
      if (c == START_FRAME >>8 ) //if (c == 0xAB)
      {
        //DEBUGT(" avail",oSerial.available())
        SerialHover2Server tmpFeedback;
        byte* p = (byte *)&tmpFeedback+2; // start word already read
        for (int i = sizeof(SerialHover2Server); i>2; i--)
          *p++    = oSerial.read();

        //while(oSerial.available()) oSerial.read();
        #ifdef DEBUG_RX
          //Serial.print(" -> ");
          //HoverLog(tmpFeedback);
        #endif

        uint16_t checksum = CalcCRC((byte *)&tmpFeedback, sizeof(SerialHover2Server)-2);
        if (checksum == tmpFeedback.checksum)
        {
            memcpy(&Feedback, &tmpFeedback, sizeof(SerialHover2Server));
            #ifdef DEBUG_RX
              Serial.println(" :-)");
            #endif
            return true;
        }
        return false;
      }
      if (c != (byte)START_FRAME) //if (c != 0xCD)
        bFirst = 1;
    }
  }
  return false;
}

#endif // REMOTE_UARTBUS

#endif // HOVERSERIAL_H