#include <Arduino.h>

#include "util.h"
#include "hoverserial.h"
#include "motor_handler.h"

#define _DEBUG
#define DEBUG_RX
#define REMOTE_UARTBUS

//#define MPU_Data    // uncomment if your hoverboard:config.h has active: #define SEND_IMU_DATA

#define SEND_MILLIS 50   // send commands to hoverboard every SEND_MILLIS milliseconds

const size_t motor_count_total = 2;
int motors_all[motor_count_total] = {1, 2};

const size_t motor_count_port1 = 1;
int motors_port1[motor_count_port1] = {1};
const size_t motor_count_port2 = 1;
int motors_port2[motor_count_port2] = {2};

const size_t motor_count_right = 1;
int motors_right[motor_count_right] = {1};
const size_t motor_count_left = 1;
int motors_left[motor_count_left] = {2};

int motor_speed[motor_count_total] = {0};
int slave_state[motor_count_total] = {0};
int motoroffset = motors_all[0] - 0;

int slaveidin;
int ispeedin;
int istatein;

HardwareSerial oSerialHover1(1);
HardwareSerial oSerialHover2(2);

SerialHover2Server oHoverFeedback1;
SerialHover2Server oHoverFeedback2;

// Identifica se o motor é do lado esquerdo
bool isMotorLeft(int motorId) {
  for (size_t i = 0; i < motor_count_left; i++) {
    if (motors_left[i] == motorId) return true;
  }
  return false;
};

void setup_motor()
{
  #ifdef _DEBUG
    Serial.begin(115200);
    Serial.println("Hello Hoverboard V2.x :-)");
  #endif
  
  HoverSetupEsp32(oSerialHover1, 19200, 21, 47);
  HoverSetupEsp32(oSerialHover2, 19200, 19, 20);
  
}

unsigned long iLast = 0;
unsigned long iNext = 0;
unsigned long iTimeNextState = 10;
uint8_t wState = 1;
uint8_t iSendId = 0;

void update_motor()
{
  unsigned long iNow = millis();

  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');

    if (command.startsWith("hover|")) {
      command.remove(0, 6);

      if (command.startsWith("all|")) {
        command.remove(0, 4);
        int numParsed = sscanf(command.c_str(), "%d|%d", &ispeedin, &istatein);
        if (numParsed == 2) {
          for (size_t i = 0; i < motor_count_total; i++) {
            int motorId = motors_all[i];
            int speed = isMotorLeft(motorId) ? -ispeedin : ispeedin;
            motor_speed[i] = speed;
            slave_state[i] = istatein;
          }
        } else {
          Serial.println("The command doesn't meet the criteria");
        }
      } else if (command.startsWith("right|")) {
        command.remove(0, 6);
        int numParsed = sscanf(command.c_str(), "%d|%d", &ispeedin, &istatein);
        if (numParsed == 2) {
          for (size_t i = 0; i < motor_count_right; i++) {
            int motorId = motors_right[i];
            motor_speed[motorId - motoroffset] = ispeedin;
            slave_state[motorId - motoroffset] = istatein;
          }
        } else {
          Serial.println("The command doesn't meet the criteria");
        }
      } else if (command.startsWith("left|")) {
        command.remove(0, 5);
        int numParsed = sscanf(command.c_str(), "%d|%d", &ispeedin, &istatein);
        if (numParsed == 2) {
          for (size_t i = 0; i < motor_count_left; i++) {
            int motorId = motors_left[i];
            motor_speed[motorId - motoroffset] = -ispeedin;
            slave_state[motorId - motoroffset] = istatein;
          }
        } else {
          Serial.println("The command doesn't meet the criteria");
        }
      } else {
        int numParsed = sscanf(command.c_str(), "%d|%d|%d", &slaveidin, &ispeedin, &istatein);
        if (numParsed == 3) {
          int motorId = slaveidin;
          int speed = isMotorLeft(motorId) ? -ispeedin : ispeedin;
          motor_speed[motorId - motoroffset] = speed;
          slave_state[motorId - motoroffset] = istatein;
        } else {
          Serial.println("The command doesn't meet the criteria");
        }
      }
    } else if (command.startsWith("stop")) {
      for (size_t i = 0; i < motor_count_total; i++) {
        motor_speed[i] = 0;
        slave_state[i] = istatein;
      }
    } else {
      Serial.println("Command not hover/stop");
      Serial.println(command);
    }

#ifdef _DEBUG
    for (size_t i = 0; i < motor_count_total; i++) {
      Serial.print("Motor ");
      Serial.print(motors_all[i]);
      Serial.print(" Speed is set to ");
      Serial.print(motor_speed[i]);
      Serial.print(" Slave state is set to ");
      Serial.println(slave_state[i]);
    }
#endif
  }

  if (iNow > iTimeNextState)
  {
    iTimeNextState = iNow + 3000;
    wState = wState << 1;
    if (wState == 64) wState = 1;  // remove this line to test Shutoff = 128
  }
  
  bool bReceived1, bReceived2;

  while ((bReceived1 = Receive(oSerialHover1, oHoverFeedback1)))
  {
    DEBUGT("millis", iNow - iLast);
    HoverLog(oHoverFeedback1);
    iLast = iNow;
  }

  while ((bReceived2 = Receive(oSerialHover2, oHoverFeedback2)))
  {
    DEBUGT("millis", iNow - iLast);
    HoverLog(oHoverFeedback2);
    iLast = iNow;
  }

  if (iNow > iNext)
  {
    #ifdef REMOTE_UARTBUS
    for (size_t i = 0; i < motor_count_port1; i++) {
      int motorId = motors_port1[i];
      HoverSend(oSerialHover1, motorId, motor_speed[motorId - motoroffset], slave_state[motorId - motoroffset]);
    }

    for (size_t i = 0; i < motor_count_port2; i++) {
      int motorId = motors_port2[i];
      HoverSend(oSerialHover2, motorId, motor_speed[motorId - motoroffset], slave_state[motorId - motoroffset]);
    }
    #endif
    iNext = iNow + SEND_MILLIS;
  }
}