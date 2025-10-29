// MotorManager.cpp
// Implements the MotorManager declared in MotorManager.h

#include "MotorManager.h"
#include "hoverserial.h"
#include <Arduino.h>

#ifdef DEBUG_RX
// Single definition of the debugging timestamp used by hoverserial helpers
unsigned long iLastRx = 0;
#endif

// Constructor
MotorManager::MotorManager(HardwareSerial& port1, HardwareSerial& port2)
  : m_port1(port1), m_port2(port2), m_nextSendTime(0), m_lastFeedbackTime(0), m_motorOffset(0),
    m_slaveidin(0), m_iSpeed(0), m_ispeedin(0), m_istatein(0),
    m_iLast(0), m_iNext(0), m_iTimeNextState(10), m_wState(1), m_iSendId(0), m_count(0), m_command("")
{
    for (size_t i = 0; i < motor_count_total; ++i) {
        m_motor_speed[i] = 0;
        m_slave_state[i] = 0;
    }
    // replicate original motoroffset calculation (motors_all[0] - 0)
    m_motorOffset = motors_all[0] - 0;
}

// Initialize serial ports used for motors
void MotorManager::begin(long baud, int rx1, int tx1, int rx2, int tx2)
{
    // Use ESP32 helper from hoverserial.h to start serial with custom pins
    HoverSetupEsp32(m_port1, baud, rx1, tx1);
    HoverSetupEsp32(m_port2, baud, rx2, tx2);
    m_nextSendTime = millis() + SEND_MILLIS;
}

// Main update loop — lightweight orchestration
void MotorManager::update()
{
    processSerialCommands();
    receiveMotorFeedback();

    unsigned long now = millis();
    if ((long)(now - m_nextSendTime) >= 0) {
        sendMotorCommands();
        m_nextSendTime = now + SEND_MILLIS;
    }
}

void MotorManager::printStatus()
{
#ifdef _DEBUG
    Serial.println("MotorManager status:");
    Serial.print("nextSend:"); Serial.println(m_nextSendTime);
#endif
}

// --- Private helpers (minimal implementations so symbols are defined) ---
void MotorManager::processSerialCommands()
{
    // Mirror the parsing logic from TestSpeed.ino
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');

        if (command.startsWith("h|")) {
            command.remove(0, 2);
            int numParsed = sscanf(command.c_str(), "%d|%d|%d", &m_slaveidin, &m_ispeedin, &m_istatein);
            if (numParsed == 3) {
                int motorId = m_slaveidin;
                int speed = isMotorLeft(motorId) ? -m_ispeedin : m_ispeedin;
                m_motor_speed[motorId - m_motorOffset] = speed;
                m_slave_state[motorId - m_motorOffset] = m_istatein;
            } else {
                
            }
        }
    }
}

void MotorManager::parseCommand(String command)
{
    command.trim();
    if (command.length() == 0) return;

    // Example: S <slaveId> <speed>
    if (command.charAt(0) == 'S' || command.charAt(0) == 's') {
        int sid = -1;
        int spd = 0;
        if (sscanf(command.c_str()+1, "%d %d", &sid, &spd) >= 1) {
            if (sid >= 1 && sid <= (int)motor_count_total) {
                m_motor_speed[sid-1] = spd;
            }
        }
    }
}

void MotorManager::sendMotorCommands()
{
    unsigned long iNow = millis();
    for (size_t i = 0; i < motor_count_port1; i++) {
        int motorId = motors_port1[i];
        int speed = m_motor_speed[motorId - m_motorOffset];
        HoverSend(m_port1, (uint8_t)motorId, speed, (uint8_t)m_slave_state[motorId - m_motorOffset]);
    }

    for (size_t i = 0; i < motor_count_port2; i++) {
        int motorId = motors_port2[i];
        int speed = m_motor_speed[motorId - m_motorOffset];
        HoverSend(m_port2, (uint8_t)motorId, speed, (uint8_t)m_slave_state[motorId - m_motorOffset]);
    }
    m_iNext = iNow + SEND_MILLIS / 2;
}

void MotorManager::receiveMotorFeedback()
{
    unsigned long iNow = millis();
    SerialHover2Server fb;
    bool bReceived1 = false, bReceived2 = false;

    while ((bReceived1 = Receive(m_port1, fb))) {
        asinusManager.updateMotorByIndex(0, fb.iSpeed, fb.iOdom, fb.iVolt, iNow);
        m_feedback1 = fb;
        m_iLast = iNow;
    }

    while ((bReceived2 = Receive(m_port2, fb))) {
        asinusManager.updateMotorByIndex(1, fb.iSpeed, fb.iOdom, fb.iVolt, iNow);
        m_feedback2 = fb;
        m_iLast = iNow;
    }
}

bool MotorManager::isMotorLeft(int motorId)
{
    for (size_t i = 0; i < motor_count_left; ++i) {
        if (motors_left[i] == motorId) return true;
    }
    return false;
}

