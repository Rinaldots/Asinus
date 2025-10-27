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

        if (command.startsWith("hover|")) {
            command.remove(0, 6);

            if (command.startsWith("all|")) {
                command.remove(0, 4);
                int numParsed = sscanf(command.c_str(), "%d|%d", &m_ispeedin, &m_istatein);
                if (numParsed == 2) {
                    for (size_t i = 0; i < motor_count_total; i++) {
                        int motorId = motors_all[i];
                        int speed = isMotorLeft(motorId) ? -m_ispeedin : m_ispeedin;
                        m_motor_speed[i] = speed;
                        m_slave_state[i] = m_istatein;
                    }
                } else {
                    Serial.println("The command doesn't meet the criteria");
                }
            } else if (command.startsWith("right|")) {
                command.remove(0, 6);
                int numParsed = sscanf(command.c_str(), "%d|%d", &m_ispeedin, &m_istatein);
                if (numParsed == 2) {
                    for (size_t i = 0; i < motor_count_right; i++) {
                        int motorId = motors_right[i];
                        m_motor_speed[motorId - m_motorOffset] = m_ispeedin;
                        m_slave_state[motorId - m_motorOffset] = m_istatein;
                    }
                } else {
                    Serial.println("The command doesn't meet the criteria");
                }
            } else if (command.startsWith("left|")) {
                command.remove(0, 5);
                int numParsed = sscanf(command.c_str(), "%d|%d", &m_ispeedin, &m_istatein);
                if (numParsed == 2) {
                    for (size_t i = 0; i < motor_count_left; i++) {
                        int motorId = motors_left[i];
                        m_motor_speed[motorId - m_motorOffset] = -m_ispeedin;
                        m_slave_state[motorId - m_motorOffset] = m_istatein;
                    }
                } else {
                    Serial.println("The command doesn't meet the criteria");
                }
            } else {
                int numParsed = sscanf(command.c_str(), "%d|%d|%d", &m_slaveidin, &m_ispeedin, &m_istatein);
                if (numParsed == 3) {
                    int motorId = m_slaveidin;
                    int speed = isMotorLeft(motorId) ? -m_ispeedin : m_ispeedin;
                    m_motor_speed[motorId - m_motorOffset] = speed;
                    m_slave_state[motorId - m_motorOffset] = m_istatein;
                } else {
                    Serial.println("The command doesn't meet the criteria");
                }
            }
        } else if (command.startsWith("stop")) {
            for (size_t i = 0; i < motor_count_total; i++) {
                m_motor_speed[i] = 0;
                m_slave_state[i] = m_istatein;
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
            Serial.print(m_motor_speed[i]);
            Serial.print(" Slave state is set to ");
            Serial.println(m_slave_state[i]);
        }
#endif
    }
}

void MotorManager::parseCommand(String command)
{
    // Minimal parser example: "S 1 100" -> set motor 1 speed 100
    // This is intentionally simple; the original project parses many commands.
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

#ifdef REMOTE_UARTBUS
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
#else
    // Non-REMOTE path not implemented in this manager; keep compatibility
    int iSteer = 0;
    int iSpeed = 0;
    HoverSend(m_port1, iSteer, iSpeed, m_wState, m_wState);
    m_iNext = iNow + SEND_MILLIS;
#endif
}

void MotorManager::receiveMotorFeedback()
{
    unsigned long iNow = millis();
    SerialHover2Server fb;
    bool bReceived1 = false, bReceived2 = false;

    while ((bReceived1 = Receive(m_port1, fb))) {
        //DEBUGT("millis", iNow - m_iLast);
        //DEBUGT("iSpeed", m_iSpeed);
        //HoverLog(fb);
        // Update telemetry: port1 always updates motor index 0 (ID will be 1)
        asinusManager.updateMotorByIndex(0, fb.iSpeed, fb.iOdom, fb.iVolt, iNow);
        m_feedback1 = fb;
        m_iLast = iNow;
    }

    while ((bReceived2 = Receive(m_port2, fb))) {
        //DEBUGT("millis", iNow - m_iLast);
        //DEBUGT("iSpeed", m_iSpeed);
        //HoverLog(fb);
        // Update telemetry: port2 always updates motor index 1 (ID will be 2)
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

