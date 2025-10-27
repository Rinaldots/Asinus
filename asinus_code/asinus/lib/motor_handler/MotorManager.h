#ifndef MOTORMANAGER_H
#define MOTORMANAGER_H

#include <Arduino.h>
#include "AsinusManager.h"
// Feature macros - must be set before including `hoverserial.h` so the
// correct communication structures and helpers are selected.
#define _DEBUG
//#define DEBUG_RX  // enable only when you need raw RX byte dumps
#define REMOTE_UARTBUS

#include "hoverserial.h"
// O intervalo de envio, definido no seu .ino original
#define SEND_MILLIS 50

class MotorManager {
public:
    // Construtor: Recebe as duas portas seriais dos motores
    MotorManager(HardwareSerial& port1, HardwareSerial& port2);

    // Inicializa as portas seriais
    void begin(long baud, int rx1, int tx1, int rx2, int tx2);

    // Função principal de atualização, deve ser chamada no loop()
    void update();

    // Imprime o status atual dos motores (se _DEBUG estiver ativado)
    void printStatus();

private:
    // Referências para as portas seriais
    HardwareSerial& m_port1;
    HardwareSerial& m_port2;

    // Buffers de feedback
    SerialHover2Server m_feedback1;
    SerialHover2Server m_feedback2;

    // Controle de tempo
    unsigned long m_nextSendTime;
    unsigned long m_lastFeedbackTime;
    // Timing/state variables mirroring original sketch
    unsigned long m_iLast;
    unsigned long m_iNext;
    unsigned long m_iTimeNextState;
    uint8_t m_wState;
    uint8_t m_iSendId;
    // optional command buffer / counters
    int m_count;
    String m_command;

    // Configuração dos motores (movida de TestSpeed.ino)
    static const size_t motor_count_total = 2;
    int motors_all[motor_count_total] = {1, 2};
    static const size_t motor_count_port1 = 1;
    int motors_port1[motor_count_port1] = {1};
    static const size_t motor_count_port2 = 1;
    int motors_port2[motor_count_port2] = {2};
    static const size_t motor_count_right = 1;
    int motors_right[motor_count_right] = {1};
    static const size_t motor_count_left = 1;
    int motors_left[motor_count_left] = {2};

    // Estado dos motores (movido de TestSpeed.ino)
    int m_motor_speed[motor_count_total];
    int m_slave_state[motor_count_total]; //
    int m_motorOffset;

    // Variáveis de parsing (movidas de TestSpeed.ino)
    int m_slaveidin;
    int m_iSpeed;
    int m_ispeedin;
    int m_istatein;

    // Funções auxiliares privadas
    void processSerialCommands();
    void parseCommand(String command);
    void sendMotorCommands();
    void receiveMotorFeedback();
    bool isMotorLeft(int motorId); //

    MotorTelemetry motor_1;
    MotorTelemetry motor_2;
};

#endif // MOTORMANAGER_H
