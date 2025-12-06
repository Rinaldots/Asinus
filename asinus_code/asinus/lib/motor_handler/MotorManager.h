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
    struct SerialPortConfig {
        Stream& port;
        void (*beginFn)(Stream&, long, int, int) = nullptr;
    };

    // Construtor: Recebe as portas seriais dos motores, permitindo combinações de HardwareSerial e SoftwareSerial
    MotorManager(const SerialPortConfig& port1,
                 const SerialPortConfig& port2,
                 const SerialPortConfig& port3,
                 const SerialPortConfig& port4);

    // Inicializa as portas seriais
    void begin(long baud, int rx1, int tx1, int rx2, int tx2, int rx3, int tx3, int rx4, int tx4);

    // Função principal de atualização, deve ser chamada no loop()
    void update();

    // Imprime o status atual dos motores (se _DEBUG estiver ativado)
    void printStatus();

    void processSerialCommands();

    void sendMotorCommands();

private:
    // Referências para as portas seriais (Hardware ou Software)
    Stream& m_port1;
    Stream& m_port2;
    Stream& m_port3;
    Stream& m_port4;

    void (*m_port1Begin)(Stream&, long, int, int);
    void (*m_port2Begin)(Stream&, long, int, int);
    void (*m_port3Begin)(Stream&, long, int, int);
    void (*m_port4Begin)(Stream&, long, int, int);


    // Buffers de feedback
    SerialHover2Server m_feedback1;
    SerialHover2Server m_feedback2;
    SerialHover2Server m_feedback3;
    SerialHover2Server m_feedback4;

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
    static const size_t motor_count_total = 4;
    int motors_all[motor_count_total] = {1, 2, 3, 4};
    static const size_t motor_count_port1 = 1;
    int motors_port1[motor_count_port1] = {1};
    static const size_t motor_count_port2 = 1;
    int motors_port2[motor_count_port2] = {2};
    static const size_t motor_count_port3 = 1;
    int motors_port3[motor_count_port3] = {3};
    static const size_t motor_count_port4 = 1;
    int motors_port4[motor_count_port4] = {3};

    static const size_t motor_count_right = 2;
    int motors_right[motor_count_right] = {1, 3};
    static const size_t motor_count_left = 2;
    int motors_left[motor_count_left] = {2, 4};

    int m_motor_speed[motor_count_total];
    int m_slave_state[motor_count_total]; //
    int m_motorOffset;

    int m_slaveidin;
    int m_iSpeed;
    int m_ispeedin;
    int m_istatein;

    // Funções auxiliares privadas
    void parseCommand(String command);
    void receiveMotorFeedback();
    

    MotorTelemetry motor_1;
    MotorTelemetry motor_2;
};

#endif // MOTORMANAGER_H
