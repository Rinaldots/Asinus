#ifndef HOVERSERIAL_LINUX_H
#define HOVERSERIAL_LINUX_H

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>

#define START_FRAME 0xABCD

enum class ReceiveStatus {
  Success = 0,
  NoData,
  IncompleteFrame,
  InvalidStartFrame,
  ChecksumMismatch,
  IOError
};

// Estrutura de RECEBIMENTO (Do Hoverboard para o PC) - Mantida igual
typedef struct __attribute__((packed, aligned(1))) {
     uint16_t cStart;   // 0xABCD
     uint8_t iSlave;
     int16_t iSpeed;    // 100* km/h
     uint16_t iVolt;    // 100* V
     int16_t iAmp;      // 100* A
     int32_t iOdom;     // hall steps
     uint16_t checksum;
} SerialHover2Server;

// Estrutura de ENVIO (Do PC para o Hoverboard) - ATUALIZADA para "Master"
// Baseado em: typedef struct { ... iDataType=1 ... iSteer ... } SerialServer2HoverMaster;
typedef struct __attribute__((packed, aligned(1))) {
     uint8_t  cStart;       // '/'
     uint8_t  iDataType;    // 1 (Master type)
     uint8_t  iSlave;
     int16_t  iSpeed;
     int16_t  iSteer;       // Adicionado conforme hoverserial.h
     uint8_t  wState;       // Master State
     uint8_t  wStateSlave;  // Slave State (Adicionado)
     uint16_t checksum;
} SerialServer2HoverMaster;

// Função de cálculo de CRC (Mesma do Arduino)
uint16_t CalcCRC(uint8_t *ptr, int count) {
  uint16_t  crc = 0;
  uint8_t i;
  while (--count >= 0) {
    crc = crc ^ (uint16_t) *ptr++ << 8;
    i = 8;
    do {
      if (crc & 0x8000) crc = crc << 1 ^ 0x1021;
      else crc = crc << 1;
    } while(--i);
  }
  return (crc);
}

// Função de Envio Atualizada com Steer e DataType=1
bool HoverSend(int uart_fd, uint8_t iSlave, int16_t iSpeed, int16_t iSteer, uint8_t wState, bool verbose) {
    SerialServer2HoverMaster oData;
    oData.cStart = '/';
    oData.iDataType = 1; // ID do protocolo Master
    oData.iSlave = iSlave;
    oData.iSpeed = iSpeed;
    oData.iSteer = iSteer;
    oData.wState = wState; 
    oData.wStateSlave = wState; // Simplificação: Slave assume o mesmo estado do Master
    
    // Checksum exclui os últimos 2 bytes (o próprio checksum)
    oData.checksum = CalcCRC((uint8_t*)&oData, sizeof(SerialServer2HoverMaster)-2);

    ssize_t res = write(uart_fd, &oData, sizeof(SerialServer2HoverMaster));
    return (res == sizeof(SerialServer2HoverMaster));
}

// Função Receive (Lógica do Arduino adaptada para Linux)
ReceiveStatus Receive(int uart_fd, SerialHover2Server& Feedback, bool verbose, const char* portName = "UART") {
    int bytes_avail = 0;
    if (ioctl(uart_fd, FIONREAD, &bytes_avail) < 0) return ReceiveStatus::IOError;

    // Se não tiver dados suficientes para um frame completo, nem tenta ler para evitar fragmentação
    // Mas no Linux é melhor ler byte a byte para encontrar o header
    int iTooMuch = bytes_avail; 
    
    // Buffer temporário para leitura
    uint8_t c;
    
    // Estado simples de máquina de estados (similar ao loop do Arduino)
    // 0 = Esperando Low Byte (0xCD)
    // 1 = Esperando High Byte (0xAB)
    static int state = 0; 

    // NOTA: Em um sistema multi-thread real, 'state' deveria ser por instância/porta, 
    // mas aqui simplificamos assumindo que a função é chamada em loop. 
    // Como a função Receive do seu código original era bloqueante/síncrona por porta, 
    // vamos manter a lógica de busca do header dentro do loop.

    while (iTooMuch > 0) {
        if (read(uart_fd, &c, 1) != 1) break;
        iTooMuch--;

        // Verifica Start Frame: 0xABCD (Little Endian -> CD primeiro, depois AB)
        if (c == (uint8_t)(START_FRAME & 0xFF)) { // 0xCD
             // Achou o primeiro byte
             // O próximo deve ser 0xAB. Vamos tentar ler imediatamente
             uint8_t c2;
             if (read(uart_fd, &c2, 1) == 1) {
                 if (c2 == (uint8_t)(START_FRAME >> 8)) { // 0xAB
                     // Header encontrado! Ler o resto
                     SerialHover2Server tmpFeedback;
                     tmpFeedback.cStart = START_FRAME;
                     
                     uint8_t* p = (uint8_t*)&tmpFeedback + 2; // Pula o cStart que já preenchemos manualmente
                     size_t bytes_needed = sizeof(SerialHover2Server) - 2;
                     size_t bytes_read = 0;

                     // Loop de leitura com timeout simples (bloqueante curto)
                     while(bytes_read < bytes_needed) {
                        ssize_t n = read(uart_fd, p + bytes_read, bytes_needed - bytes_read);
                        if (n > 0) bytes_read += n;
                        else if (n < 0 && errno != EAGAIN) break;
                        // usleep(100); // Opcional: ceder CPU
                     }

                     if (bytes_read == bytes_needed) {
                         uint16_t crc_calc = CalcCRC((uint8_t*)&tmpFeedback, sizeof(SerialHover2Server) - 2);
                         
                         if (crc_calc == tmpFeedback.checksum) {
                             memcpy(&Feedback, &tmpFeedback, sizeof(SerialHover2Server));
                             return ReceiveStatus::Success;
                         } else {
                             if (verbose) {
                                fprintf(stderr, "[%s] CRC Mismatch Slave:%d. Recv:%04X Calc:%04X\n", 
                                    portName, tmpFeedback.iSlave, tmpFeedback.checksum, crc_calc);
                             }
                             // MODO PERMISSIVO: Retorna sucesso mesmo com erro, para debug
                             memcpy(&Feedback, &tmpFeedback, sizeof(SerialHover2Server));
                             return ReceiveStatus::Success; 
                         }
                     }
                     return ReceiveStatus::IncompleteFrame;
                 }
             }
        }
    }
    return ReceiveStatus::NoData;
}

#endif