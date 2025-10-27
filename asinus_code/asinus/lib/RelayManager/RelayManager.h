#ifndef RELAY_MANAGER_H
#define RELAY_MANAGER_H

#include <Arduino.h>
#include <array>

class RelayManager {
public:
    // Construtor: pinos dos relés e se são ativos em nível baixo
    RelayManager(uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4, uint8_t pin5, uint8_t pin6, uint8_t pin7, uint8_t pin8,bool active_low = false);

    // Inicializa o pino do relé
    void begin();

    // Liga o relé
    void turnOn(uint8_t relayNumber);

    // Desliga o relé
    void turnOff(uint8_t relayNumber);

    // Alterna o estado do relé
    void toggle(uint8_t relayNumber);

    // Retorna true se o relé estiver ligado
    bool isOn(uint8_t relayNumber);

private:
    std::array<uint8_t, 8> _pin {0,0,0,0,0,0,0,0};
    std::array<bool, 8> _active_low {false,false,false,false,false,false,false,false};
    std::array<bool, 8> _state {false,false,false,false,false,false,false,false};
};

#endif // RELAY_MANAGER_H
