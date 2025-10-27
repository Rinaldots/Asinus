#include "RelayManager.h"

RelayManager::RelayManager(uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4, uint8_t pin5, uint8_t pin6, uint8_t pin7, uint8_t pin8, bool active_low)
    : _pin{pin1, pin2, pin3, pin4, pin5, pin6, pin7, pin8}, _active_low{active_low, active_low, active_low, active_low, active_low, active_low, active_low, active_low}, _state{false, false, false, false, false, false, false, false} {
}

void RelayManager::begin() {
    for (size_t i = 0; i < _pin.size(); ++i) {
        pinMode(_pin[i], OUTPUT);
    }

    // Garante que o relé comece desligado
    for (size_t i = 0; i < _pin.size(); ++i) {
        // write the inactive level
        digitalWrite(_pin[i], _active_low[i] ? HIGH : LOW);
        _state[i] = false;
    }
}

void RelayManager::turnOn(uint8_t relayNumber) {
    if (relayNumber >= _pin.size()) return;
    digitalWrite(_pin[relayNumber], _active_low[relayNumber] ? LOW : HIGH);
    _state[relayNumber] = true;
}

void RelayManager::turnOff(uint8_t relayNumber) {
    if (relayNumber >= _pin.size()) return;
    digitalWrite(_pin[relayNumber], _active_low[relayNumber] ? HIGH : LOW);
    _state[relayNumber] = false;
}

void RelayManager::toggle(uint8_t relayNumber) {
    if (isOn(relayNumber)) {
        turnOff(relayNumber);
    } else {
        turnOn(relayNumber);
    }
}

bool RelayManager::isOn(uint8_t relayNumber) {
    if (relayNumber >= _state.size()) return false;
    return _state[relayNumber];
}
