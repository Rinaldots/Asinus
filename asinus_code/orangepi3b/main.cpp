#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <sstream>
#include <cstdint>
#include "hoverserial_linux.h"

// --- Configurações ---
#define SEND_INTERVAL_MS 100 // Envia comandos a cada 100ms

struct MotorPort {
    std::string port_name;
    int fd;
    std::vector<int> slave_ids;
    speed_t baud_rate;
};

// Configuração das portas
std::vector<MotorPort> ports = {
    {"/dev/ttyS2", -1, {1}, B19200},
    {"/dev/ttyS3", -1, {2}, B19200},
    {"/dev/ttyS7", -1, {3}, B19200},
    {"/dev/ttyS9", -1, {4}, B19200}
};

// Estrutura de estado atualizada para incluir STEER (Direção)
struct MotorState {
    int speed = 0;
    int steer = 0; // Novo campo
    int state = 0; // wState (LEDs/Modo)
};
std::vector<MotorState> global_motor_states(10); // Suporta até ID 9

std::mutex data_mutex;
std::atomic<bool> running(true);
std::atomic<bool> debug_mode(false);

std::atomic<uint64_t> stats_tx_ok(0);
std::atomic<uint64_t> stats_tx_fail(0);
std::atomic<uint64_t> stats_rx_ok(0);
std::atomic<uint64_t> stats_rx_fail(0);

void print_debug_status();
void log_feedback(const std::string& port, const SerialHover2Server& feedback);

unsigned long millis() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

int setup_uart(const std::string& portName, speed_t baudRate) {
    int uart_fd = open(portName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart_fd == -1) {
        perror(("Erro ao abrir " + portName).c_str());
        return -1;
    }

    struct termios options;
    tcgetattr(uart_fd, &options);
    
    cfsetispeed(&options, baudRate);
    cfsetospeed(&options, baudRate);
    
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;

    tcsetattr(uart_fd, TCSANOW, &options);
    return uart_fd;
}

void communication_thread() {
    unsigned long last_send = 0;

    while (running) {
        unsigned long now = millis();
        bool verbose = debug_mode.load();

        // --- ENVIO ---
        if (now - last_send > SEND_INTERVAL_MS) {
            std::lock_guard<std::mutex> lock(data_mutex);
            
            for (auto& port : ports) {
                if (port.fd < 0) continue; 

                for (int slaveId : port.slave_ids) {
                    if (slaveId < 0 || slaveId >= (int)global_motor_states.size()) continue;

                    int speed = global_motor_states[slaveId].speed;
                    int steer = global_motor_states[slaveId].steer;
                    int state = global_motor_states[slaveId].state;

                    // Chama HoverSend com os novos parâmetros
                    if (HoverSend(port.fd, slaveId, speed, steer, state, verbose)) {
                        stats_tx_ok.fetch_add(1);
                    } else {
                        stats_tx_fail.fetch_add(1);
                        if (verbose) std::cerr << "[TX Fail] " << port.port_name << std::endl;
                    }
                }
            }
            last_send = now;
        }

        // --- RECEBIMENTO ---
        for (auto& port : ports) {
            if (port.fd < 0) continue;

            SerialHover2Server feedback;
            ReceiveStatus status = Receive(port.fd, feedback, verbose, port.port_name.c_str());
            
            if (status == ReceiveStatus::Success) {
                stats_rx_ok.fetch_add(1);
                if (verbose) log_feedback(port.port_name, feedback);
            } 
            else if (status != ReceiveStatus::NoData) {
                stats_rx_fail.fetch_add(1);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5)); 
    }
}

void parse_command(std::string cmd) {
    if (cmd == "debug on") { debug_mode = true; return; }
    if (cmd == "debug off") { debug_mode = false; return; }
    if (cmd == "stats") { print_debug_status(); return; }
    
    if (cmd == "stop") {
        std::lock_guard<std::mutex> lock(data_mutex);
        for(auto& m : global_motor_states) {
             m.speed = 0; 
             m.steer = 0;
        }
        std::cout << "Parando todos motores." << std::endl;
        return;
    }

    if (cmd.find("hover|") != 0) return;
    cmd.erase(0, 6); 

    int speed, steer, state, slave_id;
    std::lock_guard<std::mutex> lock(data_mutex);

    // Sintaxe: hover|all|speed|steer|state
    if (cmd.find("all|") == 0) {
        if(sscanf(cmd.c_str(), "all|%d|%d|%d", &speed, &steer, &state) == 3) {
            for(auto& m : global_motor_states) {
                m.speed = speed;
                m.steer = steer;
                m.state = state;
            }
            printf("Comando ALL: Speed=%d Steer=%d State=%d\n", speed, steer, state);
        }
    }
    // Sintaxe: hover|id|speed|steer|state
    else if (sscanf(cmd.c_str(), "%d|%d|%d|%d", &slave_id, &speed, &steer, &state) == 4) {
        if (slave_id >= 0 && slave_id < (int)global_motor_states.size()) {
            global_motor_states[slave_id].speed = speed;
            global_motor_states[slave_id].steer = steer;
            global_motor_states[slave_id].state = state;
            printf("Comando ID %d: Speed=%d Steer=%d State=%d\n", slave_id, speed, steer, state);
        }
    }
}

int main() {
    std::cout << "=== Controlador Hoverboard (Master Protocol) ===" << std::endl;
    std::cout << "Comandos: hover|id|speed|steer|state" << std::endl;

    for (auto& port : ports) {
        port.fd = setup_uart(port.port_name, port.baud_rate);
        if (port.fd >= 0) {
            std::cout << "Porta aberta: " << port.port_name << " (FD: " << port.fd << ")" << std::endl;
        } else {
            std::cerr << "FALHA ao abrir: " << port.port_name << std::endl;
        }
    }

    std::thread comms(communication_thread);

    std::string input_line;
    while (std::getline(std::cin, input_line)) {
        if (input_line == "exit") {
            running = false;
            break;
        }
        parse_command(input_line);
    }

    if (comms.joinable()) comms.join();
    
    for (auto& port : ports) {
        if (port.fd >= 0) close(port.fd);
    }
    
    return 0;
}

void log_feedback(const std::string& port, const SerialHover2Server& feedback) {
    // Exibe apenas a cada X mensagens para não floodar o terminal se estiver muito rápido
    // Ou exibe sempre se for crítico. Aqui exibe sempre se debug=on.
    std::cout << "[" << port << "] Slave:" << (int)feedback.iSlave
              << " Spd:" << feedback.iSpeed
              << " Volt:" << (float)feedback.iVolt/100.0
              << " Amp:" << (float)feedback.iAmp/100.0 
              << " Odom:" << feedback.iOdom << std::endl;
}

void print_debug_status() {
    std::cout << "[STATS] TX OK: " << stats_tx_ok << " | TX Fail: " << stats_tx_fail << std::endl;
    std::cout << "[STATS] RX OK: " << stats_rx_ok << " | RX Fail: " << stats_rx_fail << std::endl;
}