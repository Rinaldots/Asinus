// Async serial reader using Boost.Asio
// Provides a simple callback-based API that runs an io_context in a background thread.

#pragma once

#include <boost/asio.hpp>
#include <thread>
#include <functional>
#include <vector>
#include <string>
#include <atomic>
#include <memory>
#include <mutex>

namespace asinus_hardware_interface
{
class AsyncSerialReader
{
public:
    using DataCallback = std::function<void(const std::vector<uint8_t> &)>;

    AsyncSerialReader(const std::string &device, unsigned int baud_rate = 115200);
    ~AsyncSerialReader();

    // Non-copyable
    AsyncSerialReader(const AsyncSerialReader &) = delete;
    AsyncSerialReader &operator=(const AsyncSerialReader &) = delete;

    // Start async reading (returns immediately). Returns true if port opened successfully.
    bool start();

    // Stop reading and join thread
    void stop();

    // Set callback invoked when bytes are received
    void set_callback(DataCallback cb);

    // Write bytes (thread-safe post to io_context)
    void write(const std::vector<uint8_t> &data);

    bool is_running() const { return running_.load(); }

private:
    void do_read();
    void handle_read(const boost::system::error_code &ec, std::size_t bytes_transferred);
    void do_write();

    std::string device_;
    unsigned int baud_;

    boost::asio::io_context io_context_;
    std::unique_ptr<boost::asio::serial_port> serial_; // created on start
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;

    std::thread io_thread_;
    std::vector<uint8_t> read_buffer_;

    std::mutex write_mutex_;
    std::vector<uint8_t> write_queue_;

    DataCallback callback_;
    std::atomic<bool> running_{false};
};

} // namespace asinus_hardware_interface
