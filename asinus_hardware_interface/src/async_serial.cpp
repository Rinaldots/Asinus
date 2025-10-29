#include "asinus_hardware_interface/async_serial.hpp"

#include <iostream>

namespace asinus_hardware_interface
{
AsyncSerialReader::AsyncSerialReader(const std::string &device, unsigned int baud_rate)
: device_(device), baud_(baud_rate), work_guard_(io_context_.get_executor())
{
    // use a modest read buffer to avoid large zero-padding in previews
    read_buffer_.resize(512);
}

AsyncSerialReader::~AsyncSerialReader()
{
    stop();
}

bool AsyncSerialReader::start()
{
    if (running_.exchange(true)) {
        return true; // already running
    }

    // open serial port
    serial_ = std::make_unique<boost::asio::serial_port>(io_context_);
    boost::system::error_code ec;
    serial_->open(device_, ec);
    if (ec) {
        std::cerr << "Failed to open serial port " << device_ << ": " << ec.message() << std::endl;
        running_.store(false);
        return false;
    }
    serial_->set_option(boost::asio::serial_port_base::baud_rate(baud_), ec);
    if (ec) {
        std::cerr << "Failed to set baud rate: " << ec.message() << std::endl;
    }

    // start reading
    do_read();

    // run io_context in thread
    io_thread_ = std::thread([this]() {
        io_context_.run();
    });

    return true;
}

void AsyncSerialReader::stop()
{
    if (!running_.exchange(false)) {
        return; // not running
    }

    boost::system::error_code ec;
    if (serial_ && serial_->is_open()) {
        serial_->cancel(ec);
        serial_->close(ec);
    }

    work_guard_.reset();
    io_context_.stop();

    if (io_thread_.joinable()) {
        io_thread_.join();
    }

    serial_.reset();
}

void AsyncSerialReader::set_callback(DataCallback cb)
{
    callback_ = std::move(cb);
}

void AsyncSerialReader::write(const std::vector<uint8_t> &data)
{
    if (!running_) return;

    // Post write work to io_context
    boost::asio::post(io_context_, [this, data]() {
        bool write_in_progress = !write_queue_.empty();
        {
            std::lock_guard<std::mutex> lock(write_mutex_);
            write_queue_.insert(write_queue_.end(), data.begin(), data.end());
        }
        if (!write_in_progress) {
            do_write();
        }
    });
}

void AsyncSerialReader::do_read()
{
    if (!serial_ || !serial_->is_open()) {
        std::cerr << "do_read: serial not open" << std::endl;
        return;
    }

    /*std::cerr << "do_read: calling async_read_some (bufsize=" << read_buffer_.size() << ")" << std::endl;
    std::cerr << "buf contents: ";
    for (auto b : read_buffer_) { std::cerr << std::hex << static_cast<int>(b) << " "; }
    */

    serial_->async_read_some(
        boost::asio::buffer(read_buffer_.data(), read_buffer_.size()),
        [this](const boost::system::error_code &ec, std::size_t bytes_transferred) {
            handle_read(ec, bytes_transferred);
        });
}

void AsyncSerialReader::handle_read(const boost::system::error_code &ec, std::size_t bytes_transferred)
{
    if (ec) {
        if (ec == boost::asio::error::operation_aborted) return;
        std::cerr << "Serial read error: " << ec.message() << std::endl;
        return;
    }

    if (bytes_transferred > 0 && callback_) {
        std::vector<uint8_t> data(read_buffer_.begin(), read_buffer_.begin() + bytes_transferred);
        // invoke callback in io_context thread
        callback_(data);
    }

    // Continue reading
    do_read();
}

void AsyncSerialReader::do_write()
{
    if (!serial_ || !serial_->is_open()) return;

    std::vector<uint8_t> buf;
    {
        std::lock_guard<std::mutex> lock(write_mutex_);
        if (write_queue_.empty()) return;
        buf.swap(write_queue_);
    }

    boost::asio::async_write(*serial_, boost::asio::buffer(buf.data(), buf.size()),
        [this](const boost::system::error_code &ec, std::size_t bytes_transferred) {
            if (ec) {
                if (ec != boost::asio::error::operation_aborted) {
                    std::cerr << "Serial write error: " << ec.message() << std::endl;
                }
                return;
            }
            // If there's more to write, schedule another write
            std::lock_guard<std::mutex> lock(write_mutex_);
            if (!write_queue_.empty()) {
                do_write();
            }
        });
}

} // namespace asinus_hardware_interface
