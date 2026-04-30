// uart.h
// UART implementation for Raspberry Pi 4B (Linux termios)
#pragma once

#include <cstdint>
#include <cstddef>
#include <functional>
#include <thread>
#include <atomic>

// Callback signature: invoked from the RX thread when data arrives.
// Register before calling open(). Do not call send() or any blocking
// operation from inside the callback.
using UartRxCallback = std::function<void(const uint8_t* buf, size_t len)>;

class Uart {
public:
    Uart();
    ~Uart();

    // Register the RX callback. Must be called before open().
    void registerCallback(UartRxCallback cb);

    // Open and configure serial port, then start the RX thread.
    // baudRate: 230400 / 460800 / 921600 / 1000000 / 1500000 / 2000000
    bool open(const char* port, int baudRate);

    // Stop the RX thread and close the port.
    void close();

    bool isOpen() const;

    // Write len bytes. Returns bytes written, or -1 on error.
    int send(const uint8_t* data, size_t len);

    // Synchronous read — use only before open() / outside the RX thread.
    // Returns bytes read (0 = nothing available), or -1 on error.
    [[deprecated("Use registerCallback() for all RX. receive() conflicts with the RX thread.")]]
    int receive(uint8_t* buf, size_t len);

    // Bytes waiting in the kernel RX buffer.
    int available() const;

    // Discard all pending TX and RX data.
    void flush();

private:
    int fd_;
    UartRxCallback callback_;
    std::thread rxThread_;
    std::atomic<bool> running_;

    void rxLoop();
    static int toBaudConstant(int baudRate);
};
