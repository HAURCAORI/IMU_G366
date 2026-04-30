// main_ctx.h
// Application context — owns Uart and GyroAPI, handles RX framing
#pragma once

#include "uart.h"
#include "gyro_api.h"

class MainContext {
public:
    MainContext();
    ~MainContext();

    // Open UART, register RX callback, and start GyroAPI poll thread.
    bool init(const char* port, int baudRate);

    // Stop threads and close UART.
    void close();

    gyro::GyroAPI& gyroApi();

private:
    Uart    uart_;
    gyro::GyroAPI gyroApi_;

    uint8_t rxBuf_[64];
    size_t  rxLen_;

    // Framing: accumulate bytes until DELIMITER, then dispatch to gyroApi_.
    void onUartRx(const uint8_t* buf, size_t len);
};
