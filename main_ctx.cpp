// main_ctx.cpp
// Application context — owns Uart and GyroAPI, handles RX framing
#include "main_ctx.h"

#include <iostream>

MainContext::MainContext() : rxLen_(0) {}

MainContext::~MainContext() {
    close();
}

bool MainContext::init(const char* port, int baudRate) {
    uart_.registerCallback([this](const uint8_t* buf, size_t len) {
        onUartRx(buf, len);
    });

    if (!uart_.open(port, baudRate)) {
        std::cerr << "[MainContext] Failed to open UART on " << port << "\n";
        return false;
    }

    gyroApi_.init(&uart_);
    return true;
}

void MainContext::close() {
    uart_.close();
}

gyro::GyroAPI& MainContext::gyroApi() {
    return gyroApi_;
}

void MainContext::onUartRx(const uint8_t* buf, size_t len) {
    for (size_t i = 0; i < len; i++) {
        if (rxLen_ >= sizeof(rxBuf_)) {
            std::cerr << "[MainContext] RX buffer overflow — resetting\n";
            rxLen_ = 0;
        }

        rxBuf_[rxLen_++] = buf[i];

        if (buf[i] == gyro::GYRO_DELIMITER) {
            if (rxBuf_[0] == 0x80) {
                if(gyroApi_.handleBurst(rxBuf_, rxLen_) < 0) {
                    continue;
                }
            } else {
                gyroApi_.signal(rxBuf_, rxLen_);
            }
            rxLen_ = 0;
        }
    }
}
