// uart.cpp
// UART implementation for Raspberry Pi 4B (Linux termios)
#include "uart.h"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <cerrno>
#include <cstring>
#include <cstdio>

static constexpr size_t RX_BUF_SIZE = 512;
static constexpr int    RX_TIMEOUT_US = 10000; // 10 ms — select() wake interval

Uart::Uart() : fd_(-1), running_(false) {}

Uart::~Uart() {
    close();
}

void Uart::registerCallback(UartRxCallback cb) {
    callback_ = std::move(cb);
}

bool Uart::open(const char* port, int baudRate) {
    int baud = toBaudConstant(baudRate);
    if (baud == -1) {
        fprintf(stderr, "[Uart] Unsupported baud rate: %d\n", baudRate);
        return false;
    }

    fd_ = ::open(port, O_RDWR | O_NOCTTY | O_NONBLOCK); // Open in non blocking read/write mode
    if (fd_ < 0) {
        fprintf(stderr, "[Uart] Failed to open %s: %s\n", port, strerror(errno));
        return false;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd_, &tty) != 0) {
        fprintf(stderr, "[Uart] tcgetattr failed: %s\n", strerror(errno));
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    // 8N1, no flow control
    tty.c_cflag &= ~PARENB;        // No parity
    tty.c_cflag &= ~CSTOPB;        // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;            // 8 data bits
    tty.c_cflag &= ~CRTSCTS;       // No hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem lines

    // Raw mode: no echo, no signals, no special character processing
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);

    // Disable software flow control
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    // Raw output
    tty.c_oflag &= ~(OPOST | ONLCR);

    // Non-blocking read
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        fprintf(stderr, "[Uart] tcsetattr failed: %s\n", strerror(errno));
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    flush();

    // Start RX thread
    running_ = true;
    rxThread_ = std::thread(&Uart::rxLoop, this);

    return true;
}

void Uart::close() {
    // Signal and join the thread before touching fd_
    running_ = false;
    if (rxThread_.joinable())
        rxThread_.join();

    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool Uart::isOpen() const {
    return fd_ >= 0;
}

int Uart::send(const uint8_t* data, size_t len) {
    if (fd_ < 0) return -1;
    std::printf("Sending: ");
    for(size_t i = 0; i < len; i++) {
        std::printf("%02X ", data[i]);
    }
    std::printf("\n");
    return static_cast<int>(::write(fd_, data, len));
}

int Uart::receive(uint8_t* buf, size_t len) {
    if (fd_ < 0) return -1;
    int n = static_cast<int>(::read(fd_, buf, len));
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return 0;
    return n;
}

int Uart::available() const {
    if (fd_ < 0) return 0;
    int bytes = 0;
    ioctl(fd_, FIONREAD, &bytes);
    return bytes;
}

void Uart::flush() {
    if (fd_ >= 0) tcflush(fd_, TCIOFLUSH);
}

// RX thread: wakes on data or every RX_TIMEOUT_US to check running_.
void Uart::rxLoop() {
    uint8_t buf[RX_BUF_SIZE];

    while (running_) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd_, &fds);

        struct timeval tv;
        tv.tv_sec  = 0;
        tv.tv_usec = RX_TIMEOUT_US;

        int ret = select(fd_ + 1, &fds, nullptr, nullptr, &tv);
        if (ret < 0) {
            if (errno == EINTR) continue; // Interrupted by signal — retry
            fprintf(stderr, "[Uart] select() error: %s\n", strerror(errno));
            break;
        }
        if (ret == 0) continue; // Timeout — loop back to check running_

        int n = ::read(fd_, buf, sizeof(buf));
        if (n > 0 && callback_) {
            callback_(buf, static_cast<size_t>(n));
        }
    }
}

int Uart::toBaudConstant(int baudRate) {
    switch (baudRate) {
        case 115200:  return B115200;
        case 230400:  return B230400;
        case 460800:  return B460800;
        case 921600:  return B921600;
        case 1000000: return B1000000;
        case 1500000: return B1500000;
        case 2000000: return B2000000;
        default:      return -1;
    }
}
