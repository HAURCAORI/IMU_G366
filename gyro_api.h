#pragma once

#include <cstdint>
#include <cstddef>
#include <mutex>
#include <thread>
#include <atomic>

#include "uart.h"
#include "g366.h"

#include <string>
#include <fstream>

namespace gyro {

static constexpr uint8_t GYRO_DELIMITER = 0x0D;
static constexpr uint8_t WRITE_MASK     = 0x80;

// ============================================================================
// Basic packet / burst data
// ============================================================================

struct BurstData {
    uint16_t flag = 0;
    float    temp = 0.0f;
    float    gyro[3] = {};
    float    accl[3] = {};
    float    quat[4] = {};
    uint16_t gpio = 0;
    uint16_t count = 0;
};

typedef struct {
    uint8_t* buf;
    size_t   len;
} GyroPacket;

GyroPacket* gyro_packet_alloc(size_t len);
void        gyro_packet_free(GyroPacket* pkt);

// ============================================================================
// Generic raw register command parameters
// ============================================================================

typedef struct {
    uint8_t addr;
} cmd_read_reg_param_t;

typedef struct {
    uint16_t value;
} cmd_read_reg_result_t;

typedef struct {
    uint8_t addr;
    uint8_t value;
} cmd_write_reg_param_t;

// ============================================================================
// Result destruction
// ============================================================================

typedef void (*DestroyFunc)(void* p);

template <typename T>
void destroy_result(void* p) {
    delete static_cast<T*>(p);
}

// ============================================================================
// Command metadata
// ============================================================================

typedef enum : uint8_t {
    CMD_KIND_WRITE_ONLY = 0,
    CMD_KIND_READ_U16,
    CMD_KIND_READ_S16,
    CMD_KIND_READ_U32_PAIR,
    CMD_KIND_READ_MODE_CTRL,
    CMD_KIND_READ_DIAG_STAT,
    CMD_KIND_READ_FLAG,
    CMD_KIND_READ_GPIO,
    CMD_KIND_READ_RANGE_OVER,
    CMD_KIND_READ_SIG_CTRL,
    CMD_KIND_READ_MSC_CTRL,
    CMD_KIND_READ_SMPL_CTRL,
    CMD_KIND_READ_FILTER_CTRL,
    CMD_KIND_READ_UART_CTRL,
    CMD_KIND_READ_GLOB_CMD,
    CMD_KIND_READ_BURST_CTRL1,
    CMD_KIND_READ_BURST_CTRL2,
    CMD_KIND_READ_POL_CTRL,
    CMD_KIND_READ_GLOB_CMD3,
    CMD_KIND_READ_ATTI_CTRL,
    CMD_KIND_READ_GLOB_CMD2,
    CMD_KIND_READ_WIN_CTRL,
} CommandKind;

typedef struct {
    CommandID    cmdID;
    uint8_t      addr_hi;
    uint8_t      addr_lo;
    bool         has_response;
    bool         needs_second_read;
    CommandKind  kind;
    DestroyFunc  destroy;
} CommandInfo;

const CommandInfo* find_command(CommandID cmdID);

// ============================================================================
// Dispatcher
// ============================================================================

static constexpr size_t   MAX_DISPATCH_CONTEXTS = 32;
static constexpr uint64_t DISPATCH_TIMEOUT_MS   = 200;
static constexpr uint64_t POLL_INTERVAL_MS      = 10;

typedef void (*dispatch_cb)(void* result);

typedef enum : uint8_t {
    DISPATCH_WAIT = 0,
    DISPATCH_EXECUTE,
    DISPATCH_EXECUTE_2,
    DISPATCH_DONE,
    DISPATCH_TIMEOUT,
} DispatchStatus;

typedef struct {
    CommandID           cmdID          = {};
    uint32_t            sequence       = 0;
    DispatchStatus      status         = DISPATCH_WAIT;
    uint64_t            timestamp_ms   = 0;
    GyroPacket*         req            = nullptr;
    void*               resp           = nullptr;
    dispatch_cb         callback       = nullptr;
    const CommandInfo*  info           = nullptr;

    uint16_t            partial_hi     = 0;
    bool                has_partial_hi = false;
} DispatchContext;

// ============================================================================
// GyroAPI
// ============================================================================

class GyroAPI {
public:
    GyroAPI();
    ~GyroAPI();

    void init(Uart* uart);
    void sendCommand(CommandID cmdID, const void* params, dispatch_cb cb);

    // Called after one complete UART frame is received.
    void signal(const uint8_t* buf, size_t len);

    // Optional burst/auto packet handler.
    int handleBurst(const uint8_t* buf, size_t len);

    void setBurstConfig(const set_burst_ctrl1_1_t& burst_ctrl1, const set_burst_ctrl2_1_t& burst_ctrl2);

    void startFileDump(const std::string& filename = "gyro.csv");
    void stopFileDump();
    void dump(uint64_t timestamp, uint32_t temp, const uint32_t gyro[3], const uint32_t accl[3], const uint32_t quat[4]);

    void setEnablePrint(bool enable);
    bool getEnablePrint();

private:
    Uart*            uart_;
    DispatchContext  contexts_[MAX_DISPATCH_CONTEXTS];
    size_t           contextCount_;
    uint32_t         nextSequence_;

    std::mutex       mutex_;
    std::thread      pollThread_;
    std::atomic<bool> polling_;

    std::ofstream    file_;

    void poll();
    void pollLoop();

    void enqueue(DispatchContext& ctx);
    void dequeue(size_t index);

    static uint64_t nowMs();
    static uint64_t nowUs();

private:
    static GyroPacket* create_request(CommandID cmdID, const void* params);
    static void*       parse_single_response(const CommandInfo* info, const GyroPacket* resp);
    static void*       parse_double_response(const CommandInfo* info, uint16_t hi, uint16_t lo);
    static bool        has_active_read(const DispatchContext* contexts, size_t count);
    static bool        needs_second_read(CommandID cmdID);
    static uint8_t     second_read_addr(CommandID cmdID);

    uint8_t burst_ctrl1_hi_ = 0xF0;
    uint8_t burst_ctrl1_lo_ = 0x06;
    uint8_t burst_ctrl2_hi_ = 0x00;
    bool    accel_range_16g_ = false;

    bool    enable_print_ = false;
};

} // namespace gyro