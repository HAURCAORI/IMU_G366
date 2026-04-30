#include "main.h"
#include "g366.h"
#include "gyro_api.h"
#include "main_ctx.h"

#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>

#include <zmq.h>
#include <signal.h>

static MainContext* main_context = nullptr;

static void *zmq_context = nullptr;
static void *zmq_requester = nullptr;

typedef struct {
    unsigned int CmdIndex;		// Command number, generated from the client
    unsigned int CmdMode;		// Command mode (TCS_CMD_MODE_)
    unsigned int OpMode;		// Operation Mode (TCS_MODE_)
    int DataLength;	// Data length in Bytes
    double CmdTime_unixtime;	// Command generation time of the client
    uint8_t signal;
} zmq_message;

static void sendCommand(gyro::CommandID cmdID, const void* params, gyro::dispatch_cb cb) {
	main_context->gyroApi().sendCommand(cmdID, params, cb);
}

static void sendCommand(gyro::CommandID cmdID, const void* params) {
	main_context->gyroApi().sendCommand(cmdID, params, nullptr);
}


static void sendCommand(gyro::CommandID cmdID) {
	main_context->gyroApi().sendCommand(cmdID, nullptr, nullptr);
}

static void gyroConfiguration(gyro::set_burst_ctrl1_1_t burst_ctrl1_1, gyro::set_burst_ctrl2_1_t burst_ctrl2_1) {
    using namespace gyro;
    // 1) Configuration mode
    sendCommand(CMD_WIN0_SEL);
    set_mode_ctrl_0_t mode_ctrl_0 = {MODE_CMD_GO_CONFIGURATION};
    sendCommand(gyro::CMD_MODE_CTRL_W, &mode_ctrl_0);

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // 2) Filter + sample rate

    sendCommand(CMD_WIN1_SEL);
    set_filter_ctrl_1_t filter_ctrl_1 = {FILTER_MOV_AVG_TAP_0};
    sendCommand(gyro::CMD_FILTER_CTRL_W, &filter_ctrl_1);
    set_smpl_ctrl_1_t smpl_ctrl_1 = {DOUT_RATE_2000_SPS};
    sendCommand(gyro::CMD_SMPL_CTRL_W, &smpl_ctrl_1);

    std::this_thread::sleep_for(std::chrono::milliseconds(200));


    // 3) Burst layout + quaternion + data width + UART mode
    sendCommand(gyro::CMD_BURST_CTRL1_W, &burst_ctrl1_1);
    sendCommand(gyro::CMD_BURST_CTRL2_W, &burst_ctrl2_1);

    set_atti_ctrl_1_t atti_ctrl_1 = {0x04, ATTI_ON_DISABLE, ATTI_MODE_INCLINATION};
    sendCommand(gyro::CMD_ATTI_CTRL_W, &atti_ctrl_1);
    
    set_glob_cmd2_1_t glob_cmd2_1 = {.ATTITUDE_MOTION_PROFILE= ATTITUDE_MOTION_PROFILE_MODE_A};
    sendCommand(gyro::CMD_GLOB_CMD2_W, &glob_cmd2_1);

    set_glob_cmd3_1_t glob_cmd3_1 = {.A_RANGE_CTRL= ACCL_RANGE_PM8G};
    sendCommand(gyro::CMD_GLOB_CMD3_W, &glob_cmd3_1);

    set_uart_ctrl_1_t uart_ctrl_1 = {.UART_AUTO=1 ,.AUTO_START= 0, .BAUD_RATE= BAUD_460800};
    sendCommand(gyro::CMD_UART_CTRL_W, &uart_ctrl_1);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

	std::cout << "[INFO] Gyro Configuration Complete." << std::endl;
}

static void gyroStart() {
    using namespace gyro;
	std::cout << "[INFO] Starting Gyro Sampling..." << std::endl;
    sendCommand(CMD_WIN0_SEL);
    set_mode_ctrl_0_t mode_ctrl_0 = { MODE_CMD_GO_SAMPLING };
    sendCommand(gyro::CMD_MODE_CTRL_W, &mode_ctrl_0);
}

static void gyroStop() {
    using namespace gyro;
    sendCommand(CMD_WIN0_SEL);
    set_mode_ctrl_0_t mode_ctrl_0 = { MODE_CMD_GO_CONFIGURATION };
    sendCommand(gyro::CMD_MODE_CTRL_W, &mode_ctrl_0);

	std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::cout << "[INFO] Stopping Gyro Sampling..." << std::endl;
}

static void gyroRecordStart() {
    using namespace gyro;
    main_context->gyroApi().setEnablePrint(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::string filename;
    std::cout << "Enter filename: ";
    std::cin >> filename;

    zmq_message message = {
        .CmdIndex = 0,
        .CmdMode = 5,
        .OpMode = 0,
        .DataLength = 1,
        .CmdTime_unixtime = 0,
        .signal = 1
    };

    if (zmq_send(zmq_requester, &message, sizeof(zmq_message), 0) == -1) {
        std::cerr << "[ERROR] Failed to send message to server." << std::endl;
        return;
    }

    uint8_t reply[128];
    if (zmq_recv(zmq_requester, &reply, sizeof(reply), 0) == -1) {
        std::cerr << "[ERROR] Failed to receive reply from server." << std::endl;
        return;
    }

    main_context->gyroApi().startFileDump(filename);

    std::cout << "[INFO] Gyro Record Started. Press 't' to stop." << std::endl;
}

static void gyroRecordStop() {
    using namespace gyro;
    main_context->gyroApi().stopFileDump();

    zmq_message message = {
        .CmdIndex = 0,
        .CmdMode = 5,
        .OpMode = 0,
        .DataLength = 1,
        .CmdTime_unixtime = 0,
        .signal = 2
    };

    if (zmq_send(zmq_requester, &message, sizeof(zmq_message), 0) == -1) {
        std::cerr << "[ERROR] Failed to send message to server." << std::endl;
        return;
    }

    uint8_t reply[128];
    if (zmq_recv(zmq_requester, &reply, sizeof(reply), 0) == -1) {
        std::cerr << "[ERROR] Failed to receive reply from server." << std::endl;
        return;
    }

    std::cout << "[INFO] Gyro Record Stopped." << std::endl;
}

static void gyroSoftReset() {
    using namespace gyro;
    sendCommand(CMD_WIN0_SEL);
    set_mode_ctrl_0_t mode_ctrl_0 = { MODE_CMD_GO_CONFIGURATION };
    sendCommand(gyro::CMD_MODE_CTRL_W, &mode_ctrl_0);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    sendCommand(CMD_WIN1_SEL);

    cmd_write_reg_param_t cmd_write_reg_param = { .addr = 0x8A, .value = 0x80 };
    sendCommand(gyro::CMD_WRITE_REG, &cmd_write_reg_param);

    std::cout << "[INFO] Gyro Soft Reset Complete." << std::endl;
}


static void generalPrintHex(void* result) {
    auto* value = static_cast<uint16_t*>(result);
    if (value) {
        std::cout << "Value: 0x" << std::hex << std::setw(4) << std::setfill('0') << *value << std::endl;
        delete value;
    } else {
        std::cerr << "Read value timed out.\n";
    }
}

static void destroy() {
    main_context->close();
	delete main_context;

    zmq_close(zmq_requester);
    zmq_ctx_destroy(zmq_context);
}

static void signal_handler(int signal) {
    std::cout << "[INFO] Signal " << signal << " received." << std::endl;
    destroy();
    exit(0);
}





int main() {
    ::signal(SIGINT, signal_handler);
    // ZMQ Init as client
    zmq_context = zmq_ctx_new();
    zmq_requester = zmq_socket(zmq_context, ZMQ_REQ);
    zmq_connect(zmq_requester, "tcp://localhost:5556");


    main_context = new MainContext();
    if (!main_context->init("/dev/ttyAMA0", 460800))
        return -1;

    gyro::set_burst_ctrl1_1_t burst_ctrl1_1 = { .CHKSM_OUT = 0, .COUNT_OUT = 0, .GPIO_OUT = 0, .ATTI_OUT = 0, .QTN_OUT = 0, .DLTV_OUT = 0, .DLTA_OUT = 0, .ACCL_OUT = 1, .GYRO_OUT = 1, .TEMP_OUT = 1, .FLAG_OUT = 0 };
    gyro::set_burst_ctrl2_1_t burst_ctrl2_1 = {.ATTI_BIT= 1, .QTN_BIT= 1, .DLTV_BIT= 1, .DLTA_BIT= 1, .ACCL_BIT= 1, .GYRO_BIT= 1, .TEMP_BIT= 1};
	gyroConfiguration(burst_ctrl1_1, burst_ctrl2_1);
    //gyroConfig();

    main_context->gyroApi().setBurstConfig(burst_ctrl1_1, burst_ctrl2_1);

    bool interrupt = false;
    while (!interrupt) {
		// Keyboard interrupt
        char c = std::cin.get();
        
        switch(c) {
            case 's':
                gyroStart();
                break;
            case 'x':
                gyroStop();
                break;
            case 'r':
                gyroRecordStart();
                break;
            case 't':
                gyroRecordStop();
                break;
            case 'q':
                interrupt = true;
				break;
            case 'z':
                gyroSoftReset();
                break;
            case 'p':
                main_context->gyroApi().setEnablePrint(!main_context->gyroApi().getEnablePrint());
                break;
            default:
                break;
		}
    }

    destroy();

    std::cout << "[INFO] Exiting..." << std::endl;
    return 0;
}
