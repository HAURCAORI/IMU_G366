#pragma once
#include <cstdint>

// ============================================================
// G366 register interface skeleton
// Based on register map on pages 54-55 and register definitions
// on pages 56-75.
// ============================================================

// Naming convention:
// get_<register_name>_<window>_t
// set_<register_name>_<window>_t

namespace gyro
{
    typedef enum : uint8_t
    {
        // Generic commands
        CMD_READ_REG,
        CMD_WRITE_REG,
        CMD_WIN0_SEL,
        CMD_WIN1_SEL,

        // -------------------------------
        // Window 0
        // -------------------------------
        CMD_BURST_W,

        CMD_MODE_CTRL_R,
        CMD_MODE_CTRL_W,

        CMD_DIAG_STAT_R,
        CMD_FLAG_R,

        CMD_GPIO_R,
        CMD_GPIO_W,

        CMD_COUNT_R,
        CMD_RANGE_OVER_R,

        CMD_TEMP_R,

        CMD_XGYRO_R,
        CMD_YGYRO_R,
        CMD_ZGYRO_R,

        CMD_XACCL_R,
        CMD_YACCL_R,
        CMD_ZACCL_R,

        CMD_ID_R,

        CMD_QTN0_R,
        CMD_QTN1_R,
        CMD_QTN2_R,
        CMD_QTN3_R,

        CMD_XDLTA_R,
        CMD_YDLTA_R,
        CMD_ZDLTA_R,

        CMD_XDLTV_R,
        CMD_YDLTV_R,
        CMD_ZDLTV_R,

        // ANG1~3 share addresses with XDLTA~ZDLTA depending on ATTI mode
        CMD_ANG1_R,
        CMD_ANG2_R,
        CMD_ANG3_R,

        CMD_WIN_CTRL_R,
        CMD_WIN_CTRL_W,

        // -------------------------------
        // Window 1
        // -------------------------------
        CMD_SIG_CTRL_R,
        CMD_SIG_CTRL_W,

        CMD_MSC_CTRL_R,
        CMD_MSC_CTRL_W,

        CMD_SMPL_CTRL_R,
        CMD_SMPL_CTRL_W,

        CMD_FILTER_CTRL_R,
        CMD_FILTER_CTRL_W,

        CMD_UART_CTRL_R,
        CMD_UART_CTRL_W,

        CMD_GLOB_CMD_R,
        CMD_GLOB_CMD_W,

        CMD_BURST_CTRL1_R,
        CMD_BURST_CTRL1_W,

        CMD_BURST_CTRL2_R,
        CMD_BURST_CTRL2_W,

        CMD_POL_CTRL_R,
        CMD_POL_CTRL_W,

        CMD_GLOB_CMD3_R,
        CMD_GLOB_CMD3_W,

        CMD_ATTI_CTRL_R,
        CMD_ATTI_CTRL_W,

        CMD_GLOB_CMD2_R,
        CMD_GLOB_CMD2_W,

        CMD_R_MATRIX_M11_R,
        CMD_R_MATRIX_M11_W,
        CMD_R_MATRIX_M12_R,
        CMD_R_MATRIX_M12_W,
        CMD_R_MATRIX_M13_R,
        CMD_R_MATRIX_M13_W,
        CMD_R_MATRIX_M21_R,
        CMD_R_MATRIX_M21_W,
        CMD_R_MATRIX_M22_R,
        CMD_R_MATRIX_M22_W,
        CMD_R_MATRIX_M23_R,
        CMD_R_MATRIX_M23_W,
        CMD_R_MATRIX_M31_R,
        CMD_R_MATRIX_M31_W,
        CMD_R_MATRIX_M32_R,
        CMD_R_MATRIX_M32_W,
        CMD_R_MATRIX_M33_R,
        CMD_R_MATRIX_M33_W,

        CMD_PROD_ID1_R,
        CMD_PROD_ID2_R,
        CMD_PROD_ID3_R,
        CMD_PROD_ID4_R,

        CMD_VERSION_R,

        CMD_SERIAL_NUM1_R,
        CMD_SERIAL_NUM2_R,
        CMD_SERIAL_NUM3_R,
        CMD_SERIAL_NUM4_R,

        CMD_COUNT
    } CommandID;

    // ========================================================
    // Common register addresses
    // ========================================================

    // Window 0
    static constexpr uint8_t ADDR_BURST_0              = 0x00;
    static constexpr uint8_t ADDR_MODE_CTRL_LO_0       = 0x02;
    static constexpr uint8_t ADDR_MODE_CTRL_HI_0       = 0x03;
    static constexpr uint8_t ADDR_DIAG_STAT_LO_0       = 0x04;
    static constexpr uint8_t ADDR_DIAG_STAT_HI_0       = 0x05;
    static constexpr uint8_t ADDR_FLAG_LO_0            = 0x06;
    static constexpr uint8_t ADDR_FLAG_HI_0            = 0x07;
    static constexpr uint8_t ADDR_GPIO_LO_0            = 0x08;
    static constexpr uint8_t ADDR_GPIO_HI_0            = 0x09;
    static constexpr uint8_t ADDR_COUNT_0              = 0x0A;
    static constexpr uint8_t ADDR_RANGE_OVER_LO_0      = 0x0C;
    static constexpr uint8_t ADDR_RANGE_OVER_HI_0      = 0x0D;
    static constexpr uint8_t ADDR_TEMP_HIGH_0          = 0x0E;
    static constexpr uint8_t ADDR_TEMP_LOW_0           = 0x10;
    static constexpr uint8_t ADDR_XGYRO_HIGH_0         = 0x12;
    static constexpr uint8_t ADDR_XGYRO_LOW_0          = 0x14;
    static constexpr uint8_t ADDR_YGYRO_HIGH_0         = 0x16;
    static constexpr uint8_t ADDR_YGYRO_LOW_0          = 0x18;
    static constexpr uint8_t ADDR_ZGYRO_HIGH_0         = 0x1A;
    static constexpr uint8_t ADDR_ZGYRO_LOW_0          = 0x1C;
    static constexpr uint8_t ADDR_XACCL_HIGH_0         = 0x1E;
    static constexpr uint8_t ADDR_XACCL_LOW_0          = 0x20;
    static constexpr uint8_t ADDR_YACCL_HIGH_0         = 0x22;
    static constexpr uint8_t ADDR_YACCL_LOW_0          = 0x24;
    static constexpr uint8_t ADDR_ZACCL_HIGH_0         = 0x26;
    static constexpr uint8_t ADDR_ZACCL_LOW_0          = 0x28;
    static constexpr uint8_t ADDR_ID_0                 = 0x4C;
    static constexpr uint8_t ADDR_QTN0_HIGH_0          = 0x50;
    static constexpr uint8_t ADDR_QTN0_LOW_0           = 0x52;
    static constexpr uint8_t ADDR_QTN1_HIGH_0          = 0x54;
    static constexpr uint8_t ADDR_QTN1_LOW_0           = 0x56;
    static constexpr uint8_t ADDR_QTN2_HIGH_0          = 0x58;
    static constexpr uint8_t ADDR_QTN2_LOW_0           = 0x5A;
    static constexpr uint8_t ADDR_QTN3_HIGH_0          = 0x5C;
    static constexpr uint8_t ADDR_QTN3_LOW_0           = 0x5E;
    static constexpr uint8_t ADDR_XDLTA_HIGH_0         = 0x64; // shared with ANG1_HIGH
    static constexpr uint8_t ADDR_XDLTA_LOW_0          = 0x66; // shared with ANG1_LOW
    static constexpr uint8_t ADDR_YDLTA_HIGH_0         = 0x68; // shared with ANG2_HIGH
    static constexpr uint8_t ADDR_YDLTA_LOW_0          = 0x6A; // shared with ANG2_LOW
    static constexpr uint8_t ADDR_ZDLTA_HIGH_0         = 0x6C; // shared with ANG3_HIGH
    static constexpr uint8_t ADDR_ZDLTA_LOW_0          = 0x6E; // shared with ANG3_LOW
    static constexpr uint8_t ADDR_XDLTV_HIGH_0         = 0x70;
    static constexpr uint8_t ADDR_XDLTV_LOW_0          = 0x72;
    static constexpr uint8_t ADDR_YDLTV_HIGH_0         = 0x74;
    static constexpr uint8_t ADDR_YDLTV_LOW_0          = 0x76;
    static constexpr uint8_t ADDR_ZDLTV_HIGH_0         = 0x78;
    static constexpr uint8_t ADDR_ZDLTV_LOW_0          = 0x7A;

    // Window 1
    static constexpr uint8_t ADDR_SIG_CTRL_LO_1        = 0x00;
    static constexpr uint8_t ADDR_SIG_CTRL_HI_1        = 0x01;
    static constexpr uint8_t ADDR_MSC_CTRL_LO_1        = 0x02;
    static constexpr uint8_t ADDR_MSC_CTRL_HI_1        = 0x03;
    static constexpr uint8_t ADDR_SMPL_CTRL_LO_1       = 0x04;
    static constexpr uint8_t ADDR_SMPL_CTRL_HI_1       = 0x05;
    static constexpr uint8_t ADDR_FILTER_CTRL_LO_1     = 0x06;
    static constexpr uint8_t ADDR_FILTER_CTRL_HI_1     = 0x07;
    static constexpr uint8_t ADDR_UART_CTRL_LO_1       = 0x08;
    static constexpr uint8_t ADDR_UART_CTRL_HI_1       = 0x09;
    static constexpr uint8_t ADDR_GLOB_CMD_LO_1        = 0x0A;
    static constexpr uint8_t ADDR_GLOB_CMD_HI_1        = 0x0B;
    static constexpr uint8_t ADDR_BURST_CTRL1_LO_1     = 0x0C;
    static constexpr uint8_t ADDR_BURST_CTRL1_HI_1     = 0x0D;
    static constexpr uint8_t ADDR_BURST_CTRL2_LO_1     = 0x0E;
    static constexpr uint8_t ADDR_BURST_CTRL2_HI_1     = 0x0F;
    static constexpr uint8_t ADDR_POL_CTRL_LO_1        = 0x10;
    static constexpr uint8_t ADDR_POL_CTRL_HI_1        = 0x11;
    static constexpr uint8_t ADDR_GLOB_CMD3_LO_1       = 0x12;
    static constexpr uint8_t ADDR_GLOB_CMD3_HI_1       = 0x13;
    static constexpr uint8_t ADDR_ATTI_CTRL_LO_1       = 0x14;
    static constexpr uint8_t ADDR_ATTI_CTRL_HI_1       = 0x15;
    static constexpr uint8_t ADDR_GLOB_CMD2_LO_1       = 0x16;
    static constexpr uint8_t ADDR_GLOB_CMD2_HI_1       = 0x17;

    static constexpr uint8_t ADDR_R_MATRIX_M11_LO_1    = 0x38;
    static constexpr uint8_t ADDR_R_MATRIX_M11_HI_1    = 0x39;
    static constexpr uint8_t ADDR_R_MATRIX_M12_LO_1    = 0x3A;
    static constexpr uint8_t ADDR_R_MATRIX_M12_HI_1    = 0x3B;
    static constexpr uint8_t ADDR_R_MATRIX_M13_LO_1    = 0x3C;
    static constexpr uint8_t ADDR_R_MATRIX_M13_HI_1    = 0x3D;
    static constexpr uint8_t ADDR_R_MATRIX_M21_LO_1    = 0x3E;
    static constexpr uint8_t ADDR_R_MATRIX_M21_HI_1    = 0x3F;
    static constexpr uint8_t ADDR_R_MATRIX_M22_LO_1    = 0x40;
    static constexpr uint8_t ADDR_R_MATRIX_M22_HI_1    = 0x41;
    static constexpr uint8_t ADDR_R_MATRIX_M23_LO_1    = 0x42;
    static constexpr uint8_t ADDR_R_MATRIX_M23_HI_1    = 0x43;
    static constexpr uint8_t ADDR_R_MATRIX_M31_LO_1    = 0x44;
    static constexpr uint8_t ADDR_R_MATRIX_M31_HI_1    = 0x45;
    static constexpr uint8_t ADDR_R_MATRIX_M32_LO_1    = 0x46;
    static constexpr uint8_t ADDR_R_MATRIX_M32_HI_1    = 0x47;
    static constexpr uint8_t ADDR_R_MATRIX_M33_LO_1    = 0x48;
    static constexpr uint8_t ADDR_R_MATRIX_M33_HI_1    = 0x49;

    static constexpr uint8_t ADDR_PROD_ID1_1           = 0x6A;
    static constexpr uint8_t ADDR_PROD_ID2_1           = 0x6C;
    static constexpr uint8_t ADDR_PROD_ID3_1           = 0x6E;
    static constexpr uint8_t ADDR_PROD_ID4_1           = 0x70;
    static constexpr uint8_t ADDR_VERSION_1            = 0x72;
    static constexpr uint8_t ADDR_SERIAL_NUM1_1        = 0x74;
    static constexpr uint8_t ADDR_SERIAL_NUM2_1        = 0x76;
    static constexpr uint8_t ADDR_SERIAL_NUM3_1        = 0x78;
    static constexpr uint8_t ADDR_SERIAL_NUM4_1        = 0x7A;

    static constexpr uint8_t ADDR_WIN_CTRL_LO          = 0x7E;
    static constexpr uint8_t ADDR_WIN_CTRL_HI          = 0x7F;

    // ========================================================
    // Basic raw helpers
    // ========================================================

    typedef struct {
        uint16_t value;
    } get_raw16_t;

    typedef struct {
        uint16_t high;
        uint16_t low;
    } get_raw32_pair_t;

    // ========================================================
    // 0x00 (W0) BURST
    // ========================================================

    typedef struct {
        uint8_t BURST_CMD;   // write 0x00
    } set_burst_0_t;

    // ========================================================
    // 0x02/0x03 (W0) MODE_CTRL
    // ========================================================

    typedef enum : uint8_t {
        MODE_STAT_SAMPLING      = 0,
        MODE_STAT_CONFIGURATION = 1
    } mode_stat_t;

    typedef enum : uint8_t {
        MODE_CMD_NONE             = 0x00,
        MODE_CMD_GO_SAMPLING      = 0x01,
        MODE_CMD_GO_CONFIGURATION = 0x02,
        MODE_CMD_RESERVED         = 0x03
    } mode_cmd_t;

    typedef struct {
        mode_stat_t MODE_STAT;
        mode_cmd_t  MODE_CMD;
    } get_mode_ctrl_0_t;

    typedef struct {
        mode_cmd_t MODE_CMD;
    } set_mode_ctrl_0_t;

    // ========================================================
    // 0x04/0x05 (W0) DIAG_STAT
    // ========================================================

    typedef struct {
        uint8_t FLASH_BU_ERR : 1;
        uint8_t ST_ERR_ALL   : 1;
        uint8_t FLASH_ERR    : 1;
        uint8_t UART_OVF     : 1;
        uint8_t SPI_OVF      : 1;
        uint8_t HARD_ERR     : 2;
        uint8_t reserved_lo  : 1;

        uint8_t DLTV_OVF     : 1;
        uint8_t DLTA_OVF     : 1;
        uint8_t SET_ERR      : 1;
        uint8_t ST_ERR_ACCL  : 1;
        uint8_t ST_ERR_ZGYRO : 1;
        uint8_t ST_ERR_YGYRO : 1;
        uint8_t ST_ERR_XGYRO : 1;
        uint8_t reserved_hi  : 1;
    } get_diag_stat_0_t;

    // ========================================================
    // 0x06/0x07 (W0) FLAG
    // ========================================================

    typedef struct {
        uint8_t EA          : 1;
        uint8_t reserved0   : 1;
        uint8_t ND_ZDLTV    : 1;
        uint8_t ND_YDLTV    : 1;
        uint8_t ND_XDLTV    : 1;
        uint8_t ND_ZDLTA    : 1;
        uint8_t ND_YDLTA    : 1;
        uint8_t ND_XDLTA    : 1;

        uint8_t RO          : 1;
        uint8_t ND_ZACCL    : 1;
        uint8_t ND_YACCL    : 1;
        uint8_t ND_XACCL    : 1;
        uint8_t ND_ZGYRO    : 1;
        uint8_t ND_YGYRO    : 1;
        uint8_t ND_XGYRO    : 1;
        uint8_t ND_TEMP     : 1;
    } get_flag_0_t;

    // ========================================================
    // 0x08/0x09 (W0) GPIO
    // ========================================================

    typedef enum : uint8_t {
        GPIO_DIR_INPUT  = 0,
        GPIO_DIR_OUTPUT = 1
    } gpio_dir_t;

    typedef enum : uint8_t {
        GPIO_LEVEL_LOW  = 0,
        GPIO_LEVEL_HIGH = 1
    } gpio_level_t;

    typedef struct {
        gpio_dir_t   GPIO_DIR1;
        gpio_dir_t   GPIO_DIR2;
        gpio_level_t GPIO_DATA1;
        gpio_level_t GPIO_DATA2;
    } get_gpio_0_t;

    typedef struct {
        gpio_dir_t   GPIO_DIR1;
        gpio_dir_t   GPIO_DIR2;
        gpio_level_t GPIO_DATA1;
        gpio_level_t GPIO_DATA2;
    } set_gpio_0_t;

    // ========================================================
    // 0x0A (W0) COUNT
    // ========================================================

    typedef struct {
        uint16_t value;
    } get_count_0_t;

    // ========================================================
    // 0x0C/0x0D (W0) RANGE_OVER
    // ========================================================

    typedef struct {
        uint8_t RO_ATTI   : 1;
        uint8_t reserved0 : 7;

        uint8_t RO_ZACCL  : 1;
        uint8_t RO_YACCL  : 1;
        uint8_t RO_XACCL  : 1;
        uint8_t RO_ZGYRO  : 1;
        uint8_t RO_YGYRO  : 1;
        uint8_t RO_XGYRO  : 1;
        uint8_t reserved1 : 2;
    } get_range_over_0_t;

    // ========================================================
    // Data registers (raw)
    // ========================================================

    typedef get_raw32_pair_t get_temp_0_t;
    typedef get_raw32_pair_t get_xgyro_0_t;
    typedef get_raw32_pair_t get_ygyro_0_t;
    typedef get_raw32_pair_t get_zgyro_0_t;
    typedef get_raw32_pair_t get_xaccl_0_t;
    typedef get_raw32_pair_t get_yaccl_0_t;
    typedef get_raw32_pair_t get_zaccl_0_t;

    typedef struct {
        uint16_t value; // ID
    } get_id_0_t;

    typedef get_raw32_pair_t get_qtn0_0_t;
    typedef get_raw32_pair_t get_qtn1_0_t;
    typedef get_raw32_pair_t get_qtn2_0_t;
    typedef get_raw32_pair_t get_qtn3_0_t;

    typedef get_raw32_pair_t get_xdlta_0_t;
    typedef get_raw32_pair_t get_ydlta_0_t;
    typedef get_raw32_pair_t get_zdlta_0_t;

    typedef get_raw32_pair_t get_xdltv_0_t;
    typedef get_raw32_pair_t get_ydltv_0_t;
    typedef get_raw32_pair_t get_zdltv_0_t;

    typedef get_raw32_pair_t get_ang1_0_t;
    typedef get_raw32_pair_t get_ang2_0_t;
    typedef get_raw32_pair_t get_ang3_0_t;

    // ========================================================
    // 0x00/0x01 (W1) SIG_CTRL
    // ========================================================

    typedef struct {
        uint8_t reserved_lo : 2;
        uint8_t ND_EN_ZDLTV : 1;
        uint8_t ND_EN_YDLTV : 1;
        uint8_t ND_EN_XDLTV : 1;
        uint8_t ND_EN_ZDLTA : 1;
        uint8_t ND_EN_YDLTA : 1;
        uint8_t ND_EN_XDLTA : 1;

        uint8_t reserved_hi : 1;
        uint8_t ND_EN_ZACCL : 1;
        uint8_t ND_EN_YACCL : 1;
        uint8_t ND_EN_XACCL : 1;
        uint8_t ND_EN_ZGYRO : 1;
        uint8_t ND_EN_YGYRO : 1;
        uint8_t ND_EN_XGYRO : 1;
        uint8_t ND_EN_TEMP  : 1;
    } get_sig_ctrl_1_t;

    typedef get_sig_ctrl_1_t set_sig_ctrl_1_t;

    // ========================================================
    // 0x02/0x03 (W1) MSC_CTRL
    // ========================================================

    typedef enum : uint8_t {
        EXT_SEL_GPIO2                = 0x0,
        EXT_SEL_COUNTER_RESET_INPUT  = 0x1,
        EXT_SEL_INVALID              = 0x2,
        EXT_SEL_TRIGGER_INPUT        = 0x3
    } ext_sel_t;

    typedef enum : uint8_t {
        DRDY_POL_ACTIVE_LOW  = 0,
        DRDY_POL_ACTIVE_HIGH = 1
    } drdy_pol_t;

    typedef struct {
        drdy_pol_t DRDY_POL;
        uint8_t    DRDY_ON;
        ext_sel_t  EXT_SEL;
        uint8_t    SELF_TEST;
        uint8_t    FLASH_TEST;
    } get_msc_ctrl_1_t;

    typedef struct {
        drdy_pol_t DRDY_POL;
        uint8_t    DRDY_ON;
        ext_sel_t  EXT_SEL;
        uint8_t    SELF_TEST;
        uint8_t    FLASH_TEST;
    } set_msc_ctrl_1_t;

    // ========================================================
    // 0x04/0x05 (W1) SMPL_CTRL
    // ========================================================

    typedef enum : uint8_t {
        DOUT_RATE_2000_SPS   = 0x00,
        DOUT_RATE_1000_SPS   = 0x01,
        DOUT_RATE_500_SPS    = 0x02,
        DOUT_RATE_250_SPS    = 0x03,
        DOUT_RATE_125_SPS    = 0x04,
        DOUT_RATE_62_5_SPS   = 0x05,
        DOUT_RATE_31_25_SPS  = 0x06,
        DOUT_RATE_15_625_SPS = 0x07,
        DOUT_RATE_400_SPS    = 0x08,
        DOUT_RATE_200_SPS    = 0x09,
        DOUT_RATE_100_SPS    = 0x0A,
        DOUT_RATE_80_SPS     = 0x0B,
        DOUT_RATE_50_SPS     = 0x0C,
        DOUT_RATE_40_SPS     = 0x0D,
        DOUT_RATE_25_SPS     = 0x0E,
        DOUT_RATE_20_SPS     = 0x0F
    } dout_rate_t;

    typedef struct {
        dout_rate_t DOUT_RATE;
    } get_smpl_ctrl_1_t;

    typedef struct {
        dout_rate_t DOUT_RATE;
    } set_smpl_ctrl_1_t;

    // ========================================================
    // 0x06/0x07 (W1) FILTER_CTRL
    // ========================================================

    typedef enum : uint8_t {
        FILTER_MOV_AVG_TAP_0      = 0x00,
        FILTER_MOV_AVG_TAP_2      = 0x01,
        FILTER_MOV_AVG_TAP_4      = 0x02,
        FILTER_MOV_AVG_TAP_8      = 0x03,
        FILTER_MOV_AVG_TAP_16     = 0x04,
        FILTER_MOV_AVG_TAP_32     = 0x05,
        FILTER_MOV_AVG_TAP_64     = 0x06,
        FILTER_MOV_AVG_TAP_128    = 0x07,
        FILTER_FIR32_FC50         = 0x08,
        FILTER_FIR32_FC100        = 0x09,
        FILTER_FIR32_FC200        = 0x0A,
        FILTER_FIR32_FC400        = 0x0B,
        FILTER_FIR64_FC50         = 0x0C,
        FILTER_FIR64_FC100        = 0x0D,
        FILTER_FIR64_FC200        = 0x0E,
        FILTER_FIR64_FC400        = 0x0F,
        FILTER_FIR128_FC50        = 0x10,
        FILTER_FIR128_FC100       = 0x11,
        FILTER_FIR128_FC200       = 0x12,
        FILTER_FIR128_FC400       = 0x13
    } filter_sel_t;

    typedef struct {
        filter_sel_t FILTER_SEL;
        uint8_t      FILTER_STAT;
    } get_filter_ctrl_1_t;

    typedef struct {
        filter_sel_t FILTER_SEL;
    } set_filter_ctrl_1_t;

    // ========================================================
    // 0x08/0x09 (W1) UART_CTRL
    // ========================================================

    typedef enum : uint8_t {
        BAUD_460800 = 0x0,
        BAUD_230400 = 0x1,
        BAUD_921600 = 0x2
    } baud_rate_t;

    typedef struct {
        uint8_t     UART_AUTO;
        uint8_t     AUTO_START;
        baud_rate_t BAUD_RATE;
    } get_uart_ctrl_1_t;

    typedef struct {
        uint8_t     UART_AUTO;
        uint8_t     AUTO_START;
        baud_rate_t BAUD_RATE;
    } set_uart_ctrl_1_t;

    // ========================================================
    // 0x0A/0x0B (W1) GLOB_CMD
    // ========================================================

    typedef struct {
        uint8_t FLASH_BACKUP;
        uint8_t INITIAL_BACKUP;
        uint8_t SOFT_RST;
        uint8_t NOT_READY;
    } get_glob_cmd_1_t;

    typedef struct {
        uint8_t FLASH_BACKUP;
        uint8_t INITIAL_BACKUP;
        uint8_t SOFT_RST;
    } set_glob_cmd_1_t;

    // ========================================================
    // 0x0C/0x0D (W1) BURST_CTRL1
    // ========================================================

    typedef struct {
        uint8_t CHKSM_OUT : 1;
        uint8_t COUNT_OUT : 1;
        uint8_t GPIO_OUT  : 1;
        uint8_t reserved0 : 5;

        uint8_t ATTI_OUT  : 1;
        uint8_t QTN_OUT   : 1;
        uint8_t DLTV_OUT  : 1;
        uint8_t DLTA_OUT  : 1;
        uint8_t ACCL_OUT  : 1;
        uint8_t GYRO_OUT  : 1;
        uint8_t TEMP_OUT  : 1;
        uint8_t FLAG_OUT  : 1;
    } get_burst_ctrl1_1_t;

    typedef get_burst_ctrl1_1_t set_burst_ctrl1_1_t;

    // ========================================================
    // 0x0E/0x0F (W1) BURST_CTRL2
    // ========================================================

    typedef struct {
        uint8_t reserved0 : 8;

        uint8_t ATTI_BIT  : 1;
        uint8_t QTN_BIT   : 1;
        uint8_t DLTV_BIT  : 1;
        uint8_t DLTA_BIT  : 1;
        uint8_t ACCL_BIT  : 1;
        uint8_t GYRO_BIT  : 1;
        uint8_t TEMP_BIT  : 1;
        uint8_t reserved1 : 1;
    } get_burst_ctrl2_1_t;

    typedef get_burst_ctrl2_1_t set_burst_ctrl2_1_t;

    // ========================================================
    // 0x10/0x11 (W1) POL_CTRL
    // ========================================================

    typedef struct {
        uint8_t reserved0      : 1;
        uint8_t POL_ZACCL      : 1;
        uint8_t POL_YACCL      : 1;
        uint8_t POL_XACCL      : 1;
        uint8_t POL_ZGYRO      : 1;
        uint8_t POL_YGYRO      : 1;
        uint8_t POL_XGYRO      : 1;
        uint8_t reserved1      : 1;

        uint8_t reserved2      : 8;
    } get_pol_ctrl_1_t;

    typedef get_pol_ctrl_1_t set_pol_ctrl_1_t;

    // ========================================================
    // 0x12/0x13 (W1) GLOB_CMD3
    // ========================================================

    typedef enum : uint8_t {
        ACCL_RANGE_PM8G  = 0,
        ACCL_RANGE_PM16G = 1
    } accl_range_ctrl_t;

    typedef struct {
        uint8_t           DLTV_RANGE_CTRL;
        uint8_t           DLTA_RANGE_CTRL;
        accl_range_ctrl_t A_RANGE_CTRL;
    } get_glob_cmd3_1_t;

    typedef struct {
        uint8_t           DLTV_RANGE_CTRL; // 4-bit
        uint8_t           DLTA_RANGE_CTRL; // 4-bit
        accl_range_ctrl_t A_RANGE_CTRL;    // 1-bit
    } set_glob_cmd3_1_t;

    // ========================================================
    // 0x14/0x15 (W1) ATTI_CTRL
    // ========================================================

    typedef enum : uint8_t {
        ATTI_ON_DISABLE         = 0x0,
        ATTI_ON_DELTA_OUTPUT    = 0x1,
        ATTI_ON_ATTI_QTN_OUTPUT = 0x2,
        ATTI_ON_INVALID         = 0x3
    } atti_on_t;

    typedef enum : uint8_t {
        ATTI_MODE_INCLINATION = 0,
        ATTI_MODE_EULER       = 1
    } atti_mode_t;

    typedef struct {
        uint8_t     ATTI_CONV;  // 5-bit
        atti_on_t   ATTI_ON;    // 2-bit
        atti_mode_t ATTI_MODE;  // 1-bit
    } get_atti_ctrl_1_t;

    typedef struct {
        uint8_t     ATTI_CONV;  // 5-bit
        atti_on_t   ATTI_ON;    // 2-bit
        atti_mode_t ATTI_MODE;  // 1-bit
    } set_atti_ctrl_1_t;

    // ========================================================
    // 0x16/0x17 (W1) GLOB_CMD2
    // ========================================================

    typedef enum : uint8_t {
        ATTITUDE_MOTION_PROFILE_MODE_A = 0x0,
        ATTITUDE_MOTION_PROFILE_MODE_B = 0x1,
        ATTITUDE_MOTION_PROFILE_MODE_C = 0x2,
        ATTITUDE_MOTION_PROFILE_INVALID = 0x3
    } attitude_motion_profile_t;

    typedef struct {
        attitude_motion_profile_t ATTITUDE_MOTION_PROFILE;
        uint8_t                   ATTITUDE_MOTION_PROFILE_STAT;
        uint8_t                   FLASH_ROTATION_BACKUP;
        uint8_t                   INITIAL_ROTATION_BACKUP;
    } get_glob_cmd2_1_t;

    typedef struct {
        attitude_motion_profile_t ATTITUDE_MOTION_PROFILE;
        uint8_t                   FLASH_ROTATION_BACKUP;
        uint8_t                   INITIAL_ROTATION_BACKUP;
    } set_glob_cmd2_1_t;

    // ========================================================
    // R_MATRIX
    // ========================================================

    typedef struct { int16_t value; } get_r_matrix_m11_1_t;
    typedef struct { int16_t value; } set_r_matrix_m11_1_t;
    typedef struct { int16_t value; } get_r_matrix_m12_1_t;
    typedef struct { int16_t value; } set_r_matrix_m12_1_t;
    typedef struct { int16_t value; } get_r_matrix_m13_1_t;
    typedef struct { int16_t value; } set_r_matrix_m13_1_t;
    typedef struct { int16_t value; } get_r_matrix_m21_1_t;
    typedef struct { int16_t value; } set_r_matrix_m21_1_t;
    typedef struct { int16_t value; } get_r_matrix_m22_1_t;
    typedef struct { int16_t value; } set_r_matrix_m22_1_t;
    typedef struct { int16_t value; } get_r_matrix_m23_1_t;
    typedef struct { int16_t value; } set_r_matrix_m23_1_t;
    typedef struct { int16_t value; } get_r_matrix_m31_1_t;
    typedef struct { int16_t value; } set_r_matrix_m31_1_t;
    typedef struct { int16_t value; } get_r_matrix_m32_1_t;
    typedef struct { int16_t value; } set_r_matrix_m32_1_t;
    typedef struct { int16_t value; } get_r_matrix_m33_1_t;
    typedef struct { int16_t value; } set_r_matrix_m33_1_t;

    // ========================================================
    // PROD_ID / VERSION / SERIAL / WIN_CTRL
    // ========================================================

    typedef struct { uint16_t value; } get_prod_id1_1_t;
    typedef struct { uint16_t value; } get_prod_id2_1_t;
    typedef struct { uint16_t value; } get_prod_id3_1_t;
    typedef struct { uint16_t value; } get_prod_id4_1_t;
    typedef struct { uint16_t value; } get_version_1_t;
    typedef struct { uint16_t value; } get_serial_num1_1_t;
    typedef struct { uint16_t value; } get_serial_num2_1_t;
    typedef struct { uint16_t value; } get_serial_num3_1_t;
    typedef struct { uint16_t value; } get_serial_num4_1_t;

    typedef enum : uint8_t {
        WINDOW_ID_0 = 0x00,
        WINDOW_ID_1 = 0x01
    } window_id_t;

    typedef struct {
        window_id_t WINDOW_ID;
    } get_win_ctrl_t;

    typedef struct {
        window_id_t WINDOW_ID;
    } set_win_ctrl_t;
}