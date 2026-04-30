#include "gyro_api.h"

#include <cstdio>
#include <chrono>
#include <sys/time.h>
#include <time.h>
#include <thread>

namespace gyro {

// ============================================================================
// Packet helpers
// ============================================================================

GyroPacket* gyro_packet_alloc(size_t len) {
    GyroPacket* pkt = new GyroPacket;
    pkt->buf = new uint8_t[len];
    pkt->len = len;
    return pkt;
}

void gyro_packet_free(GyroPacket* pkt) {
    if (!pkt) return;
    delete[] pkt->buf;
    delete pkt;
}

static inline uint16_t be16(const uint8_t* p) {
    return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

static inline int16_t be16s(const uint8_t* p) {
    return static_cast<int16_t>(be16(p));
}

static inline int32_t be32s(const uint8_t* p) {
    return static_cast<int32_t>(
        (static_cast<uint32_t>(p[0]) << 24) |
        (static_cast<uint32_t>(p[1]) << 16) |
        (static_cast<uint32_t>(p[2]) << 8)  |
         static_cast<uint32_t>(p[3]));
}

static GyroPacket* make_read_packet(uint8_t addr) {
    GyroPacket* pkt = gyro_packet_alloc(3);
    pkt->buf[0] = static_cast<uint8_t>(addr & 0x7E);
    pkt->buf[1] = 0x00;
    pkt->buf[2] = GYRO_DELIMITER;
    return pkt;
}

static GyroPacket* make_write_packet_8(uint8_t addr, uint8_t value) {
    GyroPacket* pkt = gyro_packet_alloc(3);
    pkt->buf[0] = static_cast<uint8_t>(addr | WRITE_MASK);
    pkt->buf[1] = value;
    pkt->buf[2] = GYRO_DELIMITER;
    return pkt;
}

static GyroPacket* make_write_packet_16(uint8_t addr_lo, uint16_t value) {
    GyroPacket* pkt = gyro_packet_alloc(6);
    pkt->buf[0] = static_cast<uint8_t>(addr_lo | WRITE_MASK);
    pkt->buf[1] = static_cast<uint8_t>(value & 0x00FF);
    pkt->buf[2] = GYRO_DELIMITER;
    pkt->buf[3] = static_cast<uint8_t>((addr_lo + 1) | WRITE_MASK);
    pkt->buf[4] = static_cast<uint8_t>((value >> 8) & 0x00FF);
    pkt->buf[5] = GYRO_DELIMITER;
    return pkt;
}

// ============================================================================
// Packing helpers
// ============================================================================

static uint8_t pack_mode_ctrl_hi(const set_mode_ctrl_0_t* p) {
    return static_cast<uint8_t>(p->MODE_CMD & 0x03);
}

static uint8_t pack_gpio_lo(const set_gpio_0_t* p) {
    uint8_t v = 0;
    v |= static_cast<uint8_t>((p->GPIO_DIR1  & 0x01) << 0);
    v |= static_cast<uint8_t>((p->GPIO_DIR2  & 0x01) << 1);
    v |= static_cast<uint8_t>((p->GPIO_DATA1 & 0x01) << 8); // logical placement in 16-bit register
    v |= static_cast<uint8_t>((p->GPIO_DATA2 & 0x01) << 9); // high bits written through addr_hi in real map
    return static_cast<uint8_t>((p->GPIO_DIR1 & 0x01) |
                                ((p->GPIO_DIR2 & 0x01) << 1) |
                                ((p->GPIO_DATA1 & 0x01) << 2) |
                                ((p->GPIO_DATA2 & 0x01) << 3));
}

static uint16_t pack_sig_ctrl(const set_sig_ctrl_1_t* p) {
    uint16_t v = 0;
    v |= static_cast<uint16_t>((p->ND_EN_ZDLTV & 0x01) << 2);
    v |= static_cast<uint16_t>((p->ND_EN_YDLTV & 0x01) << 3);
    v |= static_cast<uint16_t>((p->ND_EN_XDLTV & 0x01) << 4);
    v |= static_cast<uint16_t>((p->ND_EN_ZDLTA & 0x01) << 5);
    v |= static_cast<uint16_t>((p->ND_EN_YDLTA & 0x01) << 6);
    v |= static_cast<uint16_t>((p->ND_EN_XDLTA & 0x01) << 7);
    v |= static_cast<uint16_t>((p->ND_EN_ZACCL & 0x01) << 9);
    v |= static_cast<uint16_t>((p->ND_EN_YACCL & 0x01) << 10);
    v |= static_cast<uint16_t>((p->ND_EN_XACCL & 0x01) << 11);
    v |= static_cast<uint16_t>((p->ND_EN_ZGYRO & 0x01) << 12);
    v |= static_cast<uint16_t>((p->ND_EN_YGYRO & 0x01) << 13);
    v |= static_cast<uint16_t>((p->ND_EN_XGYRO & 0x01) << 14);
    v |= static_cast<uint16_t>((p->ND_EN_TEMP  & 0x01) << 15);
    return v;
}

static uint16_t pack_msc_ctrl(const set_msc_ctrl_1_t* p) {
    uint16_t v = 0;
    v |= static_cast<uint16_t>((p->DRDY_POL   & 0x01) << 0);
    v |= static_cast<uint16_t>((p->DRDY_ON    & 0x01) << 1);
    v |= static_cast<uint16_t>((p->EXT_SEL    & 0x03) << 2);
    v |= static_cast<uint16_t>((p->SELF_TEST  & 0x01) << 4);
    v |= static_cast<uint16_t>((p->FLASH_TEST & 0x01) << 5);
    return v;
}

static uint16_t pack_smpl_ctrl(const set_smpl_ctrl_1_t* p) {
    return static_cast<uint16_t>((p->DOUT_RATE & 0x0F) << 8);
}

static uint16_t pack_filter_ctrl(const set_filter_ctrl_1_t* p) {
    return static_cast<uint16_t>(p->FILTER_SEL & 0x1F);
}

static uint16_t pack_uart_ctrl(const set_uart_ctrl_1_t* p) {
    uint16_t v = 0;
    v |= static_cast<uint16_t>((p->UART_AUTO  & 0x01) << 0);
    v |= static_cast<uint16_t>((p->AUTO_START & 0x01) << 1);
    v |= static_cast<uint16_t>((p->BAUD_RATE  & 0x03) << 2);
    return v;
}

static uint16_t pack_glob_cmd(const set_glob_cmd_1_t* p) {
    uint16_t v = 0;
    v |= static_cast<uint16_t>((p->FLASH_BACKUP   & 0x01) << 0);
    v |= static_cast<uint16_t>((p->INITIAL_BACKUP & 0x01) << 1);
    v |= static_cast<uint16_t>((p->SOFT_RST       & 0x01) << 7);
    return v;
}

static uint16_t pack_burst_ctrl1(const set_burst_ctrl1_1_t* p) {
    uint16_t v = 0;
    // LO byte (0x0C): bit0=CHKSM_OUT, bit1=COUNT_OUT, bit2=GPIO_OUT
    v |= static_cast<uint16_t>((p->CHKSM_OUT & 0x01) << 0);
    v |= static_cast<uint16_t>((p->COUNT_OUT & 0x01) << 1);
    v |= static_cast<uint16_t>((p->GPIO_OUT  & 0x01) << 2);
    // HI byte (0x0D): bit0=ATTI_OUT, bit1=QTN_OUT, ..., bit7=FLAG_OUT
    v |= static_cast<uint16_t>((p->ATTI_OUT  & 0x01) << 8);
    v |= static_cast<uint16_t>((p->QTN_OUT   & 0x01) << 9);
    v |= static_cast<uint16_t>((p->DLTV_OUT  & 0x01) << 10);
    v |= static_cast<uint16_t>((p->DLTA_OUT  & 0x01) << 11);
    v |= static_cast<uint16_t>((p->ACCL_OUT  & 0x01) << 12);
    v |= static_cast<uint16_t>((p->GYRO_OUT  & 0x01) << 13);
    v |= static_cast<uint16_t>((p->TEMP_OUT  & 0x01) << 14);
    v |= static_cast<uint16_t>((p->FLAG_OUT  & 0x01) << 15);
    return v;
}

static uint16_t pack_burst_ctrl2(const set_burst_ctrl2_1_t* p) {
    uint16_t v = 0;
    v |= static_cast<uint16_t>((p->ATTI_BIT & 0x01) << 8);
    v |= static_cast<uint16_t>((p->QTN_BIT  & 0x01) << 9);
    v |= static_cast<uint16_t>((p->DLTV_BIT & 0x01) << 10);
    v |= static_cast<uint16_t>((p->DLTA_BIT & 0x01) << 11);
    v |= static_cast<uint16_t>((p->ACCL_BIT & 0x01) << 12);
    v |= static_cast<uint16_t>((p->GYRO_BIT & 0x01) << 13);
    v |= static_cast<uint16_t>((p->TEMP_BIT & 0x01) << 14);
    return v;
}

static uint16_t pack_pol_ctrl(const set_pol_ctrl_1_t* p) {
    uint16_t v = 0;
    v |= static_cast<uint16_t>((p->POL_ZACCL & 0x01) << 1);
    v |= static_cast<uint16_t>((p->POL_YACCL & 0x01) << 2);
    v |= static_cast<uint16_t>((p->POL_XACCL & 0x01) << 3);
    v |= static_cast<uint16_t>((p->POL_ZGYRO & 0x01) << 4);
    v |= static_cast<uint16_t>((p->POL_YGYRO & 0x01) << 5);
    v |= static_cast<uint16_t>((p->POL_XGYRO & 0x01) << 6);
    return v;
}

static uint16_t pack_glob_cmd3(const set_glob_cmd3_1_t* p) {
    uint16_t v = 0;
    v |= static_cast<uint16_t>((p->A_RANGE_CTRL    & 0x01) << 0);
    v |= static_cast<uint16_t>((p->DLTA_RANGE_CTRL & 0x0F) << 8);
    v |= static_cast<uint16_t>((p->DLTV_RANGE_CTRL & 0x0F) << 12);
    return v;
}

static uint16_t pack_atti_ctrl(const set_atti_ctrl_1_t* p) {
    uint16_t v = 0;
    v |= static_cast<uint16_t>((p->ATTI_CONV & 0x1F) << 0);   // bits[4:0]  LO byte (0x14)
    v |= static_cast<uint16_t>((p->ATTI_ON   & 0x03) << 9);   // bits[10:9] HI byte (0x15)
    v |= static_cast<uint16_t>((p->ATTI_MODE & 0x01) << 11);  // bit[11]    HI byte (0x15)
    return v;
}

static uint16_t pack_glob_cmd2(const set_glob_cmd2_1_t* p) {
    uint16_t v = 0;
    v |= static_cast<uint16_t>((p->ATTITUDE_MOTION_PROFILE & 0x03) << 0);
    v |= static_cast<uint16_t>((p->FLASH_ROTATION_BACKUP   & 0x01) << 4);
    v |= static_cast<uint16_t>((p->INITIAL_ROTATION_BACKUP & 0x01) << 5);
    return v;
}

static uint16_t pack_r_matrix_value(int16_t value) {
    return static_cast<uint16_t>(value);
}

static uint8_t pack_win_ctrl_lo(const set_win_ctrl_t* p) {
    return static_cast<uint8_t>(p->WINDOW_ID & 0x01);
}

// ============================================================================
// Command table
// ============================================================================

static const CommandInfo kCommandTable[] = {
    { CMD_READ_REG,       0x00, 0x00, true,  false, CMD_KIND_READ_U16,       destroy_result<cmd_read_reg_result_t> },
    { CMD_WRITE_REG,      0x00, 0x00, false, false, CMD_KIND_WRITE_ONLY,     nullptr },
    { CMD_WIN0_SEL,       ADDR_WIN_CTRL_LO, 0x00, false, false, CMD_KIND_WRITE_ONLY, nullptr },
    { CMD_WIN1_SEL,       ADDR_WIN_CTRL_LO, 0x01, false, false, CMD_KIND_WRITE_ONLY, nullptr },

    { CMD_BURST_W,        ADDR_BURST_0,      0x00, false, false, CMD_KIND_WRITE_ONLY, nullptr },
    { CMD_MODE_CTRL_R,    ADDR_MODE_CTRL_LO_0, 0x00, true,  false, CMD_KIND_READ_MODE_CTRL, destroy_result<get_mode_ctrl_0_t> },
    { CMD_MODE_CTRL_W,    ADDR_MODE_CTRL_HI_0, 0x00, false, false, CMD_KIND_WRITE_ONLY, nullptr },
    { CMD_DIAG_STAT_R,    ADDR_DIAG_STAT_LO_0, 0x00, true,  false, CMD_KIND_READ_DIAG_STAT, destroy_result<get_diag_stat_0_t> },
    { CMD_FLAG_R,         ADDR_FLAG_LO_0,      0x00, true,  false, CMD_KIND_READ_FLAG, destroy_result<get_flag_0_t> },
    { CMD_GPIO_R,         ADDR_GPIO_LO_0,      0x00, true,  false, CMD_KIND_READ_GPIO, destroy_result<get_gpio_0_t> },
    { CMD_GPIO_W,         ADDR_GPIO_LO_0,      0x00, false, false, CMD_KIND_WRITE_ONLY, nullptr },
    { CMD_COUNT_R,        ADDR_COUNT_0,        0x00, true,  false, CMD_KIND_READ_U16, destroy_result<get_count_0_t> },
    { CMD_RANGE_OVER_R,   ADDR_RANGE_OVER_LO_0,0x00, true,  false, CMD_KIND_READ_RANGE_OVER, destroy_result<get_range_over_0_t> },

    { CMD_TEMP_R,         ADDR_TEMP_HIGH_0,    ADDR_TEMP_LOW_0,   true,  true, CMD_KIND_READ_U32_PAIR, destroy_result<get_temp_0_t> },
    { CMD_XGYRO_R,        ADDR_XGYRO_HIGH_0,   ADDR_XGYRO_LOW_0,  true,  true, CMD_KIND_READ_U32_PAIR, destroy_result<get_xgyro_0_t> },
    { CMD_YGYRO_R,        ADDR_YGYRO_HIGH_0,   ADDR_YGYRO_LOW_0,  true,  true, CMD_KIND_READ_U32_PAIR, destroy_result<get_ygyro_0_t> },
    { CMD_ZGYRO_R,        ADDR_ZGYRO_HIGH_0,   ADDR_ZGYRO_LOW_0,  true,  true, CMD_KIND_READ_U32_PAIR, destroy_result<get_zgyro_0_t> },
    { CMD_XACCL_R,        ADDR_XACCL_HIGH_0,   ADDR_XACCL_LOW_0,  true,  true, CMD_KIND_READ_U32_PAIR, destroy_result<get_xaccl_0_t> },
    { CMD_YACCL_R,        ADDR_YACCL_HIGH_0,   ADDR_YACCL_LOW_0,  true,  true, CMD_KIND_READ_U32_PAIR, destroy_result<get_yaccl_0_t> },
    { CMD_ZACCL_R,        ADDR_ZACCL_HIGH_0,   ADDR_ZACCL_LOW_0,  true,  true, CMD_KIND_READ_U32_PAIR, destroy_result<get_zaccl_0_t> },

    { CMD_ID_R,           ADDR_ID_0,           0x00, true,  false, CMD_KIND_READ_U16, destroy_result<get_id_0_t> },

    { CMD_QTN0_R,         ADDR_QTN0_HIGH_0,    ADDR_QTN0_LOW_0,   true,  true, CMD_KIND_READ_U32_PAIR, destroy_result<get_qtn0_0_t> },
    { CMD_QTN1_R,         ADDR_QTN1_HIGH_0,    ADDR_QTN1_LOW_0,   true,  true, CMD_KIND_READ_U32_PAIR, destroy_result<get_qtn1_0_t> },
    { CMD_QTN2_R,         ADDR_QTN2_HIGH_0,    ADDR_QTN2_LOW_0,   true,  true, CMD_KIND_READ_U32_PAIR, destroy_result<get_qtn2_0_t> },
    { CMD_QTN3_R,         ADDR_QTN3_HIGH_0,    ADDR_QTN3_LOW_0,   true,  true, CMD_KIND_READ_U32_PAIR, destroy_result<get_qtn3_0_t> },

    { CMD_XDLTA_R,        ADDR_XDLTA_HIGH_0,   ADDR_XDLTA_LOW_0,  true,  true, CMD_KIND_READ_U32_PAIR, destroy_result<get_xdlta_0_t> },
    { CMD_YDLTA_R,        ADDR_YDLTA_HIGH_0,   ADDR_YDLTA_LOW_0,  true,  true, CMD_KIND_READ_U32_PAIR, destroy_result<get_ydlta_0_t> },
    { CMD_ZDLTA_R,        ADDR_ZDLTA_HIGH_0,   ADDR_ZDLTA_LOW_0,  true,  true, CMD_KIND_READ_U32_PAIR, destroy_result<get_zdlta_0_t> },

    { CMD_XDLTV_R,        ADDR_XDLTV_HIGH_0,   ADDR_XDLTV_LOW_0,  true,  true, CMD_KIND_READ_U32_PAIR, destroy_result<get_xdltv_0_t> },
    { CMD_YDLTV_R,        ADDR_YDLTV_HIGH_0,   ADDR_YDLTV_LOW_0,  true,  true, CMD_KIND_READ_U32_PAIR, destroy_result<get_ydltv_0_t> },
    { CMD_ZDLTV_R,        ADDR_ZDLTV_HIGH_0,   ADDR_ZDLTV_LOW_0,  true,  true, CMD_KIND_READ_U32_PAIR, destroy_result<get_zdltv_0_t> },

    { CMD_ANG1_R,         ADDR_XDLTA_HIGH_0,   ADDR_XDLTA_LOW_0,  true,  true, CMD_KIND_READ_U32_PAIR, destroy_result<get_ang1_0_t> },
    { CMD_ANG2_R,         ADDR_YDLTA_HIGH_0,   ADDR_YDLTA_LOW_0,  true,  true, CMD_KIND_READ_U32_PAIR, destroy_result<get_ang2_0_t> },
    { CMD_ANG3_R,         ADDR_ZDLTA_HIGH_0,   ADDR_ZDLTA_LOW_0,  true,  true, CMD_KIND_READ_U32_PAIR, destroy_result<get_ang3_0_t> },

    { CMD_WIN_CTRL_R,     ADDR_WIN_CTRL_LO,    0x00, true,  false, CMD_KIND_READ_WIN_CTRL, destroy_result<get_win_ctrl_t> },
    { CMD_WIN_CTRL_W,     ADDR_WIN_CTRL_LO,    0x00, false, false, CMD_KIND_WRITE_ONLY, nullptr },

    { CMD_SIG_CTRL_R,     ADDR_SIG_CTRL_LO_1,  0x00, true,  false, CMD_KIND_READ_SIG_CTRL, destroy_result<get_sig_ctrl_1_t> },
    { CMD_SIG_CTRL_W,     ADDR_SIG_CTRL_LO_1,  0x00, false, false, CMD_KIND_WRITE_ONLY, nullptr },
    { CMD_MSC_CTRL_R,     ADDR_MSC_CTRL_LO_1,  0x00, true,  false, CMD_KIND_READ_MSC_CTRL, destroy_result<get_msc_ctrl_1_t> },
    { CMD_MSC_CTRL_W,     ADDR_MSC_CTRL_LO_1,  0x00, false, false, CMD_KIND_WRITE_ONLY, nullptr },
    { CMD_SMPL_CTRL_R,    ADDR_SMPL_CTRL_LO_1, 0x00, true,  false, CMD_KIND_READ_SMPL_CTRL, destroy_result<get_smpl_ctrl_1_t> },
    { CMD_SMPL_CTRL_W,    ADDR_SMPL_CTRL_LO_1, 0x00, false, false, CMD_KIND_WRITE_ONLY, nullptr },
    { CMD_FILTER_CTRL_R,  ADDR_FILTER_CTRL_LO_1,0x00,true,  false, CMD_KIND_READ_FILTER_CTRL, destroy_result<get_filter_ctrl_1_t> },
    { CMD_FILTER_CTRL_W,  ADDR_FILTER_CTRL_LO_1,0x00,false, false, CMD_KIND_WRITE_ONLY, nullptr },
    { CMD_UART_CTRL_R,    ADDR_UART_CTRL_LO_1, 0x00, true,  false, CMD_KIND_READ_UART_CTRL, destroy_result<get_uart_ctrl_1_t> },
    { CMD_UART_CTRL_W,    ADDR_UART_CTRL_LO_1, 0x00, false, false, CMD_KIND_WRITE_ONLY, nullptr },
    { CMD_GLOB_CMD_R,     ADDR_GLOB_CMD_LO_1,  0x00, true,  false, CMD_KIND_READ_GLOB_CMD, destroy_result<get_glob_cmd_1_t> },
    { CMD_GLOB_CMD_W,     ADDR_GLOB_CMD_LO_1,  0x00, false, false, CMD_KIND_WRITE_ONLY, nullptr },
    { CMD_BURST_CTRL1_R,  ADDR_BURST_CTRL1_LO_1,0x00,true,  false, CMD_KIND_READ_BURST_CTRL1, destroy_result<get_burst_ctrl1_1_t> },
    { CMD_BURST_CTRL1_W,  ADDR_BURST_CTRL1_LO_1,0x00,false, false, CMD_KIND_WRITE_ONLY, nullptr },
    { CMD_BURST_CTRL2_R,  ADDR_BURST_CTRL2_LO_1,0x00,true,  false, CMD_KIND_READ_BURST_CTRL2, destroy_result<get_burst_ctrl2_1_t> },
    { CMD_BURST_CTRL2_W,  ADDR_BURST_CTRL2_LO_1,0x00,false, false, CMD_KIND_WRITE_ONLY, nullptr },
    { CMD_POL_CTRL_R,     ADDR_POL_CTRL_LO_1,  0x00, true,  false, CMD_KIND_READ_POL_CTRL, destroy_result<get_pol_ctrl_1_t> },
    { CMD_POL_CTRL_W,     ADDR_POL_CTRL_LO_1,  0x00, false, false, CMD_KIND_WRITE_ONLY, nullptr },
    { CMD_GLOB_CMD3_R,    ADDR_GLOB_CMD3_LO_1, 0x00, true,  false, CMD_KIND_READ_GLOB_CMD3, destroy_result<get_glob_cmd3_1_t> },
    { CMD_GLOB_CMD3_W,    ADDR_GLOB_CMD3_LO_1, 0x00, false, false, CMD_KIND_WRITE_ONLY, nullptr },
    { CMD_ATTI_CTRL_R,    ADDR_ATTI_CTRL_LO_1, 0x00, true,  false, CMD_KIND_READ_ATTI_CTRL, destroy_result<get_atti_ctrl_1_t> },
    { CMD_ATTI_CTRL_W,    ADDR_ATTI_CTRL_LO_1, 0x00, false, false, CMD_KIND_WRITE_ONLY, nullptr },
    { CMD_GLOB_CMD2_R,    ADDR_GLOB_CMD2_LO_1, 0x00, true,  false, CMD_KIND_READ_GLOB_CMD2, destroy_result<get_glob_cmd2_1_t> },
    { CMD_GLOB_CMD2_W,    ADDR_GLOB_CMD2_LO_1, 0x00, false, false, CMD_KIND_WRITE_ONLY, nullptr },

    { CMD_R_MATRIX_M11_R, ADDR_R_MATRIX_M11_LO_1,0x00,true,false, CMD_KIND_READ_S16, destroy_result<get_r_matrix_m11_1_t> },
    { CMD_R_MATRIX_M11_W, ADDR_R_MATRIX_M11_LO_1,0x00,false,false,CMD_KIND_WRITE_ONLY, nullptr },
    { CMD_R_MATRIX_M12_R, ADDR_R_MATRIX_M12_LO_1,0x00,true,false, CMD_KIND_READ_S16, destroy_result<get_r_matrix_m12_1_t> },
    { CMD_R_MATRIX_M12_W, ADDR_R_MATRIX_M12_LO_1,0x00,false,false,CMD_KIND_WRITE_ONLY, nullptr },
    { CMD_R_MATRIX_M13_R, ADDR_R_MATRIX_M13_LO_1,0x00,true,false, CMD_KIND_READ_S16, destroy_result<get_r_matrix_m13_1_t> },
    { CMD_R_MATRIX_M13_W, ADDR_R_MATRIX_M13_LO_1,0x00,false,false,CMD_KIND_WRITE_ONLY, nullptr },
    { CMD_R_MATRIX_M21_R, ADDR_R_MATRIX_M21_LO_1,0x00,true,false, CMD_KIND_READ_S16, destroy_result<get_r_matrix_m21_1_t> },
    { CMD_R_MATRIX_M21_W, ADDR_R_MATRIX_M21_LO_1,0x00,false,false,CMD_KIND_WRITE_ONLY, nullptr },
    { CMD_R_MATRIX_M22_R, ADDR_R_MATRIX_M22_LO_1,0x00,true,false, CMD_KIND_READ_S16, destroy_result<get_r_matrix_m22_1_t> },
    { CMD_R_MATRIX_M22_W, ADDR_R_MATRIX_M22_LO_1,0x00,false,false,CMD_KIND_WRITE_ONLY, nullptr },
    { CMD_R_MATRIX_M23_R, ADDR_R_MATRIX_M23_LO_1,0x00,true,false, CMD_KIND_READ_S16, destroy_result<get_r_matrix_m23_1_t> },
    { CMD_R_MATRIX_M23_W, ADDR_R_MATRIX_M23_LO_1,0x00,false,false,CMD_KIND_WRITE_ONLY, nullptr },
    { CMD_R_MATRIX_M31_R, ADDR_R_MATRIX_M31_LO_1,0x00,true,false, CMD_KIND_READ_S16, destroy_result<get_r_matrix_m31_1_t> },
    { CMD_R_MATRIX_M31_W, ADDR_R_MATRIX_M31_LO_1,0x00,false,false,CMD_KIND_WRITE_ONLY, nullptr },
    { CMD_R_MATRIX_M32_R, ADDR_R_MATRIX_M32_LO_1,0x00,true,false, CMD_KIND_READ_S16, destroy_result<get_r_matrix_m32_1_t> },
    { CMD_R_MATRIX_M32_W, ADDR_R_MATRIX_M32_LO_1,0x00,false,false,CMD_KIND_WRITE_ONLY, nullptr },
    { CMD_R_MATRIX_M33_R, ADDR_R_MATRIX_M33_LO_1,0x00,true,false, CMD_KIND_READ_S16, destroy_result<get_r_matrix_m33_1_t> },
    { CMD_R_MATRIX_M33_W, ADDR_R_MATRIX_M33_LO_1,0x00,false,false,CMD_KIND_WRITE_ONLY, nullptr },

    { CMD_PROD_ID1_R,     ADDR_PROD_ID1_1, 0x00, true, false, CMD_KIND_READ_U16, destroy_result<get_prod_id1_1_t> },
    { CMD_PROD_ID2_R,     ADDR_PROD_ID2_1, 0x00, true, false, CMD_KIND_READ_U16, destroy_result<get_prod_id2_1_t> },
    { CMD_PROD_ID3_R,     ADDR_PROD_ID3_1, 0x00, true, false, CMD_KIND_READ_U16, destroy_result<get_prod_id3_1_t> },
    { CMD_PROD_ID4_R,     ADDR_PROD_ID4_1, 0x00, true, false, CMD_KIND_READ_U16, destroy_result<get_prod_id4_1_t> },

    { CMD_VERSION_R,      ADDR_VERSION_1,  0x00, true, false, CMD_KIND_READ_U16, destroy_result<get_version_1_t> },

    { CMD_SERIAL_NUM1_R,  ADDR_SERIAL_NUM1_1, 0x00, true, false, CMD_KIND_READ_U16, destroy_result<get_serial_num1_1_t> },
    { CMD_SERIAL_NUM2_R,  ADDR_SERIAL_NUM2_1, 0x00, true, false, CMD_KIND_READ_U16, destroy_result<get_serial_num2_1_t> },
    { CMD_SERIAL_NUM3_R,  ADDR_SERIAL_NUM3_1, 0x00, true, false, CMD_KIND_READ_U16, destroy_result<get_serial_num3_1_t> },
    { CMD_SERIAL_NUM4_R,  ADDR_SERIAL_NUM4_1, 0x00, true, false, CMD_KIND_READ_U16, destroy_result<get_serial_num4_1_t> },
};

static constexpr size_t kCommandTableCount =
    sizeof(kCommandTable) / sizeof(kCommandTable[0]);

const CommandInfo* find_command(CommandID cmdID) {
    for (size_t i = 0; i < kCommandTableCount; ++i) {
        if (kCommandTable[i].cmdID == cmdID) {
            return &kCommandTable[i];
        }
    }
    return nullptr;
}

// ============================================================================
// Request creation
// ============================================================================

GyroPacket* GyroAPI::create_request(CommandID cmdID, const void* params) {
    switch (cmdID) {
    case CMD_READ_REG: {
        const cmd_read_reg_param_t* p = static_cast<const cmd_read_reg_param_t*>(params);
        return make_read_packet(p->addr);
    }
    case CMD_WRITE_REG: {
        const cmd_write_reg_param_t* p = static_cast<const cmd_write_reg_param_t*>(params);
        return make_write_packet_8(p->addr, p->value);
    }
    case CMD_WIN0_SEL:
        return make_write_packet_8(ADDR_WIN_CTRL_LO, 0x00);
    case CMD_WIN1_SEL:
        return make_write_packet_8(ADDR_WIN_CTRL_LO, 0x01);
    case CMD_BURST_W:
        return make_write_packet_8(ADDR_BURST_0, 0x00);

    case CMD_MODE_CTRL_W: {
        const set_mode_ctrl_0_t* p = static_cast<const set_mode_ctrl_0_t*>(params);
        return make_write_packet_8(ADDR_MODE_CTRL_HI_0, pack_mode_ctrl_hi(p));
    }
    case CMD_GPIO_W: {
        const set_gpio_0_t* p = static_cast<const set_gpio_0_t*>(params);
        return make_write_packet_8(ADDR_GPIO_LO_0, pack_gpio_lo(p));
    }
    case CMD_WIN_CTRL_W: {
        const set_win_ctrl_t* p = static_cast<const set_win_ctrl_t*>(params);
        return make_write_packet_8(ADDR_WIN_CTRL_LO, pack_win_ctrl_lo(p));
    }

    case CMD_SIG_CTRL_W: {
        const set_sig_ctrl_1_t* p = static_cast<const set_sig_ctrl_1_t*>(params);
        return make_write_packet_16(ADDR_SIG_CTRL_LO_1, pack_sig_ctrl(p));
    }
    case CMD_MSC_CTRL_W: {
        const set_msc_ctrl_1_t* p = static_cast<const set_msc_ctrl_1_t*>(params);
        return make_write_packet_16(ADDR_MSC_CTRL_LO_1, pack_msc_ctrl(p));
    }
    case CMD_SMPL_CTRL_W: {
        const set_smpl_ctrl_1_t* p = static_cast<const set_smpl_ctrl_1_t*>(params);
        return make_write_packet_16(ADDR_SMPL_CTRL_LO_1, pack_smpl_ctrl(p));
    }
    case CMD_FILTER_CTRL_W: {
        const set_filter_ctrl_1_t* p = static_cast<const set_filter_ctrl_1_t*>(params);
        return make_write_packet_16(ADDR_FILTER_CTRL_LO_1, pack_filter_ctrl(p));
    }
    case CMD_UART_CTRL_W: {
        const set_uart_ctrl_1_t* p = static_cast<const set_uart_ctrl_1_t*>(params);
        return make_write_packet_16(ADDR_UART_CTRL_LO_1, pack_uart_ctrl(p));
    }
    case CMD_GLOB_CMD_W: {
        const set_glob_cmd_1_t* p = static_cast<const set_glob_cmd_1_t*>(params);
        return make_write_packet_16(ADDR_GLOB_CMD_LO_1, pack_glob_cmd(p));
    }
    case CMD_BURST_CTRL1_W: {
        const set_burst_ctrl1_1_t* p = static_cast<const set_burst_ctrl1_1_t*>(params);
        return make_write_packet_16(ADDR_BURST_CTRL1_LO_1, pack_burst_ctrl1(p));
    }
    case CMD_BURST_CTRL2_W: {
        const set_burst_ctrl2_1_t* p = static_cast<const set_burst_ctrl2_1_t*>(params);
        return make_write_packet_16(ADDR_BURST_CTRL2_LO_1, pack_burst_ctrl2(p));
    }
    case CMD_POL_CTRL_W: {
        const set_pol_ctrl_1_t* p = static_cast<const set_pol_ctrl_1_t*>(params);
        return make_write_packet_16(ADDR_POL_CTRL_LO_1, pack_pol_ctrl(p));
    }
    case CMD_GLOB_CMD3_W: {
        const set_glob_cmd3_1_t* p = static_cast<const set_glob_cmd3_1_t*>(params);
        return make_write_packet_16(ADDR_GLOB_CMD3_LO_1, pack_glob_cmd3(p));
    }
    case CMD_ATTI_CTRL_W: {
        const set_atti_ctrl_1_t* p = static_cast<const set_atti_ctrl_1_t*>(params);
        return make_write_packet_16(ADDR_ATTI_CTRL_LO_1, pack_atti_ctrl(p));
    }
    case CMD_GLOB_CMD2_W: {
        const set_glob_cmd2_1_t* p = static_cast<const set_glob_cmd2_1_t*>(params);
        return make_write_packet_16(ADDR_GLOB_CMD2_LO_1, pack_glob_cmd2(p));
    }

    case CMD_R_MATRIX_M11_W: {
        const set_r_matrix_m11_1_t* p = static_cast<const set_r_matrix_m11_1_t*>(params);
        return make_write_packet_16(ADDR_R_MATRIX_M11_LO_1, pack_r_matrix_value(p->value));
    }
    case CMD_R_MATRIX_M12_W: {
        const set_r_matrix_m12_1_t* p = static_cast<const set_r_matrix_m12_1_t*>(params);
        return make_write_packet_16(ADDR_R_MATRIX_M12_LO_1, pack_r_matrix_value(p->value));
    }
    case CMD_R_MATRIX_M13_W: {
        const set_r_matrix_m13_1_t* p = static_cast<const set_r_matrix_m13_1_t*>(params);
        return make_write_packet_16(ADDR_R_MATRIX_M13_LO_1, pack_r_matrix_value(p->value));
    }
    case CMD_R_MATRIX_M21_W: {
        const set_r_matrix_m21_1_t* p = static_cast<const set_r_matrix_m21_1_t*>(params);
        return make_write_packet_16(ADDR_R_MATRIX_M21_LO_1, pack_r_matrix_value(p->value));
    }
    case CMD_R_MATRIX_M22_W: {
        const set_r_matrix_m22_1_t* p = static_cast<const set_r_matrix_m22_1_t*>(params);
        return make_write_packet_16(ADDR_R_MATRIX_M22_LO_1, pack_r_matrix_value(p->value));
    }
    case CMD_R_MATRIX_M23_W: {
        const set_r_matrix_m23_1_t* p = static_cast<const set_r_matrix_m23_1_t*>(params);
        return make_write_packet_16(ADDR_R_MATRIX_M23_LO_1, pack_r_matrix_value(p->value));
    }
    case CMD_R_MATRIX_M31_W: {
        const set_r_matrix_m31_1_t* p = static_cast<const set_r_matrix_m31_1_t*>(params);
        return make_write_packet_16(ADDR_R_MATRIX_M31_LO_1, pack_r_matrix_value(p->value));
    }
    case CMD_R_MATRIX_M32_W: {
        const set_r_matrix_m32_1_t* p = static_cast<const set_r_matrix_m32_1_t*>(params);
        return make_write_packet_16(ADDR_R_MATRIX_M32_LO_1, pack_r_matrix_value(p->value));
    }
    case CMD_R_MATRIX_M33_W: {
        const set_r_matrix_m33_1_t* p = static_cast<const set_r_matrix_m33_1_t*>(params);
        return make_write_packet_16(ADDR_R_MATRIX_M33_LO_1, pack_r_matrix_value(p->value));
    }

    default: {
        const CommandInfo* info = find_command(cmdID);
        if (!info) return nullptr;
        return make_read_packet(info->addr_hi);
    }
    }
}

// ============================================================================
// Parsing helpers
// ============================================================================

template <typename T>
static void* make_u16_result(uint16_t value) {
    T* out = new T();
    out->value = value;
    return out;
}

template <typename T>
static void* make_s16_result(int16_t value) {
    T* out = new T();
    out->value = value;
    return out;
}

template <typename T>
static void* make_u32_pair_result(uint16_t hi, uint16_t lo) {
    T* out = new T();
    out->high = hi;
    out->low  = lo;
    return out;
}

void* GyroAPI::parse_single_response(const CommandInfo* info, const GyroPacket* resp) {
    const uint16_t raw = be16(&resp->buf[1]);

    switch (info->kind) {
    case CMD_KIND_READ_U16:
        switch (info->cmdID) {
        case CMD_READ_REG:      return make_u16_result<cmd_read_reg_result_t>(raw);
        case CMD_COUNT_R:       return make_u16_result<get_count_0_t>(raw);
        case CMD_ID_R:          return make_u16_result<get_id_0_t>(raw);
        case CMD_PROD_ID1_R:    return make_u16_result<get_prod_id1_1_t>(raw);
        case CMD_PROD_ID2_R:    return make_u16_result<get_prod_id2_1_t>(raw);
        case CMD_PROD_ID3_R:    return make_u16_result<get_prod_id3_1_t>(raw);
        case CMD_PROD_ID4_R:    return make_u16_result<get_prod_id4_1_t>(raw);
        case CMD_VERSION_R:     return make_u16_result<get_version_1_t>(raw);
        case CMD_SERIAL_NUM1_R: return make_u16_result<get_serial_num1_1_t>(raw);
        case CMD_SERIAL_NUM2_R: return make_u16_result<get_serial_num2_1_t>(raw);
        case CMD_SERIAL_NUM3_R: return make_u16_result<get_serial_num3_1_t>(raw);
        case CMD_SERIAL_NUM4_R: return make_u16_result<get_serial_num4_1_t>(raw);
        default:                return nullptr;
        }

    case CMD_KIND_READ_S16:
        switch (info->cmdID) {
        case CMD_R_MATRIX_M11_R: return make_s16_result<get_r_matrix_m11_1_t>(static_cast<int16_t>(raw));
        case CMD_R_MATRIX_M12_R: return make_s16_result<get_r_matrix_m12_1_t>(static_cast<int16_t>(raw));
        case CMD_R_MATRIX_M13_R: return make_s16_result<get_r_matrix_m13_1_t>(static_cast<int16_t>(raw));
        case CMD_R_MATRIX_M21_R: return make_s16_result<get_r_matrix_m21_1_t>(static_cast<int16_t>(raw));
        case CMD_R_MATRIX_M22_R: return make_s16_result<get_r_matrix_m22_1_t>(static_cast<int16_t>(raw));
        case CMD_R_MATRIX_M23_R: return make_s16_result<get_r_matrix_m23_1_t>(static_cast<int16_t>(raw));
        case CMD_R_MATRIX_M31_R: return make_s16_result<get_r_matrix_m31_1_t>(static_cast<int16_t>(raw));
        case CMD_R_MATRIX_M32_R: return make_s16_result<get_r_matrix_m32_1_t>(static_cast<int16_t>(raw));
        case CMD_R_MATRIX_M33_R: return make_s16_result<get_r_matrix_m33_1_t>(static_cast<int16_t>(raw));
        default:                 return nullptr;
        }

    case CMD_KIND_READ_MODE_CTRL: {
        get_mode_ctrl_0_t* out = new get_mode_ctrl_0_t();
        out->MODE_STAT = static_cast<mode_stat_t>((raw >> 10) & 0x01);
        out->MODE_CMD  = static_cast<mode_cmd_t>((raw >> 8) & 0x03);
        return out;
    }

    case CMD_KIND_READ_DIAG_STAT: {
        get_diag_stat_0_t* out = new get_diag_stat_0_t();
        out->FLASH_BU_ERR = (raw >> 0) & 0x01;
        out->ST_ERR_ALL   = (raw >> 1) & 0x01;
        out->FLASH_ERR    = (raw >> 2) & 0x01;
        out->UART_OVF     = (raw >> 3) & 0x01;
        out->SPI_OVF      = (raw >> 4) & 0x01;
        out->HARD_ERR     = (raw >> 5) & 0x03;
        out->reserved_lo  = (raw >> 7) & 0x01;
        out->DLTV_OVF     = (raw >> 8) & 0x01;
        out->DLTA_OVF     = (raw >> 9) & 0x01;
        out->SET_ERR      = (raw >> 10) & 0x01;
        out->ST_ERR_ACCL  = (raw >> 11) & 0x01;
        out->ST_ERR_ZGYRO = (raw >> 12) & 0x01;
        out->ST_ERR_YGYRO = (raw >> 13) & 0x01;
        out->ST_ERR_XGYRO = (raw >> 14) & 0x01;
        out->reserved_hi  = (raw >> 15) & 0x01;
        return out;
    }

    case CMD_KIND_READ_FLAG: {
        get_flag_0_t* out = new get_flag_0_t();
        out->EA       = (raw >> 0) & 0x01;
        out->reserved0= (raw >> 1) & 0x01;
        out->ND_ZDLTV = (raw >> 2) & 0x01;
        out->ND_YDLTV = (raw >> 3) & 0x01;
        out->ND_XDLTV = (raw >> 4) & 0x01;
        out->ND_ZDLTA = (raw >> 5) & 0x01;
        out->ND_YDLTA = (raw >> 6) & 0x01;
        out->ND_XDLTA = (raw >> 7) & 0x01;
        out->RO       = (raw >> 8) & 0x01;
        out->ND_ZACCL = (raw >> 9) & 0x01;
        out->ND_YACCL = (raw >> 10) & 0x01;
        out->ND_XACCL = (raw >> 11) & 0x01;
        out->ND_ZGYRO = (raw >> 12) & 0x01;
        out->ND_YGYRO = (raw >> 13) & 0x01;
        out->ND_XGYRO = (raw >> 14) & 0x01;
        out->ND_TEMP  = (raw >> 15) & 0x01;
        return out;
    }

    case CMD_KIND_READ_GPIO: {
        get_gpio_0_t* out = new get_gpio_0_t();
        out->GPIO_DIR1  = static_cast<gpio_dir_t>((raw >> 0) & 0x01);
        out->GPIO_DIR2  = static_cast<gpio_dir_t>((raw >> 1) & 0x01);
        out->GPIO_DATA1 = static_cast<gpio_level_t>((raw >> 8) & 0x01);
        out->GPIO_DATA2 = static_cast<gpio_level_t>((raw >> 9) & 0x01);
        return out;
    }

    case CMD_KIND_READ_RANGE_OVER: {
        get_range_over_0_t* out = new get_range_over_0_t();
        out->RO_ATTI   = (raw >> 0) & 0x01;
        out->reserved0 = (raw >> 1) & 0x7F;
        out->RO_ZACCL  = (raw >> 8) & 0x01;
        out->RO_YACCL  = (raw >> 9) & 0x01;
        out->RO_XACCL  = (raw >> 10) & 0x01;
        out->RO_ZGYRO  = (raw >> 11) & 0x01;
        out->RO_YGYRO  = (raw >> 12) & 0x01;
        out->RO_XGYRO  = (raw >> 13) & 0x01;
        out->reserved1 = (raw >> 14) & 0x03;
        return out;
    }

    case CMD_KIND_READ_SIG_CTRL: {
        get_sig_ctrl_1_t* out = new get_sig_ctrl_1_t();
        out->reserved_lo = raw & 0x03;
        out->ND_EN_ZDLTV = (raw >> 2) & 0x01;
        out->ND_EN_YDLTV = (raw >> 3) & 0x01;
        out->ND_EN_XDLTV = (raw >> 4) & 0x01;
        out->ND_EN_ZDLTA = (raw >> 5) & 0x01;
        out->ND_EN_YDLTA = (raw >> 6) & 0x01;
        out->ND_EN_XDLTA = (raw >> 7) & 0x01;
        out->reserved_hi = (raw >> 8) & 0x01;
        out->ND_EN_ZACCL = (raw >> 9) & 0x01;
        out->ND_EN_YACCL = (raw >> 10) & 0x01;
        out->ND_EN_XACCL = (raw >> 11) & 0x01;
        out->ND_EN_ZGYRO = (raw >> 12) & 0x01;
        out->ND_EN_YGYRO = (raw >> 13) & 0x01;
        out->ND_EN_XGYRO = (raw >> 14) & 0x01;
        out->ND_EN_TEMP  = (raw >> 15) & 0x01;
        return out;
    }

    case CMD_KIND_READ_MSC_CTRL: {
        get_msc_ctrl_1_t* out = new get_msc_ctrl_1_t();
        out->DRDY_POL   = static_cast<drdy_pol_t>((raw >> 0) & 0x01);
        out->DRDY_ON    = (raw >> 1) & 0x01;
        out->EXT_SEL    = static_cast<ext_sel_t>((raw >> 2) & 0x03);
        out->SELF_TEST  = (raw >> 4) & 0x01;
        out->FLASH_TEST = (raw >> 5) & 0x01;
        return out;
    }

    case CMD_KIND_READ_SMPL_CTRL: {
        get_smpl_ctrl_1_t* out = new get_smpl_ctrl_1_t();
        out->DOUT_RATE = static_cast<dout_rate_t>((raw >> 8) & 0x0F);
        return out;
    }

    case CMD_KIND_READ_FILTER_CTRL: {
        get_filter_ctrl_1_t* out = new get_filter_ctrl_1_t();
        out->FILTER_SEL  = static_cast<filter_sel_t>(raw & 0x1F);
        out->FILTER_STAT = static_cast<uint8_t>((raw >> 8) & 0x01);
        return out;
    }

    case CMD_KIND_READ_UART_CTRL: {
        get_uart_ctrl_1_t* out = new get_uart_ctrl_1_t();
        out->UART_AUTO  = (raw >> 0) & 0x01;
        out->AUTO_START = (raw >> 1) & 0x01;
        out->BAUD_RATE  = static_cast<baud_rate_t>((raw >> 2) & 0x03);
        return out;
    }

    case CMD_KIND_READ_GLOB_CMD: {
        get_glob_cmd_1_t* out = new get_glob_cmd_1_t();
        out->FLASH_BACKUP   = (raw >> 0) & 0x01;
        out->INITIAL_BACKUP = (raw >> 1) & 0x01;
        out->SOFT_RST       = (raw >> 7) & 0x01;
        out->NOT_READY      = (raw >> 8) & 0x01;
        return out;
    }

    case CMD_KIND_READ_BURST_CTRL1: {
        get_burst_ctrl1_1_t* out = new get_burst_ctrl1_1_t();
        out->CHKSM_OUT = (raw >> 0) & 0x01;
        out->COUNT_OUT = (raw >> 1) & 0x01;
        out->GPIO_OUT  = (raw >> 2) & 0x01;
        out->reserved0 = (raw >> 3) & 0x1F;
        out->ATTI_OUT  = (raw >> 8) & 0x01;
        out->QTN_OUT   = (raw >> 9) & 0x01;
        out->DLTV_OUT  = (raw >> 10) & 0x01;
        out->DLTA_OUT  = (raw >> 11) & 0x01;
        out->ACCL_OUT  = (raw >> 12) & 0x01;
        out->GYRO_OUT  = (raw >> 13) & 0x01;
        out->TEMP_OUT  = (raw >> 14) & 0x01;
        out->FLAG_OUT  = (raw >> 15) & 0x01;
        return out;
    }

    case CMD_KIND_READ_BURST_CTRL2: {
        get_burst_ctrl2_1_t* out = new get_burst_ctrl2_1_t();
        out->reserved0 = raw & 0x00FF;
        out->ATTI_BIT  = (raw >> 8) & 0x01;
        out->QTN_BIT   = (raw >> 9) & 0x01;
        out->DLTV_BIT  = (raw >> 10) & 0x01;
        out->DLTA_BIT  = (raw >> 11) & 0x01;
        out->ACCL_BIT  = (raw >> 12) & 0x01;
        out->GYRO_BIT  = (raw >> 13) & 0x01;
        out->TEMP_BIT  = (raw >> 14) & 0x01;
        out->reserved1 = (raw >> 15) & 0x01;
        return out;
    }

    case CMD_KIND_READ_POL_CTRL: {
        get_pol_ctrl_1_t* out = new get_pol_ctrl_1_t();
        out->reserved0 = (raw >> 0) & 0x01;
        out->POL_ZACCL = (raw >> 1) & 0x01;
        out->POL_YACCL = (raw >> 2) & 0x01;
        out->POL_XACCL = (raw >> 3) & 0x01;
        out->POL_ZGYRO = (raw >> 4) & 0x01;
        out->POL_YGYRO = (raw >> 5) & 0x01;
        out->POL_XGYRO = (raw >> 6) & 0x01;
        out->reserved1 = (raw >> 7) & 0x01;
        out->reserved2 = (raw >> 8) & 0xFF;
        return out;
    }

    case CMD_KIND_READ_GLOB_CMD3: {
        get_glob_cmd3_1_t* out = new get_glob_cmd3_1_t();
        out->A_RANGE_CTRL    = static_cast<accl_range_ctrl_t>(raw & 0x01);
        out->DLTA_RANGE_CTRL = static_cast<uint8_t>((raw >> 8) & 0x0F);
        out->DLTV_RANGE_CTRL = static_cast<uint8_t>((raw >> 12) & 0x0F);
        return out;
    }

    case CMD_KIND_READ_ATTI_CTRL: {
        get_atti_ctrl_1_t* out = new get_atti_ctrl_1_t();
        out->ATTI_CONV = static_cast<uint8_t>((raw >> 0) & 0x1F);
        out->ATTI_ON   = static_cast<atti_on_t>((raw >> 9) & 0x03);
        out->ATTI_MODE = static_cast<atti_mode_t>((raw >> 11) & 0x01);
        return out;
    }

    case CMD_KIND_READ_GLOB_CMD2: {
        get_glob_cmd2_1_t* out = new get_glob_cmd2_1_t();
        out->ATTITUDE_MOTION_PROFILE      = static_cast<attitude_motion_profile_t>((raw >> 0) & 0x03);
        out->ATTITUDE_MOTION_PROFILE_STAT = static_cast<uint8_t>((raw >> 2) & 0x03);
        out->FLASH_ROTATION_BACKUP        = static_cast<uint8_t>((raw >> 4) & 0x01);
        out->INITIAL_ROTATION_BACKUP      = static_cast<uint8_t>((raw >> 5) & 0x01);
        return out;
    }

    case CMD_KIND_READ_WIN_CTRL: {
        get_win_ctrl_t* out = new get_win_ctrl_t();
        out->WINDOW_ID = static_cast<window_id_t>(raw & 0x01);
        return out;
    }

    default:
        return nullptr;
    }
}

void* GyroAPI::parse_double_response(const CommandInfo* info, uint16_t hi, uint16_t lo) {
    switch (info->cmdID) {
    case CMD_TEMP_R:  return make_u32_pair_result<get_temp_0_t>(hi, lo);
    case CMD_XGYRO_R: return make_u32_pair_result<get_xgyro_0_t>(hi, lo);
    case CMD_YGYRO_R: return make_u32_pair_result<get_ygyro_0_t>(hi, lo);
    case CMD_ZGYRO_R: return make_u32_pair_result<get_zgyro_0_t>(hi, lo);
    case CMD_XACCL_R: return make_u32_pair_result<get_xaccl_0_t>(hi, lo);
    case CMD_YACCL_R: return make_u32_pair_result<get_yaccl_0_t>(hi, lo);
    case CMD_ZACCL_R: return make_u32_pair_result<get_zaccl_0_t>(hi, lo);
    case CMD_QTN0_R:  return make_u32_pair_result<get_qtn0_0_t>(hi, lo);
    case CMD_QTN1_R:  return make_u32_pair_result<get_qtn1_0_t>(hi, lo);
    case CMD_QTN2_R:  return make_u32_pair_result<get_qtn2_0_t>(hi, lo);
    case CMD_QTN3_R:  return make_u32_pair_result<get_qtn3_0_t>(hi, lo);
    case CMD_XDLTA_R: return make_u32_pair_result<get_xdlta_0_t>(hi, lo);
    case CMD_YDLTA_R: return make_u32_pair_result<get_ydlta_0_t>(hi, lo);
    case CMD_ZDLTA_R: return make_u32_pair_result<get_zdlta_0_t>(hi, lo);
    case CMD_XDLTV_R: return make_u32_pair_result<get_xdltv_0_t>(hi, lo);
    case CMD_YDLTV_R: return make_u32_pair_result<get_ydltv_0_t>(hi, lo);
    case CMD_ZDLTV_R: return make_u32_pair_result<get_zdltv_0_t>(hi, lo);
    case CMD_ANG1_R:  return make_u32_pair_result<get_ang1_0_t>(hi, lo);
    case CMD_ANG2_R:  return make_u32_pair_result<get_ang2_0_t>(hi, lo);
    case CMD_ANG3_R:  return make_u32_pair_result<get_ang3_0_t>(hi, lo);
    default:          return nullptr;
    }
}

// ============================================================================
// Dispatcher helpers
// ============================================================================

bool GyroAPI::needs_second_read(CommandID cmdID) {
    switch (cmdID) {
    case CMD_TEMP_R:
    case CMD_XGYRO_R:
    case CMD_YGYRO_R:
    case CMD_ZGYRO_R:
    case CMD_XACCL_R:
    case CMD_YACCL_R:
    case CMD_ZACCL_R:
    case CMD_QTN0_R:
    case CMD_QTN1_R:
    case CMD_QTN2_R:
    case CMD_QTN3_R:
    case CMD_XDLTA_R:
    case CMD_YDLTA_R:
    case CMD_ZDLTA_R:
    case CMD_XDLTV_R:
    case CMD_YDLTV_R:
    case CMD_ZDLTV_R:
    case CMD_ANG1_R:
    case CMD_ANG2_R:
    case CMD_ANG3_R:
        return true;
    default:
        return false;
    }
}

uint8_t GyroAPI::second_read_addr(CommandID cmdID) {
    switch (cmdID) {
    case CMD_TEMP_R:  return ADDR_TEMP_LOW_0;
    case CMD_XGYRO_R: return ADDR_XGYRO_LOW_0;
    case CMD_YGYRO_R: return ADDR_YGYRO_LOW_0;
    case CMD_ZGYRO_R: return ADDR_ZGYRO_LOW_0;
    case CMD_XACCL_R: return ADDR_XACCL_LOW_0;
    case CMD_YACCL_R: return ADDR_YACCL_LOW_0;
    case CMD_ZACCL_R: return ADDR_ZACCL_LOW_0;
    case CMD_QTN0_R:  return ADDR_QTN0_LOW_0;
    case CMD_QTN1_R:  return ADDR_QTN1_LOW_0;
    case CMD_QTN2_R:  return ADDR_QTN2_LOW_0;
    case CMD_QTN3_R:  return ADDR_QTN3_LOW_0;
    case CMD_XDLTA_R:
    case CMD_ANG1_R:  return ADDR_XDLTA_LOW_0;
    case CMD_YDLTA_R:
    case CMD_ANG2_R:  return ADDR_YDLTA_LOW_0;
    case CMD_ZDLTA_R:
    case CMD_ANG3_R:  return ADDR_ZDLTA_LOW_0;
    case CMD_XDLTV_R: return ADDR_XDLTV_LOW_0;
    case CMD_YDLTV_R: return ADDR_YDLTV_LOW_0;
    case CMD_ZDLTV_R: return ADDR_ZDLTV_LOW_0;
    default:          return 0x00;
    }
}

bool GyroAPI::has_active_read(const DispatchContext* contexts, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        if (contexts[i].status == DISPATCH_EXECUTE ||
            contexts[i].status == DISPATCH_EXECUTE_2) {
            if (contexts[i].info && contexts[i].info->has_response) {
                return true;
            }
        }
    }
    return false;
}

// ============================================================================
// GyroAPI
// ============================================================================

GyroAPI::GyroAPI()
    : uart_(nullptr),
      contextCount_(0),
      nextSequence_(0),
      polling_(false) {
}

GyroAPI::~GyroAPI() {
    polling_ = false;
    if (pollThread_.joinable()) {
        pollThread_.join();
    }

    for (size_t i = 0; i < contextCount_; ++i) {
        gyro_packet_free(contexts_[i].req);
        if (contexts_[i].resp && contexts_[i].info && contexts_[i].info->destroy) {
            contexts_[i].info->destroy(contexts_[i].resp);
        }
    }
}

void GyroAPI::init(Uart* uart) {
    uart_ = uart;
    polling_ = true;
    pollThread_ = std::thread(&GyroAPI::pollLoop, this);
}

void GyroAPI::sendCommand(CommandID cmdID, const void* params, dispatch_cb cb) {
    const CommandInfo* info = find_command(cmdID);
    if (!info || !uart_) {
        return;
    }

    GyroPacket* req = create_request(cmdID, params);
    if (!req) {
        return;
    }

    DispatchContext ctx{};
    ctx.cmdID    = cmdID;
    ctx.status   = DISPATCH_WAIT;
    ctx.req      = req;
    ctx.callback = cb;
    ctx.info     = info;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        ctx.sequence = nextSequence_++;
        enqueue(ctx);
    }
}

void GyroAPI::signal(const uint8_t* buf, size_t len) {
    if (!buf || len < 4) {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    for (size_t i = 0; i < contextCount_; ++i) {
        DispatchContext* ctx = &contexts_[i];
        if (ctx->status != DISPATCH_EXECUTE &&
            ctx->status != DISPATCH_EXECUTE_2) {
            continue;
        }

        if (!ctx->info || !ctx->info->has_response) {
            continue;
        }

        if (!ctx->req || buf[0] != ctx->req->buf[0]) {
            continue;
        }

        const uint16_t word = be16(&buf[1]);

        if (ctx->status == DISPATCH_EXECUTE && needs_second_read(ctx->cmdID)) {
            ctx->partial_hi = word;
            ctx->has_partial_hi = true;

            gyro_packet_free(ctx->req);
            ctx->req = make_read_packet(second_read_addr(ctx->cmdID));
            ctx->timestamp_ms = nowMs();
            ctx->status = DISPATCH_EXECUTE_2;
            break;
        }

        if (ctx->status == DISPATCH_EXECUTE_2) {
            if (ctx->has_partial_hi) {
                ctx->resp = parse_double_response(ctx->info, ctx->partial_hi, word);
            } else {
                ctx->resp = nullptr;
            }
            ctx->has_partial_hi = false;
            ctx->status = DISPATCH_DONE;
            break;
        }

        GyroPacket resp{ const_cast<uint8_t*>(buf), len };
        ctx->resp = parse_single_response(ctx->info, &resp);
        ctx->status = DISPATCH_DONE;
        break;
    }
}

void GyroAPI::pollLoop() {
    while (polling_) {
        poll();
        std::this_thread::sleep_for(std::chrono::milliseconds(POLL_INTERVAL_MS));
    }
}

void GyroAPI::poll() {
    struct PendingCb {
        dispatch_cb        cb;
        void*              result;
        const CommandInfo* info;
    };

    PendingCb pending[MAX_DISPATCH_CONTEXTS];
    size_t pendingCount = 0;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        const uint64_t now = nowMs();

        for (size_t i = 0; i < contextCount_;) {
            DispatchContext* ctx = &contexts_[i];

            if ((ctx->status == DISPATCH_EXECUTE || ctx->status == DISPATCH_EXECUTE_2) &&
                now - ctx->timestamp_ms > DISPATCH_TIMEOUT_MS) {
                ctx->status = DISPATCH_TIMEOUT;
            }

            switch (ctx->status) {
            case DISPATCH_WAIT:
                if (ctx->info && ctx->info->has_response &&
                    has_active_read(contexts_, contextCount_)) {
                    ++i;
                    break;
                }

                if(ctx->req->len == 6) {
                    uart_->send(ctx->req->buf, 3);
                    // Hardware delay to ensure the data is sent correctly
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    uart_->send(ctx->req->buf + 3, 3);
                } else {
                    uart_->send(ctx->req->buf, ctx->req->len);
                }
                ctx->timestamp_ms = nowMs();
                ctx->status = (ctx->info && ctx->info->has_response)
                            ? DISPATCH_EXECUTE
                            : DISPATCH_DONE;
                ++i;
                break;

            case DISPATCH_EXECUTE:
            case DISPATCH_EXECUTE_2:
                ++i;
                break;

            case DISPATCH_DONE:
                pending[pendingCount].cb     = ctx->callback;
                pending[pendingCount].result = ctx->resp;
                pending[pendingCount].info   = ctx->info;
                ++pendingCount;

                ctx->resp = nullptr;
                dequeue(i);
                break;

            case DISPATCH_TIMEOUT:
                std::fprintf(stderr, "[GyroAPI] Command %d timed out (seq=%u)\n",
                             static_cast<int>(ctx->cmdID), ctx->sequence);
                pending[pendingCount].cb     = ctx->callback;
                pending[pendingCount].result = nullptr;
                pending[pendingCount].info   = ctx->info;
                ++pendingCount;

                dequeue(i);
                break;

            default:
                ++i;
                break;
            }
        }
    }

    for (size_t i = 0; i < pendingCount; ++i) {
        if (pending[i].cb) {
            pending[i].cb(pending[i].result);
        }
        if (pending[i].result && pending[i].info && pending[i].info->destroy) {
            pending[i].info->destroy(pending[i].result);
        }
    }
}

void GyroAPI::enqueue(DispatchContext& ctx) {
    if (contextCount_ >= MAX_DISPATCH_CONTEXTS) {
        std::fprintf(stderr, "[GyroAPI] Queue full - dropping command %d\n",
                     static_cast<int>(ctx.cmdID));
        gyro_packet_free(ctx.req);
        return;
    }
    contexts_[contextCount_++] = ctx;
}

void GyroAPI::dequeue(size_t index) {
    if (index >= contextCount_) return;

    gyro_packet_free(contexts_[index].req);
    contexts_[index].req = nullptr;
    contexts_[index].resp = nullptr;

    for (size_t i = index; i + 1 < contextCount_; ++i) {
        contexts_[i] = contexts_[i + 1];
    }
    --contextCount_;
}

uint64_t GyroAPI::nowMs() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000ULL +
           static_cast<uint64_t>(ts.tv_nsec) / 1000000ULL;
}

uint64_t GyroAPI::nowUs() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000ULL +
           static_cast<uint64_t>(ts.tv_nsec) / 1000ULL;
}

double get_time_double(void) {
    struct timeval tv = { 0 };
    gettimeofday(&tv, NULL);
    return tv.tv_sec + (double)tv.tv_usec / (double)1000000;
}


// ============================================================================
// Burst parser
// ============================================================================

static float cvt_temp(uint32_t temp_raw, bool is32 = true) {
    const int32_t raw = static_cast<int32_t>(temp_raw);
    const float scale = is32 ? (1.0f / (256.0f * 65536.0f)) : (1.0f / 256.0f);
    return raw * scale + 25.0f;
}

static float cvt_gyro(uint32_t gyro_raw, bool is32 = true) {
    const int32_t raw = static_cast<int32_t>(gyro_raw);
    const float scale = is32 ? (1.0f / (66.0f * 65536.0f)) : (1.0f / 66.0f);
    return raw * scale;
}

static float cvt_accl(uint32_t accl_raw, bool is32 = true, bool accel_range_16g = false) {
    const int32_t raw = static_cast<int32_t>(accl_raw);
    const float sf_lsb_per_mg = accel_range_16g ? 2.0f : 4.0f;
    const float scale = is32 ? (1.0f / (sf_lsb_per_mg * 65536.0f)) : (1.0f / sf_lsb_per_mg);
    return raw * scale;
}

static float cvt_quat(uint32_t quat_raw, bool is32 = true) {
    const int32_t raw = static_cast<int32_t>(quat_raw);
    const float scale = is32 ? (1.0f / 1073741824.0f) : (1.0f / 16384.0f);
    return raw * scale;
}

void GyroAPI::startFileDump(const std::string& filename) {
    // If exists, overwrite first
    file_ = std::ofstream(filename, std::ios::trunc | std::ios::binary);
}

void GyroAPI::stopFileDump() {
    if (file_.is_open()) {
        file_.close();
    }
}

void GyroAPI::setEnablePrint(bool enable) {
    enable_print_ = enable;
}

bool GyroAPI::getEnablePrint() {
    return enable_print_;
}

void GyroAPI::dump(uint64_t timestamp, uint32_t temp, const uint32_t gyro[3], const uint32_t accl[3], const uint32_t quat[4]) {
    if (!file_.is_open()) {
        return;
    }
    file_.write(reinterpret_cast<const char*>(&timestamp), sizeof(timestamp));
    file_.write(reinterpret_cast<const char*>(&temp), sizeof(temp));
    file_.write(reinterpret_cast<const char*>(gyro), sizeof(gyro[0])*3);
    file_.write(reinterpret_cast<const char*>(accl), sizeof(accl[0])*3);
    // file_.write(reinterpret_cast<const char*>(quat), sizeof(quat[0])*4);
    // Total byte: 8+4+12+12 = 36 bytes
}

void GyroAPI::setBurstConfig(const set_burst_ctrl1_1_t& burst_ctrl1, const set_burst_ctrl2_1_t& burst_ctrl2) {
    burst_ctrl1_hi_ = burst_ctrl1.FLAG_OUT << 7 | burst_ctrl1.TEMP_OUT << 6 | burst_ctrl1.GYRO_OUT << 5 | burst_ctrl1.ACCL_OUT << 4 | burst_ctrl1.DLTA_OUT << 3 | burst_ctrl1.DLTV_OUT << 2 | burst_ctrl1.QTN_OUT << 1 | burst_ctrl1.ATTI_OUT << 0;
    burst_ctrl1_lo_ = burst_ctrl1.GPIO_OUT << 2 | burst_ctrl1.COUNT_OUT << 1 | burst_ctrl1.CHKSM_OUT << 0;
    burst_ctrl2_hi_ = burst_ctrl2.TEMP_BIT << 6 | burst_ctrl2.GYRO_BIT << 5 | burst_ctrl2.ACCL_BIT << 4 | burst_ctrl2.DLTA_BIT << 3 | burst_ctrl2.DLTV_BIT << 2 | burst_ctrl2.QTN_BIT << 1 | burst_ctrl2.ATTI_BIT << 0;
}

static size_t calculateTotalBytes(uint8_t burst_ctrl1_hi, uint8_t burst_ctrl1_lo, uint8_t burst_ctrl2_hi) {
    auto has_hi = [&](uint8_t bit) -> bool {
        return (burst_ctrl1_hi & (1u << bit)) != 0;
    };

    auto has_lo = [&](uint8_t bit) -> bool {
        return (burst_ctrl1_lo & (1u << bit)) != 0;
    };

    auto is32_hi = [&](uint8_t bit) -> bool {
        return (burst_ctrl2_hi & (1u << bit)) != 0;
    };

    auto bytes_per_field = [&](uint8_t bit, size_t count) -> size_t {
        if (!has_hi(bit)) {
            return 0;
        }
        return (is32_hi(bit) ? 4u : 2u) * count;
    };

    size_t total = 0;

    // Header byte: 0x80
    total += 1;

    // High control fields
    if (has_hi(7)) total += 2;                  // FLAG
    total += bytes_per_field(6, 1);             // TEMP
    total += bytes_per_field(5, 3);             // GYRO
    total += bytes_per_field(4, 3);             // ACCL
    total += bytes_per_field(3, 3);             // DLTA
    total += bytes_per_field(2, 3);             // DLTV
    total += bytes_per_field(1, 4);             // QTN
    total += bytes_per_field(0, 3);             // ATTI

    // Low control fields
    if (has_lo(2)) total += 2;                  // GPIO
    if (has_lo(1)) total += 2;                  // COUNT
    if (has_lo(0)) total += 2;                  // CHKSM

    // Delimiter byte: 0x0D
    total += 1;

    return total;
}

int GyroAPI::handleBurst(const uint8_t* buf, size_t len)
{
    if (!buf || len < 3) {
        return -1;
    }

    // const uint64_t timestamp1 = nowUs();
    // // std::printf("timestamp1: %llu\n", static_cast<unsigned long long>(timestamp1));
    // for(size_t i = 0; i < len; i++) {
    //     std::printf("%02X ", buf[i]);
    // }
    // std::printf("\n");
    // return 0;

    size_t total_bytes = calculateTotalBytes(burst_ctrl1_hi_, burst_ctrl1_lo_, burst_ctrl2_hi_);
    if (len != total_bytes) {
        //std::fprintf(stderr, "[GyroAPI] Invalid burst length: %zu != %zu\n", len, total_bytes);
        return -1;
    }

    

    const uint64_t timestamp = nowUs();
    // const double timestamp_double = get_time_double();

    uint32_t temp_raw = 0;
    uint32_t gyro_raw[3] = {0, 0, 0};
    uint32_t accl_raw[3] = {0, 0, 0};
    uint32_t quat_raw[4] = {0, 0, 0, 0};

    uint16_t flag_raw  = 0;
    uint16_t gpio_raw  = 0;
    uint16_t count_raw = 0;
    uint16_t chksm_raw = 0;

    size_t off = 1; // buf[0] == 0x80

    auto has_lo = [&](uint8_t bit) -> bool {
        return (burst_ctrl1_lo_ & (1u << bit)) != 0;
    };

    auto has_hi = [&](uint8_t bit) -> bool {
        return (burst_ctrl1_hi_ & (1u << bit)) != 0;
    };

    auto is32_hi = [&](uint8_t bit) -> bool {
        return (burst_ctrl2_hi_ & (1u << bit)) != 0;
    };

    auto r16u = [&]() -> uint16_t {
        uint16_t v = be16(&buf[off]);
        off += 2;
        return v;
    };

    auto r16s_u32 = [&]() -> uint32_t {
        int16_t s = static_cast<int16_t>(be16(&buf[off]));
        off += 2;
        return static_cast<uint32_t>(static_cast<int32_t>(s)); // sign-extend to 32-bit
    };

    auto r32u = [&]() -> uint32_t {
        uint32_t v =
            (static_cast<uint32_t>(buf[off]) << 24) |
            (static_cast<uint32_t>(buf[off + 1]) << 16) |
            (static_cast<uint32_t>(buf[off + 2]) << 8) |
             static_cast<uint32_t>(buf[off + 3]);
        off += 4;
        return v;
    };

    auto read_raw_u32 = [&](bool is32) -> uint32_t {
        return is32 ? r32u() : r16s_u32();
    };

    auto skip = [&](bool is32, int count) {
        off += static_cast<size_t>(is32 ? 4 : 2) * static_cast<size_t>(count);
    };

    // FLAG
    if (has_hi(7)) {
        flag_raw = r16u();
    }

    // TEMP
    if (has_hi(6)) {
        temp_raw = read_raw_u32(is32_hi(6));
    }

    // GYRO
    if (has_hi(5)) {
        const bool is32 = is32_hi(5);
        for (int i = 0; i < 3; ++i) {
            gyro_raw[i] = read_raw_u32(is32);
        }
    }

    // ACCL
    if (has_hi(4)) {
        const bool is32 = is32_hi(4);
        for (int i = 0; i < 3; ++i) {
            accl_raw[i] = read_raw_u32(is32);
        }
    }

    // DLTA
    if (has_hi(3)) {
        skip(is32_hi(3), 3);
    }

    // DLTV
    if (has_hi(2)) {
        skip(is32_hi(2), 3);
    }

    // QTN
    if (has_hi(1)) {
        const bool is32 = is32_hi(1);
        for (int i = 0; i < 4; ++i) {
            quat_raw[i] = read_raw_u32(is32);
        }
    }

    // ATTI
    if (has_hi(0)) {
        skip(is32_hi(0), 3);
    }

    // GPIO
    if (has_lo(2)) {
        gpio_raw = r16u();
    }

    // COUNT
    if (has_lo(1)) {
        count_raw = r16u();
    }

    // CHKSM
    if (has_lo(0)) {
        chksm_raw = r16u();
        (void)chksm_raw;
    }

    // Dump only the requested raw channels
    dump(timestamp, temp_raw, gyro_raw, accl_raw, quat_raw);
    // uint64_t timestamp_us = static_cast<uint64_t>(timestamp_double * 1000000.0);
    // dump(timestamp_us, temp_raw, gyro_raw, accl_raw, quat_raw);

    if(!enable_print_) {
        return 0;
    }

    // Print the values
    std::printf("timestamp=[%llu]", static_cast<unsigned long long>(timestamp));

    if (has_hi(6)) {
        std::printf(" temp=[%f]", cvt_temp(temp_raw, is32_hi(6)));
    }
    if (has_hi(5)) {
        std::printf(" gyro=[%f %f %f]",
            cvt_gyro(gyro_raw[0], is32_hi(5)),
            cvt_gyro(gyro_raw[1], is32_hi(5)),
            cvt_gyro(gyro_raw[2], is32_hi(5)));
    }
    if (has_hi(4)) {
        std::printf(" accl=[%f %f %f]",
            cvt_accl(accl_raw[0], is32_hi(4), accel_range_16g_),
            cvt_accl(accl_raw[1], is32_hi(4), accel_range_16g_),
            cvt_accl(accl_raw[2], is32_hi(4), accel_range_16g_));
    }
    if (has_hi(1)) {
        std::printf(" quat=[%f %f %f %f]",
            cvt_quat(quat_raw[0], is32_hi(1)),
            cvt_quat(quat_raw[1], is32_hi(1)),
            cvt_quat(quat_raw[2], is32_hi(1)),
            cvt_quat(quat_raw[3], is32_hi(1)));
    }
    std::printf("\n");

    return 0;
}

} // namespace gyro