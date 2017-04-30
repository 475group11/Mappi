#ifndef IMU_PRIVATE_H
#define IMU_PRIVATE_H
#include <cstdint>

namespace {

/**
 * Register addresses
 */
enum class Register : std::uint8_t {
    // Page 0
    CHIP_ID = 0x0,
    ACC_ID = 0x1,
    MAG_ID = 0x2,
    GYR_ID = 0x3,
    SW_REV_ID_LSB = 0x4,
    SW_REV_ID_MSB = 0x5,
    BL_REV_ID = 0x6,
    PAGE_ID = 0x07,
    ACC_DATA_X_LSB = 0x08,
    ACC_DATA_X_MSB = 0x09,
    ACC_DATA_Y_LSB = 0x0A,
    ACC_DATA_Y_MSB = 0x0B,
    ACC_DATA_Z_LSB = 0x0C,
    ACC_DATA_Z_MSB = 0x0D,
    MAG_DATA_X_LSB = 0x0E,
    MAG_DATA_X_MSB = 0x0F,
    MAG_DATA_Y_LSB = 0x10,
    MAG_DATA_Y_MSB = 0x11,
    MAG_DATA_Z_LSB = 0x12,
    MAG_DATA_Z_MSB = 0x13,
    GYR_DATA_X_LSB = 0x14,
    GYR_DATA_X_MSB = 0x15,
    GYR_DATA_Y_LSB = 0x16,
    GYR_DATA_Y_MSB = 0x17,
    GYR_DATA_Z_LSB = 0x18,
    GYR_DATA_Z_MSB = 0x19,
    EUL_DATA_X_LSB = 0x1A,
    EUL_DATA_X_MSB = 0x1B,
    EUL_DATA_Y_LSB = 0x1C,
    EUL_DATA_Y_MSB = 0x1D,
    EUL_DATA_Z_LSB = 0x1E,
    EUL_DATA_Z_MSB = 0x1F,
    QUA_DATA_W_LSB = 0x20,
    QUA_DATA_W_MSB = 0x21,
    QUA_DATA_X_LSB = 0x22,
    QUA_DATA_X_MSB = 0x23,
    QUA_DATA_Y_LSB = 0x24,
    QUA_DATA_Y_MSB = 0x25,
    QUA_DATA_Z_LSB = 0x26,
    QUA_DATA_Z_MSB = 0x27,
    LIA_DATA_X_LSB = 0x28,
    LIA_DATA_X_MSB = 0x29,
    LIA_DATA_Y_LSB = 0x2A,
    LIA_DATA_Y_MSB = 0x2B,
    LIA_DATA_Z_LSB = 0x2C,
    LIA_DATA_Z_MSB = 0x2D,
    GRV_DATA_X_LSB = 0x2E,
    GRV_DATA_X_MSB = 0x2F,
    GRV_DATA_Y_LSB = 0x30,
    GRV_DATA_Y_MSB = 0x31,
    GRV_DATA_Z_LSB = 0x32,
    GRV_DATA_Z_MSB = 0x33,
    TEMP = 0x34,
    CALIB_STAT = 0x35,
    ST_RESULT = 0x36,
    INT_STA = 0x37,
    SYS_CLK_STATUS = 0x38,
    SYS_STATUS = 0x39,
    SYS_ERR = 0x3A,
    UNIT_SEL = 0x3B,
    OPR_MODE = 0x3D,
    PWR_MODE = 0x3E,
    SYS_TRIGGER = 0x3F,
    TEMP_SOURCE = 0x40,
    AXIS_MAP_CONFIG = 0x41,
    AXIS_MAP_SIGN = 0x42,
    ACC_OFFSET_X_LSB = 0x55,
    ACC_OFFSET_X_MSB = 0x56,
    ACC_OFFSET_Y_LSB = 0x57,
    ACC_OFFSET_Y_MSB = 0x58,
    ACC_OFFSET_Z_LSB = 0x59,
    ACC_OFFSET_Z_MSB = 0x5A,
    MAG_OFFSET_X_LSB = 0x5B,
    MAG_OFFSET_X_MSB = 0x5C,
    MAG_OFFSET_Y_LSB = 0x5D,
    MAG_OFFSET_Y_MSB = 0x5E,
    MAG_OFFSET_Z_LSB = 0x5F,
    MAG_OFFSET_Z_MSB = 0x60,
    GYR_OFFSET_X_LSB = 0x61,
    GYR_OFFSET_X_MSB = 0x62,
    GYR_OFFSET_Y_LSB = 0x63,
    GYR_OFFSET_Y_MSB = 0x64,
    GYR_OFFSET_Z_LSB = 0x65,
    GYR_OFFSET_Z_MSB = 0x66,
    ACC_RADIUS_LSB = 0x67,
    ACC_RADIUS_MSB = 0x68,
    MAG_RADIUS_LSB = 0x69,
    MAG_RADIUS_MSB = 0x6A,

    // Page 1
    ACC_Config = 0x08,
    MAG_Config = 0x09,
    GYR_Config_0 = 0x0A,
    GYR_Config_1 = 0x0B,
    ACC_Sleep_Config = 0x0C,
    GYR_Sleep_Config = 0x0D,
    INT_MSK = 0x0F,
    INT_EN = 0x10,
    ACC_AM_THRES = 0x11,
    ACC_INT_Settings = 0x12,
    ACC_HG_DURATION = 0x13,
    ACC_HG_THRES = 0x14,
    ACC_NM_THRES = 0x15,
    ACC_NM_SET = 0x16,
    GYR_INT_SETTING = 0x17,
    GYR_HR_X_SET = 0x18,
    GYR_DUR_X = 0x19,
    GYR_HR_Y_SET = 0x1A,
    GYR_DUR_Y = 0x1B,
    GYR_HR_Z_SET = 0x1C,
    GYR_DUR_Z = 0x1D,
    GYR_AM_THRES = 0x1E,
    GYR_AM_SET = 0x1F
};

/** Fixed value of CHIP_ID register */
static const std::uint8_t BNO_CHIP_ID = 0xA0;
/** Fixed value of ACC_ID register */
static const std::uint8_t BNO_ACC_ID = 0xFB;
/** Fixed value of MAG_ID register */
static const std::uint8_t BNO_MAG_ID = 0xB2; // Previously 0x32
/** Fixed value of GYR_ID register */
static const std::uint8_t BNO_GYR_ID = 0x8F; // Previously 0x0F


}

#endif
