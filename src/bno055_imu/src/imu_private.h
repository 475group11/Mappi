#ifndef IMU_PRIVATE_H
#define IMU_PRIVATE_H
#include <cstdint>

namespace {

/** Fixed value of CHIP_ID register */
static const std::uint8_t BNO_CHIP_ID = 0xA0;
/** Fixed value of ACC_ID register */
static const std::uint8_t BNO_ACC_ID = 0xFB;
/** Fixed value of MAG_ID register */
static const std::uint8_t BNO_MAG_ID = 0x32;
/** Fixed value of GYR_ID register */
static const std::uint8_t BNO_GYR_ID = 0x0F;


}

#endif
