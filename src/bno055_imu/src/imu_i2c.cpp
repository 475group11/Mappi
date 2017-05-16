#include "imu_i2c.h"

IMUI2C::IMUI2C(const std::string& path, std::uint8_t address) :
    _i2c(path)
{
    _i2c.set_address(address);
}

void IMUI2C::write_register(IMU::Register reg_addr, std::uint8_t value) {
    const std::uint8_t bytes[] = { std::uint8_t(reg_addr), value };
    _i2c.write(bytes, 2);
}

void IMUI2C::read_registers(IMU::Register start_addr, std::uint8_t* values, std::size_t length) {
    // Set address
    const std::uint8_t reg_u8 = std::uint8_t(start_addr);
    _i2c.write(&reg_u8, 1);
    // Read values
    _i2c.read(values, length);
}
