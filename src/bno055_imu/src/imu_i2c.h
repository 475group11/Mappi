#ifndef IMU_I2C_H
#define IMU_I2C_H

#include "imu.h"
#include "i2c.h"
#include "imu_transport.h"

/**
 * An I2C transport layer implementation for the IMU
 */
class IMUI2C : public IMU::Transport {
public:
    /**
     * Creates an I2C connection
     *
     * @param path the path to an I2C device file to use
     * @param address the address of the device on the I2C bus
     */
    IMUI2C(const std::string& path, std::uint8_t address);

    /**
     * Writes a value to a register on the device
     */
    virtual void write_register(IMU::Register reg_addr, std::uint8_t value);

    /**
     * Reads one or more consecutive register values starting at the provided
     * address. Stores them in values.
     */
    virtual void read_registers(IMU::Register start_addr, std::uint8_t* values, std::size_t length);

    virtual ~IMUI2C() = default;
private:
    /** The I2C connection */
    I2C _i2c;
};

#endif
