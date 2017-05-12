#ifndef IMU_TRANSPORT_H
#define IMU_TRANSPORT_H

#include "imu.h"

/**
 * Interface for a transport system that may be used for the IMU.
 * (this is really more like a data link layer)
 */
class IMU::Transport {
public:
    /**
     * Writes a value to a register on the device
     */
    virtual void write_register(Register reg_addr, std::uint8_t value) = 0;

    /**
     * Reads one or more consecutive register values starting at the provided
     * address. Stores them in values.
     */
    virtual void read_registers(Register start_addr, std::uint8_t* values, std::size_t length) = 0;

    virtual ~Transport() = default;
};

#endif
