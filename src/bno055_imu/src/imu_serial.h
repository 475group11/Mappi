#ifndef IMU_SERIAL_H
#define IMU_SERIAL_H

#include "imu.h"
#include "imu_transport.h"

#include <iostream>
#include <stdexcept>

/**
 * A serial port transport implementation
 */
class IMUSerial : public IMU::Transport {
public:

    /**
     * Serial protocol error codes
     */
    enum class ErrorCode {
        WriteSuccess = 0x01,
        WriteFail = 0x03,
        InvalidAddress = 0x04,
        WriteDisabled = 0x05,
        WrongStartByte = 0x06,
        BusOverrun = 0x07,
        MaxLength = 0x08,
        MinLength = 0x09,
        ReceiveTimeout = 0x0A,
    };

    /**
     * A serial protocol exception type
     */
    class ProtocolError : public std::runtime_error {
    public:
        ProtocolError(ErrorCode code);
        ErrorCode code() const;
        virtual ~ProtocolError() = default;
    private:
        ErrorCode _code;
    };


    /**
     * Creates a serial port transport with a provided path to a serial port
     * file
     */
    explicit IMUSerial(const std::string& file_path);

    /**
     * Writes a value to a register on the device
     */
    virtual void write_register(IMU::Register reg_addr, std::uint8_t value);

    /**
     * Reads one or more consecutive register values starting at the provided
     * address. Stores them in values.
     */
    virtual void read_registers(IMU::Register start_addr, std::uint8_t* values, std::size_t length);

    virtual ~IMUSerial();

private:

    /**
     * File descriptor of the serial port file
     */
    int _fd;

    // Prevent copying
    IMUSerial(const IMUSerial&) = delete;
    IMUSerial& operator = (const IMUSerial&) = delete;

};

#endif
