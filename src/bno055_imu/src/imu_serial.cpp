#include "imu_serial.h"

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdexcept>
#include <system_error>
#include <cstring>

#include <ros/ros.h>

namespace {

/** Serial frame start byte */
static const std::uint8_t COMMAND_START = 0xAA;
static const std::uint8_t COMMAND_WRITE = 0x00;
static const std::uint8_t COMMAND_READ = 0x01;
static const std::uint8_t RESPONSE_START = 0xBB;
static const std::uint8_t WRITE_ACK_START = 0xEE;
static const std::uint8_t READ_ERROR_START = 0xEE;

/** Serial interface baud rate */
static const ::speed_t BAUD = B115200;

/**
 * Throws an std::system_error with an error code taken from errno and a
 * provided message
 */
void throw_errno(const std::string& message) {
    const auto code = std::error_code(errno, std::system_category());
    throw std::system_error(code, message);
}
}

IMUSerial::IMUSerial(const std::string& file_path) {
    _fd = ::open(file_path.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (_fd < 0) {
        throw_errno("Failed to open device file");
    }
    struct termios options;
    if (::tcgetattr(_fd, &options) != 0) {
        ::close(_fd);
        throw_errno("Failed to get device attributes");
    }
    // Set baud rate
    if (::cfsetispeed(&options, BAUD) != 0) {
        ::close(_fd);
        throw_errno("Failed to set in baud rate");
    }
    if (::cfsetospeed(&options, BAUD) != 0) {
        ::close(_fd);
        throw_errno("Failed to set out baud rate");
    }
    // 8 data bits
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    // Ignore breaks
    options.c_iflag |= IGNBRK;
    // Reset lflag
    options.c_lflag = 0;
    // Reset oflag
    options.c_oflag = 0;
    
    // Disable all flow control
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    // Disable more flow control, enable receive
    options.c_cflag |= (CLOCAL | CREAD);
    // Disable parity
    options.c_cflag &= ~(PARENB | PARODD);
    // Disable 2 stop bits and output flow control
    options.c_cflag &= ~(CSTOPB | CRTSCTS);
    
    // Set 1 second timeout
    options.c_cc[VTIME] = 10;
    options.c_cc[VMIN] = 0;
    if (::tcsetattr(_fd, TCSANOW, &options) != 0) {
        ::close(_fd);
        throw_errno("Failed to set attribute on device file");
    }
}

void IMUSerial::write_register(IMU::Register reg_addr, std::uint8_t value) {
    const std::uint8_t request[] = {
        COMMAND_START, COMMAND_WRITE, std::uint8_t(reg_addr), 1, value
    };
    const auto written = ::write(_fd, request, sizeof request);
    if (written != sizeof request) {
        ROS_ERROR("Tried to write %d bytes of write command, but could only write %d", sizeof request, written);
        throw_errno("Failed to write");
    }
    // Read response
    std::uint8_t response[2];
    const auto read = ::read(_fd, response, sizeof response);
    if (read != sizeof response) {
        ROS_ERROR("Tried to read %d bytes of write response, but could only read %d", sizeof response, read);
        if (read > 0) {
            ROS_ERROR("First byte read was 0x%x", response[0]);
        }
        throw_errno("Failed to read response to write");
    }
    if (response[0] != WRITE_ACK_START) {
        throw std::runtime_error("Incorrect first byte for write acknowledgment");
    }
    const IMUSerial::ErrorCode result = static_cast<IMUSerial::ErrorCode>(response[1]);
    if (result != IMUSerial::ErrorCode::WriteSuccess) {
        throw IMUSerial::ProtocolError(result);
    }
}

void IMUSerial::read_registers(IMU::Register start_addr, std::uint8_t* values, std::size_t length) {
    if (length > 128) {
        throw std::invalid_argument("cannot read more than 128 registers at a time");
    }
    const std::uint8_t request[] = {
        COMMAND_START, COMMAND_READ, std::uint8_t(start_addr), std::uint8_t(length)
    };
    const auto written = ::write(_fd, request, sizeof request);
    if (written != sizeof request) {
        ROS_ERROR("Tried to write %d bytes of read command, but could only write %d", sizeof request, written);
        throw_errno("Failed to write");
    }
    // Read response: header, length, [length] values
    std::vector<std::uint8_t> response(2 + length);
    // Read the first 2 bytes to check for an error
    const auto first_read = ::read(_fd, response.data(), 2);
    if (first_read != 2) {
        ROS_ERROR("Tried to read %d bytes of read response, but could only read %d", 2, first_read);
        throw_errno("Failed to read response");
    }
    // Distinguish between error and success
    if (response[0] == READ_ERROR_START) {
        const auto error_code = static_cast<IMUSerial::ErrorCode>(response[1]);
        throw IMUSerial::ProtocolError(error_code);
    } else if (response[0] == RESPONSE_START) {
        // Check indicated length
        if (response[1] != length) {
            throw std::runtime_error("Incorrect length in read response");
        }
        // Read the remaining data
        const auto read = ::read(_fd, response.data() + 2, length);
        if (read != length) {
            ROS_ERROR("Tried to read %d more bytes of read response, but could only read %d", length, read);
            throw_errno("Failed to read whole response");
        }
        // Copy values
        std::copy(response.cbegin() + 2, response.cend(), values);
    } else {
        throw std::runtime_error("Invalid first byte of read response");
    }
}

IMUSerial::~IMUSerial() {
    ::close(_fd);
}

namespace {
const char* explain_error_code(IMUSerial::ErrorCode code) {
    switch (code) {
    case IMUSerial::ErrorCode::WriteSuccess:
        return "OK";
    case IMUSerial::ErrorCode::WriteFail:
        return "write failed";
    case IMUSerial::ErrorCode::InvalidAddress:
        return "invalid address";
    case IMUSerial::ErrorCode::WriteDisabled:
        return "write disabled";
    case IMUSerial::ErrorCode::WrongStartByte:
        return "wrong start byte";
    case IMUSerial::ErrorCode::BusOverrun:
        return "bus overrun";
    case IMUSerial::ErrorCode::MaxLength:
        return "maximum length error";
    case IMUSerial::ErrorCode::MinLength:
        return "minimum length error";
    case IMUSerial::ErrorCode::ReceiveTimeout:
        return "receive timeout";
    default:
        return "unknown error";
    }
}
}

IMUSerial::ProtocolError::ProtocolError(IMUSerial::ErrorCode code) :
    std::runtime_error(explain_error_code(code)),
    _code(code)
{

}

IMUSerial::ErrorCode IMUSerial::ProtocolError::code() const {
    return _code;
}
