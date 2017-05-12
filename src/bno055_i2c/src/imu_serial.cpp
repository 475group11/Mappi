#include "imu_serial.h"

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdexcept>
#include <system_error>


namespace {

/** Serial frame start byte */
static const std::uint8_t COMMAND_START = 0xAA;
static const std::uint8_t COMMAND_WRITE = 0x00;
static const std::uint8_t COMMAND_READ = 0x01;
static const std::uint8_t RESPONSE_START = 0xBB;

/** Serial interface baud rate */
static const ::speed_t BAUD = 115200;

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
    _fd = ::open(file_path.c_str(), O_RDWR);
    if (_fd < 0) {
        throw_errno("Failed to open device file");
    }
    struct termios options;
    if (::tcgetattr(_fd, &options) != 0) {
        ::close(_fd);
        throw_errno("Failed to get device attributes");
    }
    if (::cfsetispeed(&options, BAUD) != 0) {
        ::close(_fd);
        throw_errno("Failed to set in baud rate");
    }
    if (::cfsetospeed(&options, BAUD) != 0) {
        ::close(_fd);
        throw_errno("Failed to set out baud rate");
    }
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
        throw_errno("Failed to write");
    }
    // Read response
    std::uint8_t response[2];
    const auto read = ::read(_fd, response, sizeof response);
    if (read != sizeof response) {
        throw_errno("Failed to read response");
    }
    const IMUSerial::ErrorCode result = static_cast<IMUSerial::ErrorCode>(response[1]);
    if (result != IMUSerial::ErrorCode::WriteSuccess) {
        throw IMUSerial::ProtocolError(result);
    }
}

void IMUSerial::read_registers(IMU::Register start_addr, std::uint8_t* values, std::size_t length) {
    if (length > 0xFF) {
        throw std::invalid_argument("cannot read more than 255 registers at a time");
    }
    const std::uint8_t request[] = {
        COMMAND_START, COMMAND_READ, std::uint8_t(start_addr), std::uint8_t(length)
    };
    const auto written = ::write(_fd, request, sizeof request);
    if (written != sizeof request) {
        throw_errno("Failed to write");
    }
    // Read response: header, length, [length] values
    std::vector<std::uint8_t> response(2 + length);
    const auto read = ::read(_fd, response.data(), response.size());
    if (read != response.size()) {
        throw_errno("Failed to read response");
    }
    const IMUSerial::ErrorCode result = static_cast<IMUSerial::ErrorCode>(response[1]);
    if (result != IMUSerial::ErrorCode::WriteSuccess) {
        throw IMUSerial::ProtocolError(result);
    }
    // Copy values
    std::copy(response.cbegin() + 2, response.cend(), values);
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
