#include "i2c.h"

#include <stdexcept>
#include <system_error>
#include <cerrno>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#if __linux__
#include <linux/i2c-dev.h>
#else
// Meaningless values for I2C constants
#define I2C_SLAVE 0
#endif

namespace {

/**
 * Throws an std::system_error with an error code taken from errno and a
 * provided message
 */
void throw_errno(const std::string& message) {
    const auto code = std::error_code(errno, std::system_category());
    throw std::system_error(code, message);
}

}

I2C::I2C(const std::string& path) : _device_fd(0) {
    _device_fd = ::open(path.c_str(), O_RDWR);
    if (_device_fd < 0) {
        throw_errno("Failed to open I2C device file");
    }
}

void I2C::set_address(uint8_t address) {
    if (::ioctl(_device_fd, I2C_SLAVE, address) < 0) {
        throw_errno("Failed to set slave address");
    }
}

void I2C::write(const uint8_t* buffer, std::size_t length) {
    if (::write(_device_fd, buffer, length) != ssize_t(length)) {
        throw_errno("Failed to write");
    }
}


void I2C::read(uint8_t* buffer, std::size_t length) {
    if (::read(_device_fd, buffer, length) != ssize_t(length)) {
        throw_errno("Failed to read");
    }
}

I2C::~I2C() {
    ::close(_device_fd);
}
