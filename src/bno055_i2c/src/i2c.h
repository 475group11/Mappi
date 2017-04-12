#ifndef I2C_H
#define I2C_H
#include <string>

/**
 * An interface to an I2C peripheral
 */
class I2C {
public:
    /**
     * Creates an I2C interface using the device at the provided path
     */
    I2C(const std::string& path);

    /**
     * Closes the I2C interface
     */
    ~I2C();

    /**
     * Sets the address to communicate with
     *
     * All reads and writes after a successful call to this function will
     * communicate with the device at the provided address.
     */
    void set_address(uint8_t address);

    /**
     * Writes length bytes from a buffer to the device specified by an earlier
     * call to set_address
     */
    void write(const uint8_t* buffer, std::size_t length);

    /**
     * Reads length bytes from the device to a buffer
     */
    void read(uint8_t* buffer, std::size_t length);

private:
    /**
     * An open file descriptor for the I2C device
     */
    int _device_fd;
};

#endif
