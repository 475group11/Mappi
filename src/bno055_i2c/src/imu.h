#ifndef IMU_H
#define IMU_H
#include <cstdint>
#include <string>
#include <ostream>
#include "i2c.h"

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

namespace {
// Forward-declare register enum
enum class Register : std::uint8_t;
}

/**
 * An interface to a Bosch BNO055 acceleration and orientation sensor
 *
 * This interface uses I2C for communication.
 */
class IMU {
public:

    /**
     * Device operating modes
     */
    enum class OperatingMode : std::uint8_t {
        Config = 0x0,
        AccOnly = 0x1,
        MagOnly = 0x2,
        GyroOnly = 0x3,
        AccMag = 0x4,
        AccGyro = 0x5,
        MagGyro = 0x6,
        AccMagGyro = 0x7,
        Imu = 0x8,
        Compass = 0x9,
        MagForGyro = 0xA,
        NdofFmcOff = 0xB,
        Ndof = 0xC
    };

    /**
     * System status values
     */
    enum class SystemStatus : std::uint8_t {
        Idle = 0,
        SystemError = 1,
        /** Initialing peripherals */
        PeripheralInit = 2,
        SystemInit = 3,
        /** Running a self-test */
        SelfTest = 4,
        /** Running with sensor fusion software enabled */
        RunningFusion = 5,
        /** Running without sensor fustion software */
        Running = 6
    };

    /**
     * System error codes
     */
    enum class SystemError : std::uint8_t {
        Ok = 0,
        /** Peripheral initialization error */
        PeripheralInit = 1,
        /** System initialization error */
        SystemInit = 2,
        /** Self-test failed */
        SelfTestFailed = 3,
        /** Register map value out of range */
        MapValue = 4,
        /** Register map address out of range */
        MapAddress = 5,
        /** Register map write error */
        MapWrite = 6,
        /** BNO low power mode not available for selected operation mode */
        LowPower = 7,
        /** Accelerometer power mode not available */
        AccelerometerPowerMode = 8,
        /** Fusion algorithm configuration error */
        FusionConfig = 9,
        /** Sensor configuration error */
        SensorConfig = 10
    };

    /**
     * Sensor calibration status values
     */
    enum class CalibrationState : std::uint8_t {
        /** Not callibrated */
        None = 0,
        /** Fully callibrated */
        Full = 3
    };

    /**
     * Information about sensor calibration
     */
    class Calibration {
    public:
        /** Magnetometer calibration */
        CalibrationState magnetometer;
        /** Accelerometer calibration */
        CalibrationState accelerometer;
        /** Gyroscope calibration */
        CalibrationState gyroscope;
        /** Overall system calibration */
        CalibrationState system;
        /** Returns true if all components are fully calibrated */
        bool fully_calibrated() const {
            return magnetometer == CalibrationState::Full
                && accelerometer == CalibrationState::Full
                && gyroscope == CalibrationState::Full
                && system == CalibrationState::Full;
        }
    };

    /**
     * Axes
     */
    enum class Axis : std::uint8_t {
        X = 0,
        Y = 1,
        Z = 2
    };

    /**
     * Creates an IMU connection
     *
     * @param path the path to an I2C device file to use
     * @param address the address of the device on the I2C bus
     */
    IMU(const std::string& path, std::uint8_t address);

    /**
     * Returns the revision of the software on the device
     */
    std::uint16_t software_revision();

    /**
     * Returns the revision of the bootloader
     */
    std::uint8_t bootloader_revision();

    /**
     * Returns the status of the system
     */
    SystemStatus status();

    /**
     * Returns the current system error code
     */
    SystemError error();

    /**
     * Returns the device operating mode
     */
    OperatingMode operating_mode();

    /**
     * Sets the device operating mode
     */
    void set_operating_mode(OperatingMode mode);

    /**
     * Configures whether the device should use an external oscillator
     */
    void set_use_external_oscillator(bool use_external);

    /**
     * Returns the calibration status of the sensors
     */
    Calibration calibration_state();

    /**
     * Configures the remapping of axes
     * Each axis measured by the device can be mapped to another output axis
     * and optionally iverted.
     *
     * @param fromX the output axis to map the device X axis to
     * @param invertX if the X axis should be inverted
     * @param fromY the output axis to map the device Y axis to
     * @param invertY if the Y axis should be inverted
     * @param fromZ the output axis to map the device Z axis to
     * @param invertZ if the Z axis should be inverted
     */
    void remap_axes(Axis fromX, bool invertX, Axis fromY, bool invertY, Axis fromZ, bool invertZ);

    /**
     * Returns the calculated gravity vector, in meters/second^2
     */
    geometry_msgs::Vector3 gravity_vector();

    /**
     * Returns the acceleration vector, including gravity, in meters/second^2
     */
    geometry_msgs::Vector3 acceleration();

    /**
     * Returns the angular velocity, in radians/second
     */
    geometry_msgs::Vector3 angular_velocity();

    /**
     * Returns the orientation of the vehicle as a quaternion
     */
    geometry_msgs::Quaternion orientation();

private:

    /** The I2C connection */
    I2C _i2c;
    /** The current register page, 0 or 1 */
    std::uint8_t _current_page;

    /**
     * Writes a value to a register on the device
     */
    void write_register(Register reg_addr, std::uint8_t value);
    /**
     * Reads a value from a register on the device
     */
    std::uint8_t read_register(Register reg_addr);

    /**
     * Reads one or more consecutive register values starting at the provided
     * address. Stores them in values.
     */
    void read_registers(Register start_addr, std::uint8_t* values, std::size_t length);

    /**
     * Reads a 16-bit value from two adjacent registers, starting at the
     * provided start address, with the least significant byte first.
     */
    std::uint16_t read_u16(Register start_addr);

    /**
     * Sets the register page to 0 or 1
     */
    void set_page(std::uint8_t page);

    /**
     * Checks that a register has an expected value. Throws an exception if it
     * does not.
     */
    void check_register(Register reg_addr, std::uint8_t expected, const std::string& message);
};

// Display for enums
std::ostream& operator << (std::ostream& stream, const IMU::OperatingMode& mode);
std::ostream& operator << (std::ostream& stream, const IMU::SystemStatus& status);
std::ostream& operator << (std::ostream& stream, const IMU::SystemError& error);
std::ostream& operator << (std::ostream& stream, const IMU::CalibrationState& state);
std::ostream& operator << (std::ostream& stream, const IMU::Calibration& calibration);

#endif
