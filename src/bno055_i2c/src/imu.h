#ifndef IMU_H
#define IMU_H
#include <cstdint>
#include <string>
#include <ostream>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

/**
 * An interface to a Bosch BNO055 acceleration and orientation sensor
 *
 * This interface uses I2C for communication.
 */
class IMU {
public:

    /**
     * Forward-declare transport interface
     */
    class Transport;

    /** Register addresses */
    enum class Register : std::uint8_t {
        // Page 0
        CHIP_ID = 0x0,
        ACC_ID = 0x1,
        MAG_ID = 0x2,
        GYR_ID = 0x3,
        SW_REV_ID_LSB = 0x4,
        SW_REV_ID_MSB = 0x5,
        BL_REV_ID = 0x6,
        PAGE_ID = 0x07,
        ACC_DATA_X_LSB = 0x08,
        ACC_DATA_X_MSB = 0x09,
        ACC_DATA_Y_LSB = 0x0A,
        ACC_DATA_Y_MSB = 0x0B,
        ACC_DATA_Z_LSB = 0x0C,
        ACC_DATA_Z_MSB = 0x0D,
        MAG_DATA_X_LSB = 0x0E,
        MAG_DATA_X_MSB = 0x0F,
        MAG_DATA_Y_LSB = 0x10,
        MAG_DATA_Y_MSB = 0x11,
        MAG_DATA_Z_LSB = 0x12,
        MAG_DATA_Z_MSB = 0x13,
        GYR_DATA_X_LSB = 0x14,
        GYR_DATA_X_MSB = 0x15,
        GYR_DATA_Y_LSB = 0x16,
        GYR_DATA_Y_MSB = 0x17,
        GYR_DATA_Z_LSB = 0x18,
        GYR_DATA_Z_MSB = 0x19,
        EUL_DATA_X_LSB = 0x1A,
        EUL_DATA_X_MSB = 0x1B,
        EUL_DATA_Y_LSB = 0x1C,
        EUL_DATA_Y_MSB = 0x1D,
        EUL_DATA_Z_LSB = 0x1E,
        EUL_DATA_Z_MSB = 0x1F,
        QUA_DATA_W_LSB = 0x20,
        QUA_DATA_W_MSB = 0x21,
        QUA_DATA_X_LSB = 0x22,
        QUA_DATA_X_MSB = 0x23,
        QUA_DATA_Y_LSB = 0x24,
        QUA_DATA_Y_MSB = 0x25,
        QUA_DATA_Z_LSB = 0x26,
        QUA_DATA_Z_MSB = 0x27,
        LIA_DATA_X_LSB = 0x28,
        LIA_DATA_X_MSB = 0x29,
        LIA_DATA_Y_LSB = 0x2A,
        LIA_DATA_Y_MSB = 0x2B,
        LIA_DATA_Z_LSB = 0x2C,
        LIA_DATA_Z_MSB = 0x2D,
        GRV_DATA_X_LSB = 0x2E,
        GRV_DATA_X_MSB = 0x2F,
        GRV_DATA_Y_LSB = 0x30,
        GRV_DATA_Y_MSB = 0x31,
        GRV_DATA_Z_LSB = 0x32,
        GRV_DATA_Z_MSB = 0x33,
        TEMP = 0x34,
        CALIB_STAT = 0x35,
        ST_RESULT = 0x36,
        INT_STA = 0x37,
        SYS_CLK_STATUS = 0x38,
        SYS_STATUS = 0x39,
        SYS_ERR = 0x3A,
        UNIT_SEL = 0x3B,
        OPR_MODE = 0x3D,
        PWR_MODE = 0x3E,
        SYS_TRIGGER = 0x3F,
        TEMP_SOURCE = 0x40,
        AXIS_MAP_CONFIG = 0x41,
        AXIS_MAP_SIGN = 0x42,
        ACC_OFFSET_X_LSB = 0x55,
        ACC_OFFSET_X_MSB = 0x56,
        ACC_OFFSET_Y_LSB = 0x57,
        ACC_OFFSET_Y_MSB = 0x58,
        ACC_OFFSET_Z_LSB = 0x59,
        ACC_OFFSET_Z_MSB = 0x5A,
        MAG_OFFSET_X_LSB = 0x5B,
        MAG_OFFSET_X_MSB = 0x5C,
        MAG_OFFSET_Y_LSB = 0x5D,
        MAG_OFFSET_Y_MSB = 0x5E,
        MAG_OFFSET_Z_LSB = 0x5F,
        MAG_OFFSET_Z_MSB = 0x60,
        GYR_OFFSET_X_LSB = 0x61,
        GYR_OFFSET_X_MSB = 0x62,
        GYR_OFFSET_Y_LSB = 0x63,
        GYR_OFFSET_Y_MSB = 0x64,
        GYR_OFFSET_Z_LSB = 0x65,
        GYR_OFFSET_Z_MSB = 0x66,
        ACC_RADIUS_LSB = 0x67,
        ACC_RADIUS_MSB = 0x68,
        MAG_RADIUS_LSB = 0x69,
        MAG_RADIUS_MSB = 0x6A,

        // Page 1
        ACC_Config = 0x08,
        MAG_Config = 0x09,
        GYR_Config_0 = 0x0A,
        GYR_Config_1 = 0x0B,
        ACC_Sleep_Config = 0x0C,
        GYR_Sleep_Config = 0x0D,
        INT_MSK = 0x0F,
        INT_EN = 0x10,
        ACC_AM_THRES = 0x11,
        ACC_INT_Settings = 0x12,
        ACC_HG_DURATION = 0x13,
        ACC_HG_THRES = 0x14,
        ACC_NM_THRES = 0x15,
        ACC_NM_SET = 0x16,
        GYR_INT_SETTING = 0x17,
        GYR_HR_X_SET = 0x18,
        GYR_DUR_X = 0x19,
        GYR_HR_Y_SET = 0x1A,
        GYR_DUR_Y = 0x1B,
        GYR_HR_Z_SET = 0x1C,
        GYR_DUR_Z = 0x1D,
        GYR_AM_THRES = 0x1E,
        GYR_AM_SET = 0x1F
    };

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
     * @param transport the transport mechanism to use
     * This transport mechanism must live at least as long as this I2C object.
     */
    explicit IMU(Transport& transport);

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

    /** The transport mechanism */
    Transport& _transport;

    /** The current register page, 0 or 1 */
    std::uint8_t _current_page;

    /**
     * Reads a value from a register on the device
     */
    std::uint8_t read_register(Register reg_addr);

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
