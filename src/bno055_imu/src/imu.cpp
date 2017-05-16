#include "imu.h"
#include "imu_private.h"
#include "imu_transport.h"
#include <cmath>
#include <stdexcept>
#include <sstream>
#include <chrono>
#include <thread>

#include <ros/ros.h>

IMU::IMU(IMU::Transport& transport) :
    _transport(transport),
    _current_page(0)
{
    // Check chip IDs
    check_register(Register::CHIP_ID, BNO_CHIP_ID, "Invalid chip ID");
    check_register(Register::ACC_ID, BNO_ACC_ID, "Invalid accelerometer ID");
    check_register(Register::MAG_ID, BNO_MAG_ID, "Invalid magnetometer ID");
    check_register(Register::GYR_ID, BNO_GYR_ID, "Invalid gyroscope ID");
    
    // Wait 100 ms
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Configure
    set_operating_mode(OperatingMode::Config);
    set_use_external_oscillator(true);
    // Set gyroscope units to radians/second
    auto unit_sel = read_register(Register::UNIT_SEL);
    unit_sel |= 2;
    _transport.write_register(Register::UNIT_SEL, unit_sel);
    set_operating_mode(OperatingMode::Ndof);
    const auto current_status = status();
    if (current_status == SystemStatus::SystemError) {
        ROS_ERROR_STREAM("IMU internal error: " << error());
    }
    
    ROS_INFO("IMU configured");

}

std::uint16_t IMU::software_revision() {
    set_page(0);
    return read_u16(Register::SW_REV_ID_LSB);
}

std::uint8_t IMU::bootloader_revision() {
    set_page(0);
    return read_register(Register::BL_REV_ID);
}

IMU::SystemStatus IMU::status() {
    set_page(0);
    return static_cast<SystemStatus>(read_register(Register::SYS_STATUS));
}
IMU::SystemError IMU::error() {
    set_page(0);
    return static_cast<SystemError>(read_register(Register::SYS_ERR));
}

IMU::OperatingMode IMU::operating_mode() {
    set_page(0);
    return static_cast<OperatingMode>(read_register(Register::OPR_MODE) & 0xF);
}

void IMU::set_operating_mode(OperatingMode mode) {
    set_page(0);
    _transport.write_register(Register::OPR_MODE, std::uint8_t(mode));
}

void IMU::set_use_external_oscillator(bool use_external) {
    set_page(0);
    uint8_t value = 0;
    if (use_external) {
        value |= 1 << 7;
    }
    _transport.write_register(Register::SYS_TRIGGER, value);
}

IMU::Calibration IMU::calibration_state() {
    set_page(0);
    const auto reg_value = read_register(Register::CALIB_STAT);
    const auto magnetometer = static_cast<CalibrationState>(reg_value & 3);
    const auto accelerometer = static_cast<CalibrationState>((reg_value >> 2) & 3);
    const auto gyroscope = static_cast<CalibrationState>((reg_value >> 4) & 3);
    const auto system = static_cast<CalibrationState>((reg_value >> 6) & 3);
    return { magnetometer, accelerometer, gyroscope, system };
}

void IMU::remap_axes(Axis fromX, bool invertX, Axis fromY, bool invertY, Axis fromZ, bool invertZ) {
    set_page(0);
    const std::uint8_t map_config = (std::uint8_t(fromZ) << 4)
                                    | (std::uint8_t(fromY) << 2)
                                    | std::uint8_t(fromX);
    std::uint8_t sign = 0;
    if (invertX) {
        sign |= 1 << 2;
    }
    if (invertY) {
        sign |= 1 << 1;
    }
    if (invertZ) {
        sign |= 1;
    }
    _transport.write_register(Register::AXIS_MAP_CONFIG, map_config);
    _transport.write_register(Register::AXIS_MAP_SIGN, sign);
    check_register(Register::AXIS_MAP_CONFIG, map_config, "Failed to set axis map");
    check_register(Register::AXIS_MAP_SIGN, sign, "Failed to set axis sign");
}

geometry_msgs::Vector3 IMU::gravity_vector() {
    set_page(0);
    const auto x_int = std::int16_t(read_u16(Register::GRV_DATA_X_LSB));
    const auto y_int = std::int16_t(read_u16(Register::GRV_DATA_Y_LSB));
    const auto z_int = std::int16_t(read_u16(Register::GRV_DATA_Z_LSB));
    geometry_msgs::Vector3 gravity;
    gravity.x = double(x_int) / 100.0;
    gravity.y = double(y_int) / 100.0;
    gravity.z = double(z_int) / 100.0;
    return gravity;
}

geometry_msgs::Vector3 IMU::acceleration() {
    set_page(0);
    const auto x = std::int16_t(read_u16(Register::ACC_DATA_X_LSB));
    const auto y = std::int16_t(read_u16(Register::ACC_DATA_Y_LSB));
    const auto z = std::int16_t(read_u16(Register::ACC_DATA_Z_LSB));
    geometry_msgs::Vector3 acceleration;
    acceleration.x = double(x) / 100.0;
    acceleration.y = double(y) / 100.0;
    acceleration.z = double(z) / 100.0;
    return acceleration;
}

geometry_msgs::Quaternion IMU::orientation() {
    set_page(0);
    const auto w = std::int16_t(read_u16(Register::QUA_DATA_W_LSB));
    const auto x = std::int16_t(read_u16(Register::QUA_DATA_X_LSB));
    const auto y = std::int16_t(read_u16(Register::QUA_DATA_Y_LSB));
    const auto z = std::int16_t(read_u16(Register::QUA_DATA_Z_LSB));
    // Conversion: 1 quaternion unit = 2^14 register units
    const double factor = std::exp2(-14);
    geometry_msgs::Quaternion orientation;
    orientation.w = double(w) * factor;
    orientation.x = double(x) * factor;
    orientation.y = double(y) * factor;
    orientation.z = double(z) * factor;
    return orientation;
}

geometry_msgs::Vector3 IMU::angular_velocity() {
    set_page(0);
    const auto x = std::int16_t(read_u16(Register::GYR_DATA_X_LSB));
    const auto y = std::int16_t(read_u16(Register::GYR_DATA_Y_LSB));
    const auto z = std::int16_t(read_u16(Register::GYR_DATA_Z_LSB));
    // 500 units approximately equal to 1 radian/second
    const double factor = std::exp2(-9);
    geometry_msgs::Vector3 velocity;
    velocity.x = double(x) * factor;
    velocity.y = double(y) * factor;
    velocity.z = double(z) * factor;
    return velocity;
}

void IMU::set_page(std::uint8_t page) {
    if (page == 0 || page == 1) {
        if (page != _current_page) {
            _transport.write_register(Register::PAGE_ID, page);
            _current_page = page;
        }
    } else {
        throw std::invalid_argument("Page number not 0 or 1");
    }
}

std::uint16_t IMU::read_u16(Register start_addr) {
    // less significant, then more significant
    std::uint8_t bytes[2];
    _transport.read_registers(start_addr, bytes, sizeof bytes);
    const auto lower = bytes[0];
    const auto upper = bytes[1];
    return std::uint16_t(lower) | (std::uint16_t(upper) << 8);
}

std::uint8_t IMU::read_register(Register reg_addr) {
    std::uint8_t value;
    _transport.read_registers(reg_addr, &value, 1);
    return value;
}

void IMU::check_register(Register reg_addr, uint8_t expected, const std::string& message) {
    const auto value = read_register(reg_addr);
    if (value != expected) {
        std::stringstream stream;
        stream << std::hex;
        stream << message << ": expected 0x" << int(expected) << ", got 0x" << int(value);
        throw std::runtime_error(stream.str());
    }
}
