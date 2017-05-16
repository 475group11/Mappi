#include "imu.h"

/**
 * @file imu_output.cpp
 *
 * Contains implementations for printing IMU enumerations
 */

std::ostream& operator << (std::ostream& stream, const IMU::OperatingMode& mode) {
    switch (mode) {
    case IMU::OperatingMode::Config:
        stream << "configuration";
        break;
    case IMU::OperatingMode::AccOnly:
        stream << "accelerometer only";
        break;
    case IMU::OperatingMode::MagOnly:
        stream << "magnetometer only";
        break;
    case IMU::OperatingMode::GyroOnly:
        stream << "gyroscope only";
        break;
    case IMU::OperatingMode::AccMag:
        stream << "accelerometer and magnetometer";
        break;
    case IMU::OperatingMode::AccGyro:
        stream << "accelerometer and gyroscope";
        break;
    case IMU::OperatingMode::MagGyro:
        stream << "magnetometer and gyroscope";
        break;
    case IMU::OperatingMode::AccMagGyro:
        stream << "accelerometer, magnetometer, and gyroscope";
        break;
    case IMU::OperatingMode::Imu:
        stream << "IMU";
        break;
    case IMU::OperatingMode::Compass:
        stream << "compass";
        break;
    case IMU::OperatingMode::MagForGyro:
        stream << "magnetometer for gyroscope";
        break;
    case IMU::OperatingMode::NdofFmcOff:
        stream << "N-DOF with fast magnetometer callibration off";
        break;
    case IMU::OperatingMode::Ndof:
        stream << "N-DOF";
        break;
    }
    return stream;
}

std::ostream& operator << (std::ostream& stream, const IMU::SystemStatus& status) {
    switch (status) {
    case IMU::SystemStatus::Idle:
        stream << "idle";
        break;
    case IMU::SystemStatus::SystemError:
        stream << "system error";
        break;
    case IMU::SystemStatus::PeripheralInit:
        stream << "peripheral initialization";
        break;
    case IMU::SystemStatus::SystemInit:
        stream << "system initialization";
        break;
    case IMU::SystemStatus::SelfTest:
        stream << "self-test";
        break;
    case IMU::SystemStatus::RunningFusion:
        stream << "running with fusion";
        break;
    case IMU::SystemStatus::Running:
        stream << "running";
        break;
    }
    return stream;
}

std::ostream& operator << (std::ostream& stream, const IMU::SystemError& error) {
    switch (error) {
    case IMU::SystemError::Ok:
        stream << "OK";
        break;
    case IMU::SystemError::PeripheralInit:
        stream << "peripheral initialization error";
        break;
    case IMU::SystemError::SystemInit:
        stream << "system initialization error";
        break;
    case IMU::SystemError::SelfTestFailed:
        stream << "self-test failed";
        break;
    case IMU::SystemError::MapValue:
        stream << "register map value out of range";
        break;
    case IMU::SystemError::MapAddress:
        stream << "register map address out of range";
        break;
    case IMU::SystemError::MapWrite:
        stream << "register map write error";
        break;
    case IMU::SystemError::LowPower:
        stream << "BNO low power mode not available";
        break;
    case IMU::SystemError::AccelerometerPowerMode:
        stream << "accelerometer power mode not available";
        break;
    case IMU::SystemError::FusionConfig:
        stream << "fusion algorithm configuration error";
        break;
    case IMU::SystemError::SensorConfig:
        stream << "sensor configuration error";
        break;
    }
    return stream;
}


std::ostream& operator << (std::ostream& stream, const IMU::CalibrationState& state) {
    switch (state) {
    case IMU::CalibrationState::None:
        stream << "none";
        break;
    case IMU::CalibrationState::Full:
        stream << "full";
        break;
    default:
        stream << "other (" << int(state) << ')';
    }
    return stream;
}
std::ostream& operator << (std::ostream& stream, const IMU::Calibration& calibration) {
    stream << "{ accelerometer: " << calibration.accelerometer
        << ", gyroscope: " << calibration.gyroscope
        << ", magnetometer: " << calibration.magnetometer
        << ", system: " << calibration.system << " }";
    return stream;
}
