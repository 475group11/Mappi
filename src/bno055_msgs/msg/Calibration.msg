# A message with information about the calibration status of the BNO055

# Indicates a sensor that is not calibrated at all
uint8 CALIBRATION_NONE = 0
# Indicates a sensor that is partially calibrated
uint8 CALIBRATION_SOME = 1
# Indicates a sensor that is partially calibrated, but more calibrated than
# CALIBRATION_SOME
uint8 CALIBRATION_MOST = 2
# Indicates a fully calibrated sensor
uint8 CALIBRATION_FULL = 3


# Calibration state of accelerometer
uint8 accelerometer_status
# Calibration state of gyroscope sensor
uint8 gyroscope_status
# Calibration state of magnetometer
uint8 magnetometer_status
# Calibration state of the whole system/fusion
uint8 system_status
