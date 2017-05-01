#include "take_back_half.h"
#include <cmath>

namespace {

inline double clamp_motor_power(double power) {
    if (power > 1.0) {
        return 1.0;
    } else if (power < 0.0) {
        return 0.0;
    } else {
        return power;
    }
}

}

TakeBackHalf::TakeBackHalf(double gain, double max_speed) :
    _current_speed(0),
    _target_speed(0),
    _motor_power(0),
    _gain(gain),
    _last_error(0),
    _take_back_half(0),
    _max_speed(max_speed)
{

}

double TakeBackHalf::get_motor_power() const {
    return _motor_power;
}

void TakeBackHalf::set_current_speed(double current_speed) {
    _current_speed = current_speed;
}

void TakeBackHalf::set_target_speed(double new_target_speed) {
    if (_target_speed != new_target_speed) {
        // Optimize spinup
        if (_target_speed < new_target_speed) {
            _last_error = 1.0;
        } else if (_target_speed > new_target_speed) {
            _last_error = -1.0;
        }
        _take_back_half = 2 * (new_target_speed / _max_speed) - 1;

        _target_speed = new_target_speed;
    }
}

void TakeBackHalf::run(double delta_time_seconds) {
    const auto error = (_target_speed - _current_speed) * delta_time_seconds;
    _motor_power = clamp_motor_power(_motor_power + error * _gain);
    // Check error sign change
    if (std::signbit(_last_error) != std::signbit(error)) {
        _motor_power = clamp_motor_power(0.5 * (_motor_power + _take_back_half));
        _take_back_half = _motor_power;
        _last_error = error;
    }
}
