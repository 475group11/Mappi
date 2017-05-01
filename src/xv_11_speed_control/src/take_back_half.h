#ifndef TAKE_BACK_HALF_H
#define TAKE_BACK_HALF_H

/**
 * A take-back-half closed-loop speed controller
 *
 * Reference: https://www.chiefdelphi.com/media/papers/2674
 */
class TakeBackHalf {
private:

    /** The current speed */
    double _current_speed;

    /** The target speed */
    double _target_speed;
    
    /** The current power provided to the motor, 0-1 */
    double _motor_power;
    
    /** The gain */
    const double _gain;
    
    /** The error encountered during the last loop iteration */
    double _last_error;
    
    /** The take-back-half variable */
    double _take_back_half;
    
    /** The approximate speed at full power */
    const double _max_speed;

public:

    /**
     * @param gain the gain to use for control
     * @param max_speed the approximate speed of the system
     * at full power
     */
    explicit TakeBackHalf(double gain, double max_speed);
    
    void set_current_speed(double current_speed);
    void set_target_speed(double target_speed);
    
    double get_motor_power() const;
    
    /**
     * Runs the speed control algorithm.
     * This should be called frequently.
     * @param delta_time_seconds the time in seconds since
     * this function was last called
     */
    void run(double delta_time_seconds);

};


#endif
