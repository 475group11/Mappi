#include <cstring>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include "rpiPWM1/rpiPWM1.h"

typedef boost::function<void (const std_msgs::Float64::ConstPtr&)> duty_cycle_callback_t;

/**
 * Interfaces with the Raspberry Pi PWM module
 *
 * Subscribes to the topic provided in the parameter "duty_cycle_topic". Each
 * Float64 message on that topic is interpreted as a duty cycle (0 - 1) and
 * output on the pin connected to PWM 1.
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "pwm_node");
    ros::NodeHandle handle;

    // Parameters
    const std::string duty_cycle_topic = ros::param::param<std::string>("duty_cycle_topic", "/pwm_duty_cycle");
    const double frequency = ros::param::param<double>("pwm_frequency", 100);

    if (frequency < 0) {
        ROS_ERROR("PWM frequency may not be less than 0");
        return -9;
    }

    // Set up PWM with initial frequency and zero duty cycle
    rpiPWM1 pwm;
    const auto duty_cycle_result = pwm.setDutyCycle(1e-5);
    if (duty_cycle_result != 0) {
        ROS_ERROR("Failed to initialize PWM duty cycle to zero: %s", std::strerror(duty_cycle_result));
        return -3;
    }
    const auto frequency_result = pwm.setFrequency(frequency);
    if (frequency_result != 0) {
        ROS_ERROR("Failed to set PWM frequency to %f Hz: %s", frequency, std::strerror(frequency_result));
        return -3;
    }

    // Callback
    auto duty_cycle_callback = [&pwm] (const std_msgs::Float64::ConstPtr& message) {
        double duty_ratio = message->data;
        if (duty_ratio > 1) {
            ROS_WARN("PWM duty cycle %f is greater than 1, ignoring", message->data);
            return;
        }
        if (duty_ratio < 0) {
            ROS_WARN("PWM duty cycle %f is less than 0, ignoring", message->data);
            return;
        }
        // Clamp to 1e-5, because the PWM code will not accept zero
        if (duty_ratio < 1e-5) {
            duty_ratio = 1e-5;
        }
        // Multiply by 100 to convert from a ratio to a percentage
        const auto result = pwm.setDutyCycle(duty_ratio * 100.0);
        if (result != 0) {
            ROS_WARN("Failed to set PWM duty cycle to %f: %s", duty_ratio, std::strerror(result));
        }
    };
    auto subscription = handle.subscribe(duty_cycle_topic, 0, duty_cycle_callback_t(duty_cycle_callback));

    ros::spin();

    return 0;
}
