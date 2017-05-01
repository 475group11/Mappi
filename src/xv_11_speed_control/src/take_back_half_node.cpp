#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include "take_back_half.h"

typedef boost::function<void (const std_msgs::Float64::ConstPtr&)> speed_callback_t;

int main(int argc, char** argv) {
    ros::init(argc, argv, "take_back_half_node");
    ros::NodeHandle handle;
    
    // Parameters
    // Approximate speed (RPM) at full power
    const auto max_rpm = ros::param::param<double>("~max_rpm", 300.0);
    // Gain
    const auto gain = ros::param::param<double>("~gain", 1.0e-5);
    // Speed input topic
    const auto current_speed_topic = ros::param::param<std::string>("~current_speed_topic", "current_speed");
    // Target speed input topic
    const auto target_speed_topic = ros::param::param<std::string>("~target_speed_topic", "target_speed");
    // Motor power output topic
    const auto power_output_topic = ros::param::param<std::string>("~power_output_topic", "power");
    
    // Create controller
    TakeBackHalf take_back_half(gain, max_rpm);
    
    const auto current_speed_callback = [&take_back_half] (const std_msgs::Float64::ConstPtr& message) {
        take_back_half.set_current_speed(message->data);
    };
    // Subscribe to current speed speed
    auto current_speed_subscription = handle.subscribe(current_speed_topic, 0, speed_callback_t(current_speed_callback));
    
    
    const auto target_speed_callback = [&take_back_half] (const std_msgs::Float64::ConstPtr& message) {
        take_back_half.set_target_speed(message->data);
    };
    // Subscribe to current speed speed
    auto target_speed_subscription = handle.subscribe(target_speed_topic, 0, speed_callback_t(target_speed_callback));
    
    
    // Advertise power
    auto publisher = handle.advertise<std_msgs::Float64>(power_output_topic, 0);
    
    const auto loop_callback = [&take_back_half, &publisher] (const ros::TimerEvent& event) {
    	const auto interval = event.current_real - event.last_real;
    	take_back_half.run(interval.toSec());
    	
    	std_msgs::Float64 message;
    	message.data = take_back_half.get_motor_power();
    	publisher.publish(message);
    };

    ros::Rate loop_rate(100);
    auto timer = handle.createTimer(loop_rate.expectedCycleTime(), loop_callback);
    timer.start();
    ros::spin();
}