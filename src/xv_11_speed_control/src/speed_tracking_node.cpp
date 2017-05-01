#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>

typedef boost::function<void (const std_msgs::UInt16::ConstPtr&)> rpm_callback_t;

/**
 * Reads sensor RPM messages from an xv_11_laser_driver neato_laser_publisher
 * node. When messages are received, converts them into Float64 messages and
 * forwards them. When no message is received for a specified duration,
 * sends a message with a value of zero.
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "speed_tracking_node");
    ros::NodeHandle handle;

    // Parameters
    const auto rpm_in_topic = ros::param::param<std::string>("~rpm_in_topic", "rpms");
    const auto rpm_out_topic = ros::param::param<std::string>("~rpm_out_topic", "rpm_filtered");

    // The maximum expected time between incoming speed updates
    const ros::Duration timeout_duration(0.5);

    // Advertise f64 RPM messages
    const auto rpm_publisher = handle.advertise<std_msgs::Float64>(rpm_out_topic, 4);

    const auto timeout_callback = [&rpm_publisher] (const ros::TimerEvent&) {
        ROS_DEBUG("Speed tracking timed out");
        std_msgs::Float64 message;
        message.data = 0.0;
        rpm_publisher.publish(message);
    };
    auto timer = handle.createTimer(timeout_duration, timeout_callback);

    const auto rpm_callback = [&timer, &rpm_publisher] (const std_msgs::UInt16::ConstPtr& message) {
        // Stop and restart the timer, effectively resetting it
        timer.stop();
        timer.start();
        const auto rpm = message->data;
        // Ignore messages with speeds that are too high
        if (rpm > 400) {
            return;
        }
        // Convert and send the RPM
        std_msgs::Float64 forward_message;
        forward_message.data = static_cast<double>(rpm);
        rpm_publisher.publish(forward_message);
    };
    const auto subscription = handle.subscribe(rpm_in_topic, 0, rpm_callback_t(rpm_callback));

    timer.start();

    ros::spin();
    return 0;
}
