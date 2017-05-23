#include <string>
#include <iostream>

#include <boost/optional.hpp>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <mappi_msgs/PositionVelocityAcceleration.h>

namespace {

/**
 * Differentiates a vector
 *
 * @param previous the previous value
 * @param current the current value
 * @param dt the time between the prevous and current values
 *
 * @return the derivative between previous and current
 */
geometry_msgs::Vector3 vector_derivative(const geometry_msgs::Vector3& previous, const geometry_msgs::Vector3& current, const ros::Duration& dt);

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "velocity_acceleration_node");
    ros::NodeHandle handle;

    // Transform check rate
    // TODO: Possibly subscribe to a topic published by the SLAM node and ignore
    // the messages, to synchronize updates with SLAM position updates
    ros::Rate rate(10);

    // Parameters

    // The parent frame
    const std::string parent_frame = ros::param::param<std::string>("~parent_frame", "map");
    // The child frame (this node calculates the velocity/acceleration of this
    // frame relative to the parent frame)
    const std::string child_frame = ros::param::param<std::string>("~child_frame", "base_link");
    // The topic on which to publish the position/velocity/acceleration
    const std::string pva_topic = ros::param::param<std::string>("~pva_topic", "pva");

    // Message publisher
    auto publisher = handle.advertise<mappi_msgs::PositionVelocityAcceleration>(pva_topic, 16);

    // Receive transform
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener transform_listener(buffer);

    // The most recently received transform
    boost::optional<geometry_msgs::Vector3> last_position = boost::none;
    // The most recent acceleration
    boost::optional<geometry_msgs::Vector3> last_velocity = boost::none;

    const auto timer_callback = [&last_position, &last_velocity, &buffer,
        &publisher, pva_topic, parent_frame, child_frame]
        (const ros::TimerEvent& event) {
        const auto interval = event.current_real - event.last_real;
        try {
            // Look up latest transform
            const geometry_msgs::TransformStamped transform_stamped = buffer.lookupTransform(child_frame, parent_frame, ros::Time(0));
            const geometry_msgs::Vector3 position = std::move(transform_stamped.transform.translation);

            if (last_position) {
                // Calculate velocity
                const geometry_msgs::Vector3 velocity = vector_derivative(*last_position, position, interval);
                if (last_velocity) {
                    // Calculate acceleration
                    const geometry_msgs::Vector3 acceleration = vector_derivative(*last_velocity, velocity, interval);

                    // Publish result
                    mappi_msgs::PositionVelocityAcceleration pva;
                    pva.header.stamp = transform_stamped.header.stamp;
                    pva.header.frame_id = parent_frame;
                    pva.child_frame_id = child_frame;
                    pva.position = position;
                    pva.velocity = velocity;
                    pva.acceleration = std::move(acceleration);
                    publisher.publish(pva);
                }
                last_velocity = std::move(velocity);
            }
            last_position = std::move(position);

        } catch (tf2::TransformException& e) {
            ROS_WARN("Failed to get transform: %s", e.what());
        }
    };

    auto timer = handle.createTimer(rate.expectedCycleTime(), timer_callback);
    timer.start();
    ros::spin();
    return 0;
}


namespace {
geometry_msgs::Vector3 vector_derivative(const geometry_msgs::Vector3& previous, const geometry_msgs::Vector3& current, const ros::Duration& dt) {
    const auto seconds = dt.toSec();
    geometry_msgs::Vector3 derivative;

    derivative.x = (current.x - previous.x) / seconds;
    derivative.y = (current.y - previous.y) / seconds;
    derivative.z = (current.z - previous.z) / seconds;

    return derivative;
}
}
