#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

typedef boost::function<void (const sensor_msgs::Imu::ConstPtr&)> imu_callback_t;

/**
 * Reads yaw from an IMU topic, and publishes a transform from a parent frame
 * to a child frame with no translation and the orientation of the IMU.
 *
 * This significantly reduces the rotation of the parent frame
 * and allows the child frame to rotate.
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "transform_node");
    ros::NodeHandle handle;

    const auto parent_frame = ros::param::param<std::string>("~parent_frame", "base_link");
    const auto child_frame = ros::param::param<std::string>("~child_frame", "laser");

    // Broadcast transform
    tf2_ros::TransformBroadcaster broadcaster;

    const auto imu_callback = [broadcaster, parent_frame, child_frame]
        (const sensor_msgs::Imu::ConstPtr& imu_data) mutable {
        // Publish orientation
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = ros::Time::now();
        transform.header.frame_id = parent_frame;
        transform.child_frame_id = child_frame;
        // Transform: no translation, orientation equal to the IMU orientation
        auto imu_orientation = tf2::Quaternion(imu_data->orientation.x,
                                                imu_data->orientation.y,
                                                imu_data->orientation.z,
                                                imu_data->orientation.w);
        imu_orientation.normalize();
        transform.transform.rotation.x = imu_orientation.getX();
        transform.transform.rotation.y = imu_orientation.getY();
        transform.transform.rotation.z = imu_orientation.getZ();
        transform.transform.rotation.w = imu_orientation.getW();
        broadcaster.sendTransform(transform);
    };

    const auto subscription = handle.subscribe("imu", 16, imu_callback_t(imu_callback));

    ros::spin();
    return 0;
}
