#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

typedef boost::function<void (const sensor_msgs::Imu::ConstPtr&)> imu_callback_t;

/**
 * Reads yaw from an IMU topic, and publishes a transform from [the frame the
 * IMU data are in] to another frame with no translation and equal orientation.
 *
 * This significantly reduces the rotation of [the frame the IMU data are in]
 * and allows the laser frame to rotate.
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "transform_node");
    ros::NodeHandle handle;

    // The laser frame to publish a transform to
    const auto laser_frame = ros::param::param<std::string>("~laser_frame", "laser");

    // Broadcast transform
    tf2_ros::TransformBroadcaster broadcaster;

    const auto imu_callback = [broadcaster, laser_frame]
        (const sensor_msgs::Imu::ConstPtr& imu_data) mutable {
        // Publish orientation
        geometry_msgs::TransformStamped inverse;
        inverse.header.stamp = imu_data->header.stamp;
        inverse.header.frame_id = imu_data->header.frame_id;
        inverse.child_frame_id = laser_frame;
        // Transform: no translation, orientation equal to the IMU orientation
        auto imu_orientation = tf2::Quaternion(imu_data->orientation.x,
                                                imu_data->orientation.y,
                                                imu_data->orientation.z,
                                                imu_data->orientation.w);
        imu_orientation.normalize();
        inverse.transform.rotation.x = imu_orientation.getX();
        inverse.transform.rotation.y = imu_orientation.getY();
        inverse.transform.rotation.z = imu_orientation.getZ();
        inverse.transform.rotation.w = imu_orientation.getW();

        broadcaster.sendTransform(inverse);
    };

    const auto subscription = handle.subscribe("imu", 16, imu_callback_t(imu_callback));

    ros::spin();
    return 0;
}
