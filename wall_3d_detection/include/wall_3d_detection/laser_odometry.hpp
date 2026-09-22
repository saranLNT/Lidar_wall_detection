#ifndef WALL_3D_DETECTION__LASER_ODOMETRY_HPP_
#define WALL_3D_DETECTION__LASER_ODOMETRY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>

class LaserOdometry : public rclcpp::Node
{
public:
    LaserOdometry();

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::string odom_frame_;
    std::string laser_frame_;

    double max_correspondence_distance_;
    int max_iterations_;
    double fitness_threshold_;
    double voxel_leaf_size_;
    double keyframe_translation_threshold_;
    double keyframe_rotation_threshold_;

    bool use_imu_orientation_;
    double imu_rp_alpha_;
    bool has_imu_orientation_;
    double imu_roll_;
    double imu_pitch_;
    double imu_x_;
    double imu_y_;
    double imu_z_;
    double imu_yaw_;
    double imu_pitch_offset_;
    double imu_roll_offset_;
    bool has_initial_laser_rotation_;
    Eigen::Matrix3f initial_laser_rotation_;

    double lidar_x_;
    double lidar_y_;
    double lidar_z_;
    double lidar_roll_;
    double lidar_pitch_;
    double lidar_yaw_;
    Eigen::Matrix4f base_to_laser_;
    tf2::Quaternion mount_quat_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
    bool has_previous_;

    Eigen::Matrix4f cumulative_transform_;
    Eigen::Matrix4f last_keyframe_transform_;

    void applyImuRollPitch(Eigen::Matrix4f & transform);

    void broadcastTransform(
        const Eigen::Matrix4f & transform,
        const rclcpp::Time & stamp);

    bool shouldInsertKeyframe(const Eigen::Matrix4f & current_transform) const;
};

#endif  // WALL_3D_DETECTION__LASER_ODOMETRY_HPP_