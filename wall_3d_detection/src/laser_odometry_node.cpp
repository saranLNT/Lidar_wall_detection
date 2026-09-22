#include <wall_3d_detection/laser_odometry.hpp>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

LaserOdometry::LaserOdometry()
: rclcpp::Node("laser_odometry"),
  has_imu_orientation_(false),
  imu_roll_(0.0),
  imu_pitch_(0.0),
    has_initial_laser_rotation_(false),
    initial_laser_rotation_(Eigen::Matrix3f::Identity()),
  has_previous_(false),
  cumulative_transform_(Eigen::Matrix4f::Identity()),
  last_keyframe_transform_(Eigen::Matrix4f::Identity())
{
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    laser_frame_ = this->declare_parameter<std::string>("laser_frame", "laser");
    max_correspondence_distance_ = this->declare_parameter<double>("max_correspondence_distance", 0.3);
    max_iterations_ = this->declare_parameter<int>("max_iterations", 50);
    fitness_threshold_ = this->declare_parameter<double>("fitness_threshold", 0.05);
    voxel_leaf_size_ = this->declare_parameter<double>("voxel_leaf_size", 0.05);
    keyframe_translation_threshold_ = this->declare_parameter<double>("keyframe_translation_threshold", 0.1);
    keyframe_rotation_threshold_ = this->declare_parameter<double>("keyframe_rotation_threshold", 0.1);
    use_imu_orientation_ = this->declare_parameter<bool>("use_imu_orientation", true);
    imu_rp_alpha_ = this->declare_parameter<double>("imu_rp_alpha", 0.98);
    imu_x_ = this->declare_parameter<double>("imu_x", 0.0);
    imu_y_ = this->declare_parameter<double>("imu_y", 0.0);
    imu_z_ = this->declare_parameter<double>("imu_z", 0.0);
    imu_yaw_ = this->declare_parameter<double>("imu_yaw", 0.0);
    imu_pitch_offset_ = this->declare_parameter<double>("imu_pitch", 0.0);
    imu_roll_offset_ = this->declare_parameter<double>("imu_roll", 0.0);

    lidar_x_ = this->declare_parameter<double>("lidar_x", 0.0);
    lidar_y_ = this->declare_parameter<double>("lidar_y", 0.0);
    lidar_z_ = this->declare_parameter<double>("lidar_z", 0.0);
    lidar_roll_ = this->declare_parameter<double>("lidar_roll", -1.57079632679);
    lidar_pitch_ = this->declare_parameter<double>("lidar_pitch", 0.0);
    lidar_yaw_ = this->declare_parameter<double>("lidar_yaw", 0.0);

    tf2::Quaternion mount_q;
    mount_q.setRPY(lidar_roll_, lidar_pitch_, lidar_yaw_);
    mount_quat_ = mount_q;
    tf2::Matrix3x3 mount_r(mount_q);
    base_to_laser_ = Eigen::Matrix4f::Identity();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            base_to_laser_(r, c) = static_cast<float>(mount_r[r][c]);
        }
    }
    base_to_laser_(0, 3) = static_cast<float>(lidar_x_);
    base_to_laser_(1, 3) = static_cast<float>(lidar_y_);
    base_to_laser_(2, 3) = static_cast<float>(lidar_z_);

    cumulative_transform_ = base_to_laser_;
    last_keyframe_transform_ = base_to_laser_;

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    map_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/scan_pointcloud",
        rclcpp::SensorDataQoS(),
        std::bind(&LaserOdometry::cloudCallback, this, std::placeholders::_1));

    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data",
        rclcpp::SensorDataQoS(),
        std::bind(&LaserOdometry::imuCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "Laser Odometry started. odom_frame: %s, laser_frame: %s. "
                "Using frame-to-map ICP: each scan registers against the accumulated 3D map "
                "(not just the previous scan), which helps constrain the axis perpendicular "
                "to the LiDAR's scanning plane once out-of-plane structure accumulates. "
                "A single flat scan is still geometrically degenerate along its own normal "
                "until the map gains 3D structure from actual out-of-plane motion. "
                "use_imu_orientation=%s blends IMU roll/pitch (gravity vector) into the "
                "ICP result to directly fix that degenerate axis.",
                odom_frame_.c_str(), laser_frame_.c_str(),
                use_imu_orientation_ ? "true" : "false");
}

void LaserOdometry::imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    tf2::Quaternion imu_to_odom(
        imu_msg->orientation.x,
        imu_msg->orientation.y,
        imu_msg->orientation.z,
        imu_msg->orientation.w);

    tf2::Quaternion laser_to_imu;
    laser_to_imu.setRPY(imu_roll_offset_, imu_pitch_offset_, imu_yaw_);

    tf2::Quaternion body_to_imu = mount_quat_ * laser_to_imu;
    tf2::Quaternion body_to_odom = imu_to_odom * body_to_imu.inverse();

    double yaw;
    tf2::Matrix3x3(body_to_odom).getRPY(imu_roll_, imu_pitch_, yaw);
    (void)yaw;

    has_imu_orientation_ = true;
}

void LaserOdometry::applyImuRollPitch(Eigen::Matrix4f & transform)
{
    if (!use_imu_orientation_ || !has_imu_orientation_) {
        return;
    }

    Eigen::Matrix4f odom_to_base = transform * base_to_laser_.inverse();

    tf2::Matrix3x3 body_rotation(
        odom_to_base(0, 0), odom_to_base(0, 1), odom_to_base(0, 2),
        odom_to_base(1, 0), odom_to_base(1, 1), odom_to_base(1, 2),
        odom_to_base(2, 0), odom_to_base(2, 1), odom_to_base(2, 2));

    double body_roll, body_pitch, body_yaw;
    body_rotation.getRPY(body_roll, body_pitch, body_yaw);

    double fused_roll = imu_rp_alpha_ * imu_roll_ + (1.0 - imu_rp_alpha_) * body_roll;
    double fused_pitch = imu_rp_alpha_ * imu_pitch_ + (1.0 - imu_rp_alpha_) * body_pitch;

    tf2::Quaternion fused_q;
    fused_q.setRPY(fused_roll, fused_pitch, body_yaw);
    tf2::Matrix3x3 fused_rotation(fused_q);

    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            odom_to_base(r, c) = static_cast<float>(fused_rotation[r][c]);
        }
    }

    transform = odom_to_base * base_to_laser_;
}

bool LaserOdometry::shouldInsertKeyframe(const Eigen::Matrix4f & current_transform) const
{
    Eigen::Matrix4f delta = last_keyframe_transform_.inverse() * current_transform;
    float translation = delta.block<3, 1>(0, 3).norm();
    Eigen::AngleAxisf angle_axis(Eigen::Matrix3f(delta.block<3, 3>(0, 0)));
    float rotation = std::abs(angle_axis.angle());

    return translation > keyframe_translation_threshold_ ||
           rotation > keyframe_rotation_threshold_;
}

void LaserOdometry::broadcastTransform(
    const Eigen::Matrix4f & transform,
    const rclcpp::Time & stamp)
{
    Eigen::Matrix3f rotation = transform.block<3, 3>(0, 0);
    Eigen::Quaternionf q(rotation);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = laser_frame_;

    tf_msg.transform.translation.x = transform(0, 3);
    tf_msg.transform.translation.y = transform(1, 3);
    tf_msg.transform.translation.z = transform(2, 3);

    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf_msg);
}

void LaserOdometry::cloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
    if (!cloud_msg) {
        RCLCPP_WARN(this->get_logger(), "Received null PointCloud2 message");
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    current_cloud->reserve(cloud_msg->width * cloud_msg->height);

    sensor_msgs::PointCloud2ConstIterator<float> x_iter(*cloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y_iter(*cloud_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z_iter(*cloud_msg, "z");

    for (; x_iter != x_iter.end(); ++x_iter, ++y_iter, ++z_iter) {
        current_cloud->push_back(pcl::PointXYZ(*x_iter, *y_iter, *z_iter));
    }

    std::vector<int> nan_indices;
    pcl::removeNaNFromPointCloud(*current_cloud, *current_cloud, nan_indices);

    if (current_cloud->empty()) {
        RCLCPP_WARN(this->get_logger(), "Current cloud is empty after NaN removal");
        return;
    }

    if (!has_previous_) {
        applyImuRollPitch(cumulative_transform_);
        pcl::transformPointCloud(*current_cloud, *map_cloud_, cumulative_transform_);
        has_previous_ = true;
        last_keyframe_transform_ = cumulative_transform_;
        broadcastTransform(cumulative_transform_, cloud_msg->header.stamp);
        return;
    }

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(current_cloud);
    icp.setInputTarget(map_cloud_);
    icp.setMaxCorrespondenceDistance(max_correspondence_distance_);
    icp.setMaximumIterations(max_iterations_);

    pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
    icp.align(aligned_cloud, cumulative_transform_);

    if (icp.hasConverged() && icp.getFitnessScore() < fitness_threshold_) {

        cumulative_transform_ = icp.getFinalTransformation();
        applyImuRollPitch(cumulative_transform_);

        if (shouldInsertKeyframe(cumulative_transform_)) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_odom(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::transformPointCloud(*current_cloud, *cloud_in_odom, cumulative_transform_);
            *map_cloud_ += *cloud_in_odom;

            pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
            voxel_filter.setInputCloud(map_cloud_);
            voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
            voxel_filter.filter(*map_cloud_);

            last_keyframe_transform_ = cumulative_transform_;
        }

    } else {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "ICP failed to converge (fitness=%f), reusing last known pose",
            icp.getFitnessScore());
    }

    broadcastTransform(cumulative_transform_, cloud_msg->header.stamp);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserOdometry>());
    rclcpp::shutdown();
    return 0;
}