#ifndef LIDAR_ODOMETRY_FASTGICP_LIDAR_ODOMETRY_HPP
#define LIDAR_ODOMETRY_FASTGICP_LIDAR_ODOMETRY_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>

#include <Eigen/Dense>
#include <memory>
#include <string>

namespace lidar_odometry_fastgicp
{

class LidarOdometry : public rclcpp::Node
{
public:
    explicit LidarOdometry(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~LidarOdometry() = default;

private:
    // ROS2 interfaces
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Parameters
    std::string input_topic_;
    std::string output_topic_;
    std::string base_frame_;
    std::string odom_frame_;
    double voxel_size_;
    bool use_cuda_;
    int max_iterations_;
    double transformation_epsilon_;
    double euclidean_fitness_epsilon_;
    double max_correspondence_distance_;

    // Registration algorithm
    std::shared_ptr<fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ>> registration_;

    // Point clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud_;
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;

    // Odometry state
    Eigen::Matrix4f accumulated_transform_;
    bool is_first_cloud_;
    rclcpp::Time last_timestamp_;

    // Callbacks
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // Helper functions
    void initializeParameters();
    void initializeRegistration();
    pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessPointCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void publishOdometry(const Eigen::Matrix4f& transform, const std_msgs::msg::Header& header);
    void publishTransform(const Eigen::Matrix4f& transform, const std_msgs::msg::Header& header);
    geometry_msgs::msg::TransformStamped eigenToTransformStamped(
        const Eigen::Matrix4f& transform, const std_msgs::msg::Header& header);
    nav_msgs::msg::Odometry eigenToOdometry(
        const Eigen::Matrix4f& transform, const std_msgs::msg::Header& header);
};

} // namespace lidar_odometry_fastgicp

#endif // LIDAR_ODOMETRY_FASTGICP_LIDAR_ODOMETRY_HPP
