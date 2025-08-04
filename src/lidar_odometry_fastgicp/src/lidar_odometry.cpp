#include "lidar_odometry_fastgicp/lidar_odometry.hpp"

namespace lidar_odometry_fastgicp
{

LidarOdometry::LidarOdometry(const rclcpp::NodeOptions & options)
: Node("lidar_odometry_node", options),
  current_cloud_(new pcl::PointCloud<pcl::PointXYZ>()),
  previous_cloud_(new pcl::PointCloud<pcl::PointXYZ>()),
  accumulated_transform_(Eigen::Matrix4f::Identity()),
  is_first_cloud_(true)
{
    // Initialize parameters
    initializeParameters();

    // Initialize TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Initialize registration algorithm
    initializeRegistration();

    // Initialize voxel filter
    voxel_filter_.setLeafSize(voxel_size_, voxel_size_, voxel_size_);

    // Create subscriber
    pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, 
        rclcpp::SensorDataQoS(),
        std::bind(&LidarOdometry::pointCloudCallback, this, std::placeholders::_1)
    );

    // Create publisher
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
        output_topic_, 
        rclcpp::QoS(10)
    );

    RCLCPP_INFO(this->get_logger(), "LIDAR Odometry node initialized");
    RCLCPP_INFO(this->get_logger(), "Input topic: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Output topic: %s", output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Base frame: %s", base_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "Odom frame: %s", odom_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "Voxel size: %.3f", voxel_size_);
    RCLCPP_INFO(this->get_logger(), "Use CUDA: %s", use_cuda_ ? "true" : "false");
}

void LidarOdometry::initializeParameters()
{
    // Declare and get parameters
    this->declare_parameter("input_topic", "/livox/lidar");
    this->declare_parameter("output_topic", "/odometry/lidar");
    this->declare_parameter("base_frame", "base_link");
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("voxel_size", 0.5);
    this->declare_parameter("use_cuda", false);
    this->declare_parameter("max_iterations", 64);
    this->declare_parameter("transformation_epsilon", 1e-6);
    this->declare_parameter("euclidean_fitness_epsilon", 1e-6);
    this->declare_parameter("max_correspondence_distance", 1.0);

    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    voxel_size_ = this->get_parameter("voxel_size").as_double();
    use_cuda_ = this->get_parameter("use_cuda").as_bool();
    max_iterations_ = this->get_parameter("max_iterations").as_int();
    transformation_epsilon_ = this->get_parameter("transformation_epsilon").as_double();
    euclidean_fitness_epsilon_ = this->get_parameter("euclidean_fitness_epsilon").as_double();
    max_correspondence_distance_ = this->get_parameter("max_correspondence_distance").as_double();
}

void LidarOdometry::initializeRegistration()
{
    // Initialize FastGICP
    registration_ = std::make_shared<fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ>>();
    
    // Set registration parameters
    registration_->setMaximumIterations(max_iterations_);
    registration_->setTransformationEpsilon(transformation_epsilon_);
    registration_->setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);
    registration_->setMaxCorrespondenceDistance(max_correspondence_distance_);

    // Try to use CUDA if available and requested
    if (use_cuda_) {
        try {
            // Note: FastVGICP can use GPU acceleration
            auto fast_vgicp = std::make_shared<fast_gicp::FastVGICP<pcl::PointXYZ, pcl::PointXYZ>>();
            fast_vgicp->setMaximumIterations(max_iterations_);
            fast_vgicp->setTransformationEpsilon(transformation_epsilon_);
            fast_vgicp->setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);
            fast_vgicp->setMaxCorrespondenceDistance(max_correspondence_distance_);
            
            registration_ = fast_vgicp;
            RCLCPP_INFO(this->get_logger(), "Using FastVGICP (with potential GPU acceleration)");
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to initialize CUDA acceleration: %s", e.what());
            RCLCPP_INFO(this->get_logger(), "Falling back to CPU-based FastGICP");
        }
    } else {
        RCLCPP_INFO(this->get_logger(), "Using CPU-based FastGICP");
    }
}

void LidarOdometry::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // Convert ROS point cloud to PCL
    pcl::fromROSMsg(*msg, *current_cloud_);

    if (current_cloud_->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
        return;
    }

    // Preprocess point cloud
    auto preprocessed_cloud = preprocessPointCloud(current_cloud_);

    if (is_first_cloud_) {
        // Initialize with first cloud
        previous_cloud_ = preprocessed_cloud;
        last_timestamp_ = msg->header.stamp;
        is_first_cloud_ = false;
        
        // Publish identity transform for first frame
        publishOdometry(Eigen::Matrix4f::Identity(), msg->header);
        publishTransform(Eigen::Matrix4f::Identity(), msg->header);
        
        RCLCPP_INFO(this->get_logger(), "First cloud received and processed");
        return;
    }

    // Set input clouds for registration
    registration_->setInputTarget(previous_cloud_);
    registration_->setInputSource(preprocessed_cloud);

    // Perform registration
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    auto start_time = this->get_clock()->now();
    
    registration_->align(*aligned_cloud);
    
    auto end_time = this->get_clock()->now();
    auto duration = (end_time - start_time).seconds();

    if (registration_->hasConverged()) {
        // Get transformation matrix
        Eigen::Matrix4f transformation = registration_->getFinalTransformation();
        
        // Update accumulated transform
        accumulated_transform_ = accumulated_transform_ * transformation;

        // Publish odometry and transform
        publishOdometry(accumulated_transform_, msg->header);
        publishTransform(accumulated_transform_, msg->header);

        // Update for next iteration
        previous_cloud_ = preprocessed_cloud;
        last_timestamp_ = msg->header.stamp;

        RCLCPP_DEBUG(this->get_logger(), 
            "Registration converged in %.3f seconds, fitness score: %.6f", 
            duration, registration_->getFitnessScore());
    } else {
        RCLCPP_WARN(this->get_logger(), "Registration did not converge");
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LidarOdometry::preprocessPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    
    // Apply voxel grid filter
    voxel_filter_.setInputCloud(cloud);
    voxel_filter_.filter(*filtered_cloud);

    return filtered_cloud;
}

void LidarOdometry::publishOdometry(const Eigen::Matrix4f& transform, const std_msgs::msg::Header& header)
{
    auto odometry_msg = eigenToOdometry(transform, header);
    odometry_publisher_->publish(odometry_msg);
}

void LidarOdometry::publishTransform(const Eigen::Matrix4f& transform, const std_msgs::msg::Header& header)
{
    auto transform_msg = eigenToTransformStamped(transform, header);
    tf_broadcaster_->sendTransform(transform_msg);
}

geometry_msgs::msg::TransformStamped LidarOdometry::eigenToTransformStamped(
    const Eigen::Matrix4f& transform, const std_msgs::msg::Header& header)
{
    geometry_msgs::msg::TransformStamped transform_stamped;
    
    transform_stamped.header.stamp = header.stamp;
    transform_stamped.header.frame_id = odom_frame_;
    transform_stamped.child_frame_id = base_frame_;

    // Extract translation
    transform_stamped.transform.translation.x = transform(0, 3);
    transform_stamped.transform.translation.y = transform(1, 3);
    transform_stamped.transform.translation.z = transform(2, 3);

    // Extract rotation as quaternion
    Eigen::Matrix3f rotation_matrix = transform.block<3, 3>(0, 0);
    Eigen::Quaternionf quaternion(rotation_matrix);
    quaternion.normalize();

    transform_stamped.transform.rotation.x = quaternion.x();
    transform_stamped.transform.rotation.y = quaternion.y();
    transform_stamped.transform.rotation.z = quaternion.z();
    transform_stamped.transform.rotation.w = quaternion.w();

    return transform_stamped;
}

nav_msgs::msg::Odometry LidarOdometry::eigenToOdometry(
    const Eigen::Matrix4f& transform, const std_msgs::msg::Header& header)
{
    nav_msgs::msg::Odometry odometry_msg;
    
    odometry_msg.header.stamp = header.stamp;
    odometry_msg.header.frame_id = odom_frame_;
    odometry_msg.child_frame_id = base_frame_;

    // Set position
    odometry_msg.pose.pose.position.x = transform(0, 3);
    odometry_msg.pose.pose.position.y = transform(1, 3);
    odometry_msg.pose.pose.position.z = transform(2, 3);

    // Set orientation
    Eigen::Matrix3f rotation_matrix = transform.block<3, 3>(0, 0);
    Eigen::Quaternionf quaternion(rotation_matrix);
    quaternion.normalize();

    odometry_msg.pose.pose.orientation.x = quaternion.x();
    odometry_msg.pose.pose.orientation.y = quaternion.y();
    odometry_msg.pose.pose.orientation.z = quaternion.z();
    odometry_msg.pose.pose.orientation.w = quaternion.w();

    // Set covariance (identity for now - could be improved with actual uncertainty estimation)
    for (int i = 0; i < 36; ++i) {
        odometry_msg.pose.covariance[i] = 0.0;
    }
    odometry_msg.pose.covariance[0] = 0.1;   // x
    odometry_msg.pose.covariance[7] = 0.1;   // y
    odometry_msg.pose.covariance[14] = 0.1;  // z
    odometry_msg.pose.covariance[21] = 0.1;  // roll
    odometry_msg.pose.covariance[28] = 0.1;  // pitch
    odometry_msg.pose.covariance[35] = 0.1;  // yaw

    // Set twist (velocity) - for now set to zero
    // Could be computed from pose differences and timestamps
    for (int i = 0; i < 36; ++i) {
        odometry_msg.twist.covariance[i] = 0.0;
    }

    return odometry_msg;
}

} // namespace lidar_odometry_fastgicp
