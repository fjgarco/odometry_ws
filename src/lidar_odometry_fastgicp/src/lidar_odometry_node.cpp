#include "lidar_odometry_fastgicp/lidar_odometry.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<lidar_odometry_fastgicp::LidarOdometry>();
    
    RCLCPP_INFO(node->get_logger(), "Starting LIDAR Odometry FastGICP node");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
