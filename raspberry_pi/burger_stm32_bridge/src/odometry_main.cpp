#include <rclcpp/rclcpp.hpp>
#include "turtlebot3_node/odometry.hpp"
int main(int argc,char **argv) {
 rclcpp::init(argc,argv);
 auto node=std::make_shared<rclcpp::Node>("diff_drive_controller");
 double radius=node->declare_parameter("wheels.radius",0.033);
 double separation=node->declare_parameter("wheels.separation",0.160);
 robotis::turtlebot3::Odometry odometry(node,separation,radius);
 rclcpp::spin(node);rclcpp::shutdown();
}
