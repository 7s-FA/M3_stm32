#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <algorithm>
#include <cmath>
#include <string>

class Adapter: public rclcpp::Node {
 public:
 Adapter():Node("turtlebot3_node") {
  prefix_=declare_parameter<std::string>("frame_prefix","");
  if (!prefix_.empty() && prefix_.back()!='/') prefix_+='/';
  auto transport=declare_parameter<std::string>("transport_namespace","/stm32");
  imu_pub_=create_publisher<sensor_msgs::msg::Imu>("imu",10);
  joint_pub_=create_publisher<sensor_msgs::msg::JointState>("joint_states",10);
  cmd_pub_=create_publisher<geometry_msgs::msg::Twist>(transport+"/cmd_vel",1);
  imu_sub_=create_subscription<sensor_msgs::msg::Imu>(transport+"/imu/data",rclcpp::SensorDataQoS(),
   [this](sensor_msgs::msg::Imu::ConstSharedPtr in) {
    auto q=in->orientation;
    double n=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    if (!fresh(in->header.stamp,0.5) || in->orientation_covariance[0]<0 ||
        !std::isfinite(n) || std::abs(n-1)>0.02) return;
    auto out=*in;out.header.frame_id=prefix_+"imu_link";imu_pub_->publish(out);
   });
  joint_sub_=create_subscription<sensor_msgs::msg::JointState>(transport+"/joint_states",rclcpp::SensorDataQoS(),
   [this](sensor_msgs::msg::JointState::ConstSharedPtr in) {
    if (!fresh(in->header.stamp,0.5)) return;
    sensor_msgs::msg::JointState out;out.header=in->header;
    for (auto name:{"wheel_left_joint","wheel_right_joint"}) {
     auto it=std::find(in->name.begin(),in->name.end(),name);
     if(it==in->name.end()) return;
     auto i=static_cast<size_t>(it-in->name.begin());
     if(i>=in->position.size() || i>=in->velocity.size() ||
       !std::isfinite(in->position[i]) || !std::isfinite(in->velocity[i])) return;
     out.name.push_back(prefix_+name);out.position.push_back(in->position[i]);
     out.velocity.push_back(in->velocity[i]);
    }
    joint_pub_->publish(out);
   });
  bool stamped=declare_parameter("enable_stamped_cmd_vel",true);
  if(stamped) stamped_sub_=create_subscription<geometry_msgs::msg::TwistStamped>("cmd_vel",1,
   [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr in) {
    if(fresh(in->header.stamp,0.3)) command(in->twist);
   });
  else twist_sub_=create_subscription<geometry_msgs::msg::Twist>("cmd_vel",1,
   [this](geometry_msgs::msg::Twist::ConstSharedPtr in){command(*in);});
 }
 private:
 bool fresh(const builtin_interfaces::msg::Time &t,double max_age) {
  if(t.sec<=0) return false;
  double age=(now()-rclcpp::Time(t)).seconds();
  bool valid=age>=-0.1 && age<=max_age;
  if(!valid) {
   RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
     "Dropping timestamp outside window: Pi-minus-message=%.3f s (allowed -0.1..%.1f). Check MCU/Agent clock sync.",
     age,max_age);
  }
  return valid;
 }
 void command(const geometry_msgs::msg::Twist &in) {
  if(!std::isfinite(in.linear.x)||!std::isfinite(in.angular.z)) return;
  geometry_msgs::msg::Twist out;out.linear.x=in.linear.x;out.angular.z=in.angular.z;
  cmd_pub_->publish(out); // No replay timer: STM32's 300 ms watchdog owns stopping.
 }
 std::string prefix_;
 rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
 rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
 rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
 rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
 rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
 rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr stamped_sub_;
 rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
};
int main(int argc,char **argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<Adapter>());rclcpp::shutdown();}
