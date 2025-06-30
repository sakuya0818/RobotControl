#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"  // 提供消息接口
#include "tf2/LinearMath/Quaternion.h"              // 提供tf2::Quaternion类
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  // 提供消息类型转换函数
#include "tf2_ros/transform_broadcaster.h"          // 提供变换广播器
#include <chrono>

class TFBroadcaster : public rclcpp::Node
{
public:
  TFBroadcaster() : Node("tf_broadcaster")
  {
    // 创建动态变换广播器
    static_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    timer_ = create_wall_timer(std::chrono::milliseconds(1000), std::bind(&TFBroadcaster::publish_tf, this));
  }

  void publish_tf()
  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    // 设置变换消息的属性
    transform_stamped.header.stamp = this->get_clock()->now();
    transform_stamped.header.frame_id = "map";
    transform_stamped.child_frame_id = "base_link";

    // 设置变换消息的坐标系
    transform_stamped.transform.translation.x = 2.0;
    transform_stamped.transform.translation.y = 3.0;
    transform_stamped.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 30 * M_PI / 180.0);
    transform_stamped.transform.rotation = tf2::toMsg(q);
    this->static_tf_broadcaster_->sendTransform(transform_stamped);
  }

  private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> static_tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}