#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"  // 提供消息接口
#include "tf2/LinearMath/Quaternion.h"              // 提供tf2::Quaternion类
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  // 提供消息类型转换函数
#include "tf2_ros/static_transform_broadcaster.h"   // 提供静态变换广播器

class StaticTFBroadcaster : public rclcpp::Node
{
public:
  StaticTFBroadcaster() : Node("static_tf_broadcaster")
  {
    // 创建静态变换广播器
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    publish_static_tf();
  }

  void publish_static_tf()
  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    // 设置变换消息的属性
    transform_stamped.header.stamp = this->get_clock()->now();
    transform_stamped.header.frame_id = "map";
    transform_stamped.child_frame_id = "target_point";

    // 设置变换消息的坐标系
    transform_stamped.transform.translation.x = 5.0;
    transform_stamped.transform.translation.y = 3.0;
    transform_stamped.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 60 * M_PI / 180.0);
    transform_stamped.transform.rotation = tf2::toMsg(q);
    this->static_tf_broadcaster_->sendTransform(transform_stamped);
  }

  private:
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr publisher_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticTFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}