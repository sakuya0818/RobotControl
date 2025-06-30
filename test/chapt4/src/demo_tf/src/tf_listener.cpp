#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"  // 提供消息接口
#include "tf2/LinearMath/Quaternion.h"              // 提供tf2::Quaternion类
#include "tf2/utils.h"                              // 提供四元数转欧拉角的函数
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  // 提供消息类型转换函数
#include "tf2_ros/transform_listener.h"             // 提供坐标监听
#include "tf2_ros/buffer.h"                         // 提供tf2::Buffer类
#include <chrono>

class TFListener : public rclcpp::Node
{
public:
    TFListener() : Node("tf_listener")
    {
        // 创建坐标监听器
        buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, this);
        timer_ = create_wall_timer(std::chrono::milliseconds(1000), std::bind(&TFListener::getTransform, this));
    }

    void getTransform()
    {
        // 从buffer_中获取变换
        try
        {
            // 查询坐标关系
            const auto transform = buffer_->lookupTransform("base_link", "target_point", this->get_clock()->now(),
             rclcpp::Duration::from_seconds(1.0));
            // 获取查询结果
            auto translation = transform.transform.translation;
            auto rotation = transform.transform.rotation;
            double roll, pitch, yaw;
            tf2::getEulerYPR(rotation, roll, pitch, yaw);
            RCLCPP_INFO(this->get_logger(), "平移:%f, %f, %f", translation.x, translation.y, translation.z);
            RCLCPP_INFO(this->get_logger(), "旋转(欧拉角): roll=%f, pitch=%f, yaw=%f", roll, pitch, yaw);
        }
        catch (const tf2::TransformException & ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        }
    }

//   void publish_static_tf()
//   {
//     geometry_msgs::msg::TransformStamped transform_stamped;
//     // 设置变换消息的属性
//     transform_stamped.header.stamp = this->get_clock()->now();
//     transform_stamped.header.frame_id = "map";
//     transform_stamped.child_frame_id = "base_link";

//     // 设置变换消息的坐标系
//     transform_stamped.transform.translation.x = 2.0;
//     transform_stamped.transform.translation.y = 3.0;
//     transform_stamped.transform.translation.z = 0.0;

//     tf2::Quaternion q;
//     q.setRPY(0.0, 0.0, 30 * M_PI / 180.0);
//     transform_stamped.transform.rotation = tf2::toMsg(q);
//     this->static_tf_broadcaster_->sendTransform(transform_stamped);
//   }

private:
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    std::shared_ptr<tf2_ros::Buffer> buffer_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFListener>());
  rclcpp::shutdown();
  return 0;
}