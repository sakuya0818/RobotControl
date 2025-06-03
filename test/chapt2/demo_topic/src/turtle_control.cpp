#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <chrono>

class TurtleControlNode : public rclcpp::Node
{
public:
    explicit TurtleControlNode(const std::string& node_name)
    : Node(node_name)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10, [this](const turtlesim::msg::Pose::SharedPtr pose) {
                // 获取当前位置
                auto current_x = pose->x;
                auto current_y = pose->y;
                RCLCPP_INFO(get_logger(), "当前，x: %f, y: %f", current_x, current_y);

                // 计算目标点与当前位置之间的距离差和角度差
                auto destance = std::sqrt(std::pow(target_x - current_x, 2) + std::pow(target_y - current_y, 2));
                auto angle = std::atan2(target_y - current_y, target_x - current_x) - pose->theta;

                // 控制策略
                if (destance > 0.01)
                {
                    auto cmd_vel = geometry_msgs::msg::Twist();
                    cmd_vel.linear.x = std::min(k_ * destance, max_speed_);
                    cmd_vel.angular.z = std::fabs(angle);
                    publisher_->publish(cmd_vel);
                } else {
                    RCLCPP_INFO(get_logger(), "到达目标点，停止移动");
                    geometry_msgs::msg::Twist stop_cmd;
                    stop_cmd.linear.x = 0.0;
                    stop_cmd.angular.z = 0.0;
                    publisher_->publish(stop_cmd);
                }
            });
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    double target_x{1.0};
    double target_y{1.0};
    double k_{1.0};         // 比例系数
    double max_speed_{3.0}; // 最大速度
};

int main(int argc, char* argv[] )
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControlNode>("turtle_control_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}