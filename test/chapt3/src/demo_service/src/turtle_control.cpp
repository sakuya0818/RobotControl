#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <service_interfaces/srv/patrol.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <chrono>

class TurtleControlNode : public rclcpp::Node
{
public:
    explicit TurtleControlNode(const std::string& node_name)
    : Node(node_name)
    {
        this->declare_parameter("k", 1.0);
        this->declare_parameter("max_speed", 1.0);
        this->get_parameter("k", k_);
        this->get_parameter("max_speed", max_speed_);
        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& parameters)->rcl_interfaces::msg::SetParametersResult {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                result.reason = "success";
                for (const auto& parameter : parameters)
                {
                    RCLCPP_INFO(this->get_logger(), "设置参数 %s: %f", parameter.get_name().c_str(), parameter.as_double());
                    if (parameter.get_name() == "k")
                    {
                        k_ = parameter.as_double();
                    }
                    if (parameter.get_name() == "max_speed")
                    {
                        max_speed_ = parameter.as_double();
                    }
                }
                return result;
            });
        patrol_service_ = this->create_service<service_interfaces::srv::Patrol>(
            "patrol", [this](const std::shared_ptr<service_interfaces::srv::Patrol::Request> request,
                             std::shared_ptr<service_interfaces::srv::Patrol::Response> response) {
                RCLCPP_INFO(get_logger(), "收到patrol服务请求");
                if (0 < request->target_x && request->target_x < 12.0 &&
                    0 < request->target_y && request->target_y < 12.0)
                {
                    target_x = request->target_x;
                    target_y = request->target_y;
                    response->result = service_interfaces::srv::Patrol_Response::SUCCESS;
                    RCLCPP_INFO(get_logger(), "设置目标点为 x: %f, y: %f", request->target_x, request->target_y);
                } else {
                    response->result = service_interfaces::srv::Patrol_Response::FAILURE;
                    RCLCPP_ERROR(get_logger(), "目标点超出范围");
                }
            });
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
    // 服务回调函数
    rclcpp::Service<service_interfaces::srv::Patrol>::SharedPtr patrol_service_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    // 事件参数的回调函数
    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
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