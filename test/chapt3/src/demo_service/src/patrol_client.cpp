#include <rclcpp/rclcpp.hpp>
#include <service_interfaces/srv/patrol.hpp>
#include <chrono>
#include <ctime>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>

class PatrolClient : public rclcpp::Node
{
public:
    explicit PatrolClient(const std::string& node_name)
    : Node(node_name)
    {
        srand(time(NULL));  // 初始化随机数种子
        patrol_client_ = create_client<service_interfaces::srv::Patrol>("patrol");
        timer_ = create_wall_timer(std::chrono::milliseconds(10000), [this]() {
            if (!patrol_client_->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_ERROR(get_logger(), "Service not available");
                return;
            }
            auto request = std::make_shared<service_interfaces::srv::Patrol::Request>();
            request->target_x = static_cast<float>(rand() % 12 + 1);  // 随机生成1到12之间的x坐标
            request->target_y = static_cast<float>(rand() % 12 + 1);  // 随机生成1到12之间的y坐标
            RCLCPP_INFO(get_logger(), "Patrol request sent successfully,Target: (%f, %f)", request->target_x, request->target_y);
            patrol_client_->async_send_request(request, [this](rclcpp::Client<service_interfaces::srv::Patrol>::SharedFuture result_future)-> void {
                auto response = result_future.get();
                if (response->result == service_interfaces::srv::Patrol::Response::SUCCESS)
                {
                    RCLCPP_INFO(get_logger(), "Patrol request sent successfully");
                }
                else
                {
                    RCLCPP_ERROR(get_logger(), "Failed to send patrol request");
                }
                RCLCPP_INFO(get_logger(), "Sent patrol request");
            });
        });
    }

    // 创建客户端，发送请求
    rcl_interfaces::srv::SetParameters::Response::SharedPtr call_set_parameters(
        rcl_interfaces::msg::Parameter& param)
    {
        auto param_client_ = create_client<rcl_interfaces::srv::SetParameters>("/turtle_control_node/set_parameters");
        // 检测服务端是否上线
        while (!param_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return nullptr;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
    }

    // 更新参数K
    void update_server_param_k(double k)
    { 
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<service_interfaces::srv::Patrol>::SharedPtr patrol_client_;
};

int main(int argc, char* argv[] )
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatrolClient>("patrol_client");
    node->update_server_param_k(4.0);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}