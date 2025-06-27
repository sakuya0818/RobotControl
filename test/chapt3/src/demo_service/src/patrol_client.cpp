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
        const rcl_interfaces::msg::Parameter& param)
    {
        auto param_client = create_client<rcl_interfaces::srv::SetParameters>("/turtle_control_node/set_parameters");
        // 检测服务端是否上线
        while (!param_client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return nullptr;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
        // 构造请求的对象
        auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
        request->parameters.push_back(param);
        RCLCPP_INFO(this->get_logger(), "SetParameters request name: %s", param.name.c_str());
        auto future = param_client->async_send_request(request);
        rclcpp::spin_until_future_complete(this->shared_from_this(), future);
        auto response = future.get();

        return response;
    }

    // 更新参数K
    void update_server_param_k(double k)
    { 
        // 创建参数对象
        auto param = rcl_interfaces::msg::Parameter();
        param.name = "k";

        // 创建参数值
        auto param_value = rcl_interfaces::msg::ParameterValue();
        param_value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        param_value.double_value = k;
        param.value = param_value;

        // 请求更新参数并处理
        auto reponse = call_set_parameters(param);
        if (reponse == nullptr) {
            RCLCPP_ERROR(get_logger(), "Failed to update parameter k");
        } else if (reponse->results.empty() || !reponse->results[0].successful) {
            RCLCPP_ERROR(get_logger(), "Failed to update parameter k");
        } else {
            RCLCPP_INFO(get_logger(), "Parameter k updated successfully to %f", k);
        }
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