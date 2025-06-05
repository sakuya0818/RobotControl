#include <QApplication>
#include <QLabel>
#include <QString>
#include <rclcpp/rclcpp.hpp>
#include <status_interfaces/msg/system_status.hpp>

class SysStatusDisplay : public rclcpp::Node
{
public:
    SysStatusDisplay() : Node("sys_status_display")
    {
        status_label_ = new QLabel();
        status_subscriber_ = this->create_subscription<status_interfaces::msg::SystemStatus>(
            "sys_status",
            10,
            [this](const status_interfaces::msg::SystemStatus::SharedPtr msg) {
                status_label_->setText(getStatus(msg));
            }
        );
        status_label_->setText("sssssssssssssssss");
        status_label_->show();
    };

    QString getStatus(const status_interfaces::msg::SystemStatus::SharedPtr msg)
    {
        std::stringstream show_str;
        show_str << "==========新提供状态可视化显示工具==========\n" 
                 << "数 据 时 间:\t"<< msg->stamp.sec << "\ts\n"
                 << "主 机 名 字:\t" << msg->host_name << "\t\n"
                 << "CPU 使用率:\t" << msg->cpu_percentage << "\t%\n"
                 << "内存使用率:\t"<< msg->memory_percentage << "\t%\n"
                 << "内存总大小:\t"<< msg->memory_total << "\tMB\n"
                 << "剩余有效内存:\t"<< msg->memory_available <<"\tMB\n"
                 << "网络发送量:\t" << msg->net_send << "\tMB\n"
                 << "网络接受量:\t" << msg->net_recv << "\tMB\n"
                 << "=============================================";
        return QString::fromStdString(show_str.str());
    }

private:
    rclcpp::Subscription<status_interfaces::msg::SystemStatus>::SharedPtr status_subscriber_;
    QLabel *status_label_ = nullptr;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    auto node = std::make_shared<SysStatusDisplay>();
    std::thread spin_thread(
        [&]()->void
        {
            rclcpp::spin(node);
        });
    spin_thread.detach();
    
    app.exec();
    return 0;
}