#include "std_msgs/msg/string.hpp"

#include "rclcpp/rclcpp.hpp"

#include <goby/middleware/marshalling/json.h>

#include "goby_ros_gateway/goby_ros_gateway.h"

namespace groups
{
constexpr goby::middleware::Group nav_to_ros{"nav_to_ros"};
constexpr goby::middleware::Group nav_from_ros{"nav_from_ros"};
} // namespace groups


class GobyToROSTest : public goby::ros::TranslatorToROS
{
  public:
    GobyToROSTest(const goby::apps::ros::protobuf::GatewayConfig& cfg)
        : goby::ros::TranslatorToROS(cfg, "goby_to_ros_test")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("nav_from_goby", 10);

        interprocess().subscribe_type_regex<groups::nav_to_ros, nlohmann::json>(
            [this](std::shared_ptr<const nlohmann::json> json, const std::string& type)
            {
                RCLCPP_INFO(this->get_logger(), "Rx (type: %s): '%s'", type.c_str(), json->dump().c_str());
                auto message = std_msgs::msg::String();
                message.data = json->dump();
                RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
                publisher_->publish(message);
            });
    }

  private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

class ROSToGobyTest : public goby::ros::TranslatorToGoby
{
  public:
    ROSToGobyTest(const goby::apps::ros::protobuf::GatewayConfig& cfg)
        : goby::ros::TranslatorToGoby(cfg, "ros_to_goby_test")
    {
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "nav_to_goby", 1,
            [this](const std_msgs::msg::String::ConstSharedPtr msg)
            {
                RCLCPP_INFO(this->get_logger(), "Rx: '%s'", msg->data.c_str());
                nlohmann::json j = nlohmann::json::parse(msg->data);
                interprocess().publish<groups::nav_from_ros>(j);
            });
    }

  private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};



extern "C"
{
    void goby_ros_gateway_load(
        goby::zeromq::MultiThreadApplication<goby::apps::ros::protobuf::GatewayConfig>*
            handler)
    {
        handler->launch_thread<GobyToROSTest>();
        handler->launch_thread<ROSToGobyTest>();
    }

    void goby_ros_gateway_unload(
        goby::zeromq::MultiThreadApplication<goby::apps::ros::protobuf::GatewayConfig>*
            handler)
    {
        handler->join_thread<GobyToROSTest>();
        handler->join_thread<ROSToGobyTest>();
    }
}
