#include "std_msgs/msg/string.hpp"

#include "rclcpp/rclcpp.hpp"

#include <goby/middleware/marshalling/json.h>

#include "goby_ros_gateway/goby_ros_gateway.h"

namespace groups
{
constexpr goby::middleware::Group json_to_ros{"json_to_ros"};
constexpr goby::middleware::Group json_from_ros{"json_from_ros"};
} // namespace groups


class GobyToROSExample : public goby::ros::TranslatorToROS
{
  public:
    GobyToROSExample(const goby::apps::ros::protobuf::GatewayConfig& cfg)
        : goby::ros::TranslatorToROS(cfg, "goby_to_ros_test")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("json_from_goby", 10);

        interprocess().subscribe_type_regex<groups::json_to_ros, nlohmann::json>(
            [this](std::shared_ptr<const nlohmann::json> json, const std::string& /*type*/)
            {
                RCLCPP_INFO(this->get_logger(), "Rx (from Goby): '%s'", json->dump().c_str());
                auto message = std_msgs::msg::String();
                message.data = json->dump();
                RCLCPP_INFO(this->get_logger(), "Tx (to ROS): '%s'", message.data.c_str());
                publisher_->publish(message);
            });
    }

  private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

class ROSToGobyExample : public goby::ros::TranslatorToGoby
{
  public:
    ROSToGobyExample(const goby::apps::ros::protobuf::GatewayConfig& cfg)
        : goby::ros::TranslatorToGoby(cfg, "ros_to_goby_test")
    {
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "json_to_goby", 1,
            [this](const std_msgs::msg::String::ConstSharedPtr msg)
            {
                RCLCPP_INFO(this->get_logger(), "Rx (from ROS): '%s'", msg->data.c_str());
                nlohmann::json j = nlohmann::json::parse(msg->data);
                RCLCPP_INFO(this->get_logger(), "Tx (to Goby): '%s'", j.dump().c_str());
                interprocess().publish<groups::json_from_ros>(j);
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
        handler->launch_thread<GobyToROSExample>();
        handler->launch_thread<ROSToGobyExample>();
    }

    void goby_ros_gateway_unload(
        goby::zeromq::MultiThreadApplication<goby::apps::ros::protobuf::GatewayConfig>*
            handler)
    {
        handler->join_thread<GobyToROSExample>();
        handler->join_thread<ROSToGobyExample>();
    }
}
