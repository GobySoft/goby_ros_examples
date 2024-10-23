#include <google/protobuf/text_format.h>

#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"

#include <goby/middleware/marshalling/protobuf.h>

#include "goby_ros_gateway/goby_ros_gateway.h"
#include "goby_ros_examples/nav.pb.h"
#include "goby_ros_examples/msg/nav.hpp"


namespace groups
{
constexpr goby::middleware::Group protobuf_to_ros{"protobuf_to_ros"};
constexpr goby::middleware::Group protobuf_from_ros{"protobuf_from_ros"};
} // namespace groups


class GobyToROSTest : public goby::ros::TranslatorToROS
{
  public:
    GobyToROSTest(const goby::apps::ros::protobuf::GatewayConfig& cfg)
        : goby::ros::TranslatorToROS(cfg, "goby_to_ros_test")
    {
        text_format_publisher_ = this->create_publisher<std_msgs::msg::String>("protobuf_from_goby", 10);
        nav_encoded_publisher_ = this->create_publisher<std_msgs::msg::String>("nav_encoded_from_goby", 10);
        nav_publisher_ = this->create_publisher<goby_ros_examples::msg::Nav>("nav_from_goby", 10);

        // generic Protobuf to TextFormat string
        interprocess().subscribe_type_regex<groups::protobuf_to_ros, google::protobuf::Message>(
            [this](std::shared_ptr<const google::protobuf::Message> protobuf, const std::string& type)
            {
                RCLCPP_INFO(this->get_logger(), "Generic Protobuf: Rx (from Goby, type: %s): '%s'", type.c_str(), protobuf->ShortDebugString().c_str());
                auto message = std_msgs::msg::String();

                google::protobuf::TextFormat::Printer printer;
                printer.SetSingleLineMode(true);
                printer.PrintToString(*protobuf, &message.data);
                RCLCPP_INFO(this->get_logger(), "Generic Protobuf: Tx (to ROS): '%s'", message.data.c_str());
                text_format_publisher_->publish(message);
            });

        interprocess().subscribe<groups::protobuf_to_ros>([this](const goby_ros_examples::protobuf::Nav& pb_nav)
            {
                RCLCPP_INFO(this->get_logger(), "Nav Protobuf: Rx (from Goby): '%s'", pb_nav.ShortDebugString().c_str());
                auto message = std_msgs::msg::String();

                std::string serialized;
                pb_nav.SerializeToString(&serialized);
                message.data = boost::trim_copy(dccl::b64_encode(serialized));
                    
                RCLCPP_INFO(this->get_logger(), "Nav Encoded Protobuf: Tx (to ROS): '%s'",message.data.c_str());
                nav_encoded_publisher_->publish(message);

                goby_ros_examples::msg::Nav ros_nav;
                ros_nav.lat = pb_nav.lat();
                ros_nav.lon = pb_nav.lon();
                RCLCPP_INFO(this->get_logger(), "Nav Protobuf to ROS Msg: Tx (to ROS): 'lat=%lf, lon=%lf'", ros_nav.lat, ros_nav.lon);
                nav_publisher_->publish(ros_nav);
                
            }
            );
    }

  private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr text_format_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr nav_encoded_publisher_;
    rclcpp::Publisher<goby_ros_examples::msg::Nav>::SharedPtr nav_publisher_;
};

class ROSToGobyTest : public goby::ros::TranslatorToGoby
{
  public:
    ROSToGobyTest(const goby::apps::ros::protobuf::GatewayConfig& cfg)
        : goby::ros::TranslatorToGoby(cfg, "ros_to_goby_test")
    {
        // subscriber_ = this->create_subscription<std_msgs::msg::String>(
        //     "protobuf_to_goby", 1,
        //     [this](const std_msgs::msg::String::ConstSharedPtr msg)
        //     {
        //         RCLCPP_INFO(this->get_logger(), "Rx (from ROS): '%s'", msg->data.c_str());
        //         nlohmann::protobuf j = nlohmann::protobuf::parse(msg->data);
        //         RCLCPP_INFO(this->get_logger(), "Tx (to Goby): '%s'", j.dump().c_str());
        //         interprocess().publish<groups::protobuf_from_ros>(j);
        //     });
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
