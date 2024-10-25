#include <google/protobuf/util/json_util.h>

#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"

#include <goby/middleware/marshalling/protobuf.h>

#include "goby_ros_gateway/goby_ros_gateway.h"
#include "goby_ros_examples/nav.pb.h"
#include "goby_ros_examples/msg/nav.hpp"
#include "goby_ros_examples/msg/protobuf_encoded.hpp"
#include "goby_ros_examples/msg/protobuf_json.hpp"

namespace groups
{
constexpr goby::middleware::Group protobuf_to_ros{"protobuf_to_ros"};
constexpr goby::middleware::Group protobuf_from_ros{"protobuf_from_ros"};

} // namespace groups


class GobyToROSExample : public goby::ros::TranslatorToROS
{
  public:
    GobyToROSExample(const goby::apps::ros::protobuf::GatewayConfig& cfg)
        : goby::ros::TranslatorToROS(cfg, "goby_to_ros_test")
    {
        protobuf_encoded_publisher_ = this->create_publisher<goby_ros_examples::msg::ProtobufEncoded>("protobuf_encoded_from_goby", 10);

        interprocess().subscribe_type_regex<groups::protobuf_to_ros, google::protobuf::Message>(
            [this](std::shared_ptr<const google::protobuf::Message> protobuf, const std::string& type)
            {
                {
                    auto ros_msg = goby_ros_examples::msg::ProtobufEncoded();
                    ros_msg.encoded.resize(protobuf->ByteSizeLong());
                    protobuf->SerializeToArray(&ros_msg.encoded[0], ros_msg.encoded.size());
                    ros_msg.pb_type = type;
                    RCLCPP_INFO(this->get_logger(), "Generic Encoded Protobuf: Tx (to ROS): '%s'", goby::util::hex_encode(std::string(ros_msg.encoded.begin(), ros_msg.encoded.end())).c_str());
                    protobuf_encoded_publisher_->publish(ros_msg);
                }
            });
    }

  private:
    rclcpp::Publisher<goby_ros_examples::msg::ProtobufEncoded>::SharedPtr protobuf_encoded_publisher_;
};

class ROSToGobyExample : public goby::ros::TranslatorToGoby
{
  public:
    ROSToGobyExample(const goby::apps::ros::protobuf::GatewayConfig& cfg)
        : goby::ros::TranslatorToGoby(cfg, "ros_to_goby_test")
    {
        protobuf_encoded_subscriber_ = this->create_subscription<goby_ros_examples::msg::ProtobufEncoded>(
            "protobuf_encoded_to_goby", 1,
            [this](const goby_ros_examples::msg::ProtobufEncoded::ConstSharedPtr ros_msg)
            {
                if(ros_msg->pb_type == goby_ros_examples::protobuf::Nav::descriptor()->full_name())
                {
                    RCLCPP_INFO(this->get_logger(), "Rx Encoded Nav (from ROS)");
                    goby_ros_examples::protobuf::Nav pb_msg;
                    pb_msg.ParseFromArray(&ros_msg->encoded[0], ros_msg->encoded.size());
                    RCLCPP_INFO(this->get_logger(), "Tx (to Goby): '%s'", pb_msg.ShortDebugString().c_str());
                    interprocess().publish<groups::protobuf_from_ros>(pb_msg);
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Rx UNKNOWN TYPE %s (to Goby)", ros_msg->pb_type.c_str());
                }
            });
    }

  private:
    rclcpp::Subscription<goby_ros_examples::msg::ProtobufEncoded>::SharedPtr protobuf_encoded_subscriber_;

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
