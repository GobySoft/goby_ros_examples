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


class GobyToROSTest : public goby::ros::TranslatorToROS
{
  public:
    GobyToROSTest(const goby::apps::ros::protobuf::GatewayConfig& cfg)
        : goby::ros::TranslatorToROS(cfg, "goby_to_ros_test")
    {
        json_publisher_ = this->create_publisher<goby_ros_examples::msg::ProtobufJSON>("protobuf_json_from_goby", 10);
        protobuf_encoded_publisher_ = this->create_publisher<goby_ros_examples::msg::ProtobufEncoded>("protobuf_encoded_from_goby", 10);
        nav_publisher_ = this->create_publisher<goby_ros_examples::msg::Nav>("nav_from_goby", 10);

        // generic Protobuf to JSON string
        interprocess().subscribe_type_regex<groups::protobuf_to_ros, google::protobuf::Message>(
            [this](std::shared_ptr<const google::protobuf::Message> protobuf, const std::string& type)
            {
                RCLCPP_INFO(this->get_logger(), "Generic Protobuf: Rx (from Goby, type: %s): '%s'", type.c_str(), protobuf->ShortDebugString().c_str());
                {
                    auto ros_msg = goby_ros_examples::msg::ProtobufJSON();
                    ros_msg.pb_type = type;
                    google::protobuf::util::MessageToJsonString(*protobuf, &ros_msg.json);
                    RCLCPP_INFO(this->get_logger(), "Generic Protobuf: Tx (to ROS): '%s'", ros_msg.json.c_str());
                    json_publisher_->publish(ros_msg);
                }

                {
                    auto ros_msg = goby_ros_examples::msg::ProtobufEncoded();
                    ros_msg.encoded.resize(protobuf->ByteSizeLong());
                    protobuf->SerializeToArray(&ros_msg.encoded[0], ros_msg.encoded.size());
                    ros_msg.pb_type = type;
                    RCLCPP_INFO(this->get_logger(), "Generic Encoded Protobuf: Tx (to ROS): '%s'", goby::util::hex_encode(std::string(ros_msg.encoded.begin(), ros_msg.encoded.end())).c_str());
                    protobuf_encoded_publisher_->publish(ros_msg);
                }
            });

        interprocess().subscribe<groups::protobuf_to_ros>([this](const goby_ros_examples::protobuf::Nav& pb_nav)
            {
                goby_ros_examples::msg::Nav ros_nav;
                ros_nav.lat = pb_nav.lat();
                ros_nav.lon = pb_nav.lon();
                RCLCPP_INFO(this->get_logger(), "Nav Protobuf to ROS Msg: Tx (to ROS): 'lat=%lf, lon=%lf'", ros_nav.lat, ros_nav.lon);
                nav_publisher_->publish(ros_nav);
                
            }
            );
    }

  private:
    rclcpp::Publisher<goby_ros_examples::msg::ProtobufJSON>::SharedPtr json_publisher_;
    rclcpp::Publisher<goby_ros_examples::msg::ProtobufEncoded>::SharedPtr protobuf_encoded_publisher_;
    rclcpp::Publisher<goby_ros_examples::msg::Nav>::SharedPtr nav_publisher_;
};

class ROSToGobyTest : public goby::ros::TranslatorToGoby
{
  public:
    ROSToGobyTest(const goby::apps::ros::protobuf::GatewayConfig& cfg)
        : goby::ros::TranslatorToGoby(cfg, "ros_to_goby_test")
    {
        json_subscriber_ = this->create_subscription<goby_ros_examples::msg::ProtobufJSON>(
            "protobuf_json_to_goby", 1,
            [this](const goby_ros_examples::msg::ProtobufJSON::ConstSharedPtr ros_msg)
            {
                if(ros_msg->pb_type == goby_ros_examples::protobuf::Nav::descriptor()->full_name())
                {
                    RCLCPP_INFO(this->get_logger(), "Rx JSON Nav (from ROS): '%s'", ros_msg->json.c_str());
                    goby_ros_examples::protobuf::Nav pb_msg;
                    google::protobuf::util::JsonStringToMessage(ros_msg->json, &pb_msg);
                    RCLCPP_INFO(this->get_logger(), "Tx (to Goby): '%s'", pb_msg.ShortDebugString().c_str());
                    interprocess().publish<groups::protobuf_from_ros>(pb_msg);
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Rx UNKNOWN TYPE %s (to Goby)", ros_msg->pb_type.c_str());
                }
            });

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
        

        nav_subscriber_ = this->create_subscription<goby_ros_examples::msg::Nav>(
            "nav_to_goby", 1,
            [this](const goby_ros_examples::msg::Nav::ConstSharedPtr ros_msg)
            {
                RCLCPP_INFO(this->get_logger(), "Nav Protobuf Rx (from ROS): 'lat=%lf, lon=%lf'", ros_msg->lat, ros_msg->lon);
                goby_ros_examples::protobuf::Nav pb_nav;
                pb_nav.set_lat(ros_msg->lat);
                pb_nav.set_lon(ros_msg->lon);
                RCLCPP_INFO(this->get_logger(), "Nav Protobuf Tx (to Goby): '%s'", pb_nav.ShortDebugString().c_str());
                interprocess().publish<groups::protobuf_from_ros>(pb_nav);
            });
    }

  private:
    rclcpp::Subscription<goby_ros_examples::msg::ProtobufJSON>::SharedPtr json_subscriber_;
    rclcpp::Subscription<goby_ros_examples::msg::ProtobufEncoded>::SharedPtr protobuf_encoded_subscriber_;
    rclcpp::Subscription<goby_ros_examples::msg::Nav>::SharedPtr nav_subscriber_;

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
