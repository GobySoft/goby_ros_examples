#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "goby_ros_examples/msg/protobuf_encoded.hpp"
#include "goby_ros_examples/nav.pb.h"

using std::placeholders::_1;

class ProtobufSubscriber : public rclcpp::Node
{
  public:
    ProtobufSubscriber()
    : Node("protobuf_subscriber")
    {
      subscription_ = this->create_subscription<goby_ros_examples::msg::ProtobufEncoded>(
      "protobuf_encoded_from_goby", 10, std::bind(&ProtobufSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const goby_ros_examples::msg::ProtobufEncoded& ros_msg) const
    {
        goby_ros_examples::protobuf::Nav pb_nav;

        if(ros_msg.pb_type == goby_ros_examples::protobuf::Nav::descriptor()->full_name())
        {
            goby_ros_examples::protobuf::Nav pb_msg;
            pb_msg.ParseFromArray(&ros_msg.encoded[0], ros_msg.encoded.size());
            RCLCPP_INFO(this->get_logger(), "Rx (%s): '%s'", ros_msg.pb_type.c_str(), pb_msg.ShortDebugString().c_str());
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Rx UNKNOWN TYPE %s", ros_msg.pb_type.c_str());
        }
    }
    rclcpp::Subscription<goby_ros_examples::msg::ProtobufEncoded>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ProtobufSubscriber>());
  rclcpp::shutdown();
  return 0;
}
