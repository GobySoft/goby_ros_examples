#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "goby_ros_examples/msg/protobuf_encoded.hpp"
#include "goby_ros_examples/nav.pb.h"

using namespace std::chrono_literals;

class ProtobufPublisher : public rclcpp::Node
{
  public:
    ProtobufPublisher()
    : Node("protobuf_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<goby_ros_examples::msg::ProtobufEncoded>("protobuf_encoded_to_goby", 10);
      timer_ = this->create_wall_timer(1000ms, std::bind(&ProtobufPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        // Define Protobuf message
        goby_ros_examples::protobuf::Nav pb_nav;
        pb_nav.set_lat(100.3);
        pb_nav.set_lon(34.2);

        // Serialize to ROS Msg
        auto ros_msg = goby_ros_examples::msg::ProtobufEncoded();
        ros_msg.pb_type = pb_nav.GetDescriptor()->full_name();
        ros_msg.encoded.resize(pb_nav.ByteSizeLong());
        pb_nav.SerializeToArray(&ros_msg.encoded[0], ros_msg.encoded.size());

        // Publish it to goby_ros_gateway
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s': '%s'", ros_msg.pb_type.c_str(), pb_nav.ShortDebugString().c_str());
        publisher_->publish(ros_msg);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<goby_ros_examples::msg::ProtobufEncoded>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ProtobufPublisher>());
  rclcpp::shutdown();
  return 0;
}
