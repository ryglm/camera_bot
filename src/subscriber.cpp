#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class Subscriber: public rclcpp::Node {
public:
 Subscriber() : rclcpp::Node("subscriber") {
  topic_ = this->declare_parameter<std::string>("topic", "readings");
  threshold_ = this->declare_parameter<double>("threshold", 0.5);

  sub_ = this->create_subscription<std_msgs::msg::Float32>(topic_, 10,
		  std::bind(&Subscriber::callback, this, std::placeholders::_1)
	);
  RCLCPP_INFO(this->get_logger(), "Subscribing to '%s' with threshold %.3f", topic_.c_str(), threshold_);
 }

private:
 void callback(const std_msgs::msg::Float32::SharedPtr msg) {
  const double v = static_cast<double>(msg->data);
  if (v > threshold_){
   RCLCPP_WARN(this->get_logger(), "Alert! Your value %.3f exceeds threshold %.3f", v, threshold_);
  }
  else {
   RCLCPP_INFO(this->get_logger(), "Value %.3f is below threshold %.3f", v, threshold_);
  }
 }

 std::string topic_;
 double threshold_;
 rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;

};

int main(int argc, char** argv){
 rclcpp::init(argc, argv);
 rclcpp::spin(std::make_shared<Subscriber>());
 rclcpp::shutdown();
 return 0;
}
