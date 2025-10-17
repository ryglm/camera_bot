#include <chrono>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"


using namespace std::chrono_literals;


class Publisher: public rclcpp::Node {
public:
 Publisher() : rclcpp::Node("publisher"),
  gen_(std::random_device{}()),
   dist_(0.0f, 1.0f) {
    topic_ = this->declare_parameter<std::string>("topic", "readings");
    double rate_hz = this->declare_parameter<double>("publish_rate_hz", 10.0);
    if (rate_hz < 0.0) {
     RCLCPP_WARN(this->get_logger(), "frequency < 0 Hz; using 10.0 Hz");
     rate_hz = 10.0;
    }
    pub_ = this->create_publisher<std_msgs::msg::Float32>(topic_,10);

    const auto period = std::chrono::duration<double>(1.0/rate_hz);
    timer_ = this->create_wall_timer(
     std::chrono::duration_cast<std::chrono::milliseconds>(period),
     std::bind(&Publisher::on_timer, this)
    );
    RCLCPP_INFO(this->get_logger(), "Publishing on '%s' at %.2f Hz", topic_.c_str(), rate_hz);
	}
private:
 void on_timer() {
  std_msgs::msg::Float32 msg;
  msg.data = dist_(gen_);
  pub_->publish(msg);
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,"Latest value: %.3f", msg.data);
 }
 std::string topic_;
 rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
 rclcpp::TimerBase::SharedPtr timer_;
 std::mt19937 gen_;
 std::uniform_real_distribution<float> dist_;
 };

int main(int argc, char ** argv) {
 rclcpp::init(argc,argv);
 rclcpp::spin(std::make_shared<Publisher>());
 rclcpp::shutdown();
 return 0;
}
