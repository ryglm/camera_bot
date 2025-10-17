/*
===================================================================================

TELEOP SCRIPT FOR UGV

===================================================================================

*/

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <atomic>
#include <thread>
#include <chrono>
#include <algorithm>  // clamp
#include <cmath>      // abs
#include <termios.h>
#include <unistd.h>

using namespace std::chrono_literals;

class RawTerminal {
public:
  RawTerminal() : active_(false) {
    if (tcgetattr(STDIN_FILENO, &orig_) == 0) {
      raw_ = orig_;
      raw_.c_lflag &= ~(ICANON | ECHO);
      raw_.c_cc[VMIN] = 0;
      raw_.c_cc[VTIME] = 1; // 100ms read timeout
      tcsetattr(STDIN_FILENO, TCSANOW, &raw_);
      active_ = true;
    }
  }
  ~RawTerminal() {
    if (active_) tcsetattr(STDIN_FILENO, TCSANOW, &orig_);
  }
  RawTerminal(const RawTerminal&) = delete;
  RawTerminal& operator=(const RawTerminal&) = delete;

private:
  termios orig_{};
  termios raw_{};
  bool active_;
};

// C++17-friendly atomic helpers (CAS loops)
inline void atomic_add(std::atomic<double>& x, double delta) {
  double old = x.load(std::memory_order_relaxed), desired;
  do { desired = old + delta; }
  while (!x.compare_exchange_weak(old, desired,
                                  std::memory_order_relaxed,
                                  std::memory_order_relaxed));
}
inline void atomic_set(std::atomic<double>& x, double value) {
  x.store(value, std::memory_order_relaxed);
}

class TeleopUGV : public rclcpp::Node {
public:
  TeleopUGV()
  : Node("teleop_ugv"),
    steer_cmd_(0.0), speed_cmd_(0.0),
    running_(true)
  {
    // Parameters
    steering_topic_ = this->declare_parameter<std::string>(
        "steering_topic", "/ugv/steering_controller/commands");
    rear_topic_ = this->declare_parameter<std::string>(
        "rear_topic", "/ugv/rear_wheels_controller/commands");
    max_steer_rad_ = this->declare_parameter<double>("max_steer_rad", 0.6);
    max_speed_radps_ = this->declare_parameter<double>("max_speed_radps", 12.0);
    step_steer_ = this->declare_parameter<double>("step_steer", 0.02);
    step_speed_ = this->declare_parameter<double>("step_speed", 0.5);
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 30.0);
    auto_center_ = this->declare_parameter<bool>("auto_center", true);
    center_decay_per_sec_ = this->declare_parameter<double>("center_decay_per_sec", 1.5); // rad/s toward zero
    deadman_timeout_sec_ = this->declare_parameter<double>("deadman_timeout_sec", 0.7);

    steer_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(steering_topic_, 10);
    rear_pub_  = this->create_publisher<std_msgs::msg::Float64MultiArray>(rear_topic_, 10);

    last_input_time_ = this->now();

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1e-3, publish_rate_hz_)),
      std::bind(&TeleopUGV::onTimer, this));

    input_thread_ = std::thread(&TeleopUGV::inputLoop, this);

    RCLCPP_INFO(get_logger(),
      "Keys: w/s speed ±, a/d steer ±, space stop, x brake, c center steer, r reset, q quit");
  }

  ~TeleopUGV() override {
    running_.store(false, std::memory_order_relaxed);
    if (input_thread_.joinable()) input_thread_.join();
  }

private:
  void onTimer() {
    const double dt = 1.0 / std::max(1e-3, publish_rate_hz_);
    if ((this->now() - last_input_time_).seconds() > deadman_timeout_sec_) {
      atomic_set(speed_cmd_, 0.0);
      if (auto_center_) {
        double s = steer_cmd_.load(std::memory_order_relaxed);
        const double decay = center_decay_per_sec_ * dt;
        if (std::abs(s) <= decay) s = 0.0;
        else s += (s > 0.0 ? -decay : decay);
        atomic_set(steer_cmd_, s);
      }
    }

    double s = std::clamp(steer_cmd_.load(std::memory_order_relaxed), -max_steer_rad_, max_steer_rad_);
    double v = std::clamp(speed_cmd_.load(std::memory_order_relaxed), -max_speed_radps_, max_speed_radps_);

    // Build and publish messages: steering [fl, fr]; rear wheels [rl, rr]
    std_msgs::msg::Float64MultiArray steer_msg;
    steer_msg.data = {s, s};
    std_msgs::msg::Float64MultiArray rear_msg;
    rear_msg.data = {v, v};

    steer_pub_->publish(steer_msg);
    rear_pub_->publish(rear_msg);
  }

  void inputLoop() {
    RawTerminal rt;
    while (running_.load(std::memory_order_relaxed)) {
      char c = 0;
      const ssize_t n = read(STDIN_FILENO, &c, 1);
      if (n == 1) {
        last_input_time_ = this->now();
        handleKey(c);
      } else {
        std::this_thread::sleep_for(10ms);
      }
    }
  }

    void handleKey(char c) {
    switch (c) {
        case 'w': atomic_add(speed_cmd_, -step_speed_);  break;  // decelerate / reverse
        case 's': atomic_add(speed_cmd_,  step_speed_);  break;  // accelerate / forward
        case 'a': atomic_add(steer_cmd_,  step_steer_);  break;  // steer right
        case 'd': atomic_add(steer_cmd_,  -step_steer_);  break;  // steer left
        case 'x': atomic_set(speed_cmd_, 0.0);           break;
        case 'c': atomic_set(steer_cmd_, 0.0);           break;
        case 'r': atomic_set(steer_cmd_, 0.0);
                atomic_set(speed_cmd_, 0.0);           break;
        case ' ': atomic_set(steer_cmd_, 0.0);
                atomic_set(speed_cmd_, 0.0);           break;
        case 'q': rclcpp::shutdown();
                running_.store(false, std::memory_order_relaxed); break;
        default: break;
    }
    }

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steer_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rear_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters / state
  std::string steering_topic_, rear_topic_;
  double max_steer_rad_, max_speed_radps_;
  double step_steer_, step_speed_;
  double publish_rate_hz_;
  bool auto_center_;
  double center_decay_per_sec_;
  double deadman_timeout_sec_;

  std::atomic<double> steer_cmd_;
  std::atomic<double> speed_cmd_;
  std::atomic<bool> running_;
  rclcpp::Time last_input_time_;
  std::thread input_thread_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopUGV>());
  rclcpp::shutdown();
  return 0;
}
