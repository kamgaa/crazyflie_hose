#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <builtin_interfaces/msg/time.hpp>

using std::placeholders::_1;

class HoverSyncNode : public rclcpp::Node {
public:
  HoverSyncNode() : Node("hover_sync_node"), cf01_ready_(false), cf02_ready_(false), published_(false) {
    auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    sub_cf01_ = this->create_subscription<std_msgs::msg::Bool>(
      "/cf01/armed", 10, std::bind(&HoverSyncNode::cb_cf01, this, _1));
    sub_cf02_ = this->create_subscription<std_msgs::msg::Bool>(
      "/cf02/armed", 10, std::bind(&HoverSyncNode::cb_cf02, this, _1));

    pub_start_ = this->create_publisher<builtin_interfaces::msg::Time>("/hover_start_time", latched);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
      std::bind(&HoverSyncNode::tick, this));
  }

private:
  void cb_cf01(const std_msgs::msg::Bool::SharedPtr msg) { cf01_ready_ = msg->data; }
  void cb_cf02(const std_msgs::msg::Bool::SharedPtr msg) { cf02_ready_ = msg->data; }

  void tick() {
    if (published_) return;
    if (!(cf01_ready_ && cf02_ready_)) return;

    // 둘 다 준비됨 → 현재 시각 + 2.0초에 동시 시작 예약
    auto start = this->now() + rclcpp::Duration::from_seconds(2.0);
    int64_t ns = start.nanoseconds();

    builtin_interfaces::msg::Time t;
    t.sec = static_cast<int32_t>(ns / 1000000000LL);
    t.nanosec = static_cast<uint32_t>(ns % 1000000000LL);

    pub_start_->publish(t);
    published_ = true;

    RCLCPP_INFO(this->get_logger(), "[SYNC] hover will start at +2.0s (sec=%d, nsec=%u)", t.sec, t.nanosec);
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_cf01_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_cf02_;
  rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr pub_start_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool cf01_ready_;
  bool cf02_ready_;
  bool published_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HoverSyncNode>());
  rclcpp::shutdown();
  return 0;
}

