#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/serialized_message.hpp"

// 滑动窗口频率 + 最近一条消息的时间
class RateTracker  //频率跟踪器
{
public:
  explicit RateTracker(double window_sec) : window_sec_(window_sec) {}

  void tick(const rclcpp::Time & now)  //收到消息时调用：把时间塞进队列，更新 last_
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stamps_.push_back(now);
    last_ = now;
    has_msg_ = true;
    trim(now);
  }

  double rate(const rclcpp::Time & now)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    trim(now);
    if (stamps_.size() < 2) {
      return 0.0;
    }
    const double span = (stamps_.back() - stamps_.front()).seconds();
    return span > 0.0 ? static_cast<double>(stamps_.size() - 1) / span : 0.0;  //频率 = (窗口内条数 − 1) ÷ (最新时间 − 最早时间)
  }

  // 从未收到消息返回 -1
  double age(const rclcpp::Time & now)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return has_msg_ ? (now - last_).seconds() : -1.0;   //现在距最近一条消息过了多久；从没收到返回 -1
  }

private:
  void trim(const rclcpp::Time & now)   //滑动窗口
  {
    while (!stamps_.empty() && (now - stamps_.front()).seconds() > window_sec_) {
      stamps_.pop_front();
    }
  }

  double window_sec_;
  std::deque<rclcpp::Time> stamps_;
  rclcpp::Time last_{0, 0, RCL_ROS_TIME};
  bool has_msg_{false};
  std::mutex mutex_;
};

enum class Status { NO_DATA, OK, SLOW, STALE };

static const char * to_str(Status s)
{
  switch (s) {
    case Status::NO_DATA: return "NO_DATA";
    case Status::OK: return "OK";
    case Status::SLOW: return "SLOW";
    case Status::STALE: return "STALE";
  }
  return "?";
}

struct Channel
{
  Channel(const std::string & n, double window) : name(n), tracker(window) {}

  std::string name;
  std::string topic;
  std::string type;
  double expected_hz{0.0};  // 0 表示不检查频率
  double timeout_sec{1.0};
  RateTracker tracker;
  Status status{Status::NO_DATA};
  rclcpp::GenericSubscription::SharedPtr sub;
};

class RobotMonitor : public rclcpp::Node
{
public:
  RobotMonitor() : Node("robot_monitor")
  {
    const double window = declare_parameter("window_sec", 2.0);
    const double report_period = declare_parameter("report_period_sec", 1.0);
    const double summary_period = declare_parameter("summary_period_sec", 5.0);
    slow_ratio_ = declare_parameter("slow_ratio", 0.7);
    summary_every_ = std::max(1, static_cast<int>(summary_period / report_period + 0.5));

    const auto names = declare_parameter("topics", std::vector<std::string>{});
    auto qos = rclcpp::SensorDataQoS();

    for (const auto & name : names) {
      auto ch = std::make_unique<Channel>(name, window);
      ch->topic = declare_parameter(name + ".topic", std::string(""));
      ch->type = declare_parameter(name + ".type", std::string(""));
      ch->expected_hz = declare_parameter(name + ".expected_hz", 0.0);
      ch->timeout_sec = declare_parameter(name + ".timeout_sec", 1.0);

      if (ch->topic.empty() || ch->type.empty()) {
        RCLCPP_ERROR(get_logger(), "[%s] 话题/类型未配置，已跳过", name.c_str());
        continue;
      }

      Channel * raw = ch.get();
      ch->sub = create_generic_subscription(
        ch->topic, ch->type, qos,
        [this, raw](std::shared_ptr<rclcpp::SerializedMessage>) {
          raw->tracker.tick(now());
        });

      RCLCPP_INFO(get_logger(), "monitoring %-8s %s (%s) expect %.1f Hz, timeout %.1f s",
        name.c_str(), ch->topic.c_str(), ch->type.c_str(), ch->expected_hz, ch->timeout_sec);
      channels_.push_back(std::move(ch));
    }

    timer_ = create_wall_timer(
      std::chrono::duration<double>(report_period), [this]() { report(); });
  }

private:
  Status evaluate(Channel & ch, double rate, double age) const
  {
    if (age < 0.0) {
      return Status::NO_DATA;
    }
    if (age > ch.timeout_sec) {
      return Status::STALE;
    }
    if (ch.expected_hz > 0.0 && rate < slow_ratio_ * ch.expected_hz) {
      return Status::SLOW;
    }
    return Status::OK;
  }

  void report()
  {
    const auto t = now();
    const bool print_summary = (++tick_ % summary_every_ == 0);

    for (auto & ch : channels_) {
      const double rate = ch->tracker.rate(t);
      const double age = ch->tracker.age(t);
      const Status s = evaluate(*ch, rate, age);

      if (s != ch->status) {
        // 状态变化：立刻打印
        if (s == Status::OK) {
          RCLCPP_INFO(get_logger(), "[%s] %s -> OK  (%.1f Hz)",
            ch->topic.c_str(), to_str(ch->status), rate);
        } else {
          RCLCPP_WARN(get_logger(), "[%s] %s -> %s  (%.1f Hz, expect %.1f, last msg %.2f s ago)",
            ch->topic.c_str(), to_str(ch->status), to_str(s), rate, ch->expected_hz, age);
        }
        ch->status = s;
      } else if (print_summary) {
        // 状态不变：只在汇总周期打印一行
        if (s == Status::OK) {
          RCLCPP_INFO(get_logger(), "[%s] OK  %.1f Hz  age %.2f s",
            ch->topic.c_str(), rate, age);
        } else {
          RCLCPP_WARN(get_logger(), "[%s] %s  %.1f Hz  age %.2f s",
            ch->topic.c_str(), to_str(s), rate, age);
        }
      }
    }
  }

  std::vector<std::unique_ptr<Channel>> channels_;
  rclcpp::TimerBase::SharedPtr timer_;
  double slow_ratio_{0.7};
  int summary_every_{5};
  int tick_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotMonitor>());
  rclcpp::shutdown();
  return 0;
}
