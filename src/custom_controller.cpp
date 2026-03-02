// ── Includes: project header first, then ROS2, then C++ stdlib ────────────────
#include "ur5e_rt_controller/controllers/pd_controller.hpp"
#include "ur5e_rt_controller/data_logger.hpp"
#include "ur5e_rt_controller/rt_controller_interface.hpp"
#include "ur5e_rt_controller/thread_config.hpp"
#include "ur5e_rt_controller/thread_utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <sys/mman.h>  // mlockall

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

using namespace std::chrono_literals;
namespace urtc = ur5e_rt_controller;

// ── CustomController ───────────────────────────────────────────────────────────
//
// 500 Hz position controller node with multi-threaded executors.
// 
// CallbackGroup assignment:
//   - cb_group_rt_:     control_timer_, timeout_timer_  (RT core)
//   - cb_group_sensor_: joint_state_sub_, target_sub_, hand_state_sub_  (Sensor core)
//   - cb_group_log_:    logging operations  (non-RT core)
//   - cb_group_aux_:    estop_pub_  (aux core)
class CustomController : public rclcpp::Node {
 public:
  CustomController()
      : Node("custom_controller"),
        controller_(std::make_unique<urtc::PDController>()),
        logger_(std::make_unique<urtc::DataLogger>("/tmp/ur5e_control_log.csv"))
  {
    CreateCallbackGroups();
    DeclareAndLoadParameters();
    CreateSubscriptions();
    CreatePublishers();
    CreateTimers();

    RCLCPP_INFO(get_logger(), "CustomController ready — %.0f Hz, E-STOP: %s",
                control_rate_, enable_estop_ ? "ON" : "OFF");
    RCLCPP_INFO(get_logger(), "CallbackGroups enabled: RT, Sensor, Log, Aux");
  }

  ~CustomController() override {
    if (logger_) {
      logger_->Flush();
    }
  }

  // Public accessors for main() to retrieve callback groups
  rclcpp::CallbackGroup::SharedPtr GetRtGroup()     const { return cb_group_rt_; }
  rclcpp::CallbackGroup::SharedPtr GetSensorGroup() const { return cb_group_sensor_; }
  rclcpp::CallbackGroup::SharedPtr GetLogGroup()    const { return cb_group_log_; }
  rclcpp::CallbackGroup::SharedPtr GetAuxGroup()    const { return cb_group_aux_; }

 private:
  // ── CallbackGroup creation ──────────────────────────────────────────────────
  void CreateCallbackGroups() {
    cb_group_rt_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_group_sensor_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_group_log_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_group_aux_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
  }

  // ── Initialisation helpers ──────────────────────────────────────────────────
  void DeclareAndLoadParameters() {
    declare_parameter("control_rate",    500.0);
    declare_parameter("kp",              5.0);
    declare_parameter("kd",              0.5);
    declare_parameter("enable_logging",  true);
    declare_parameter("robot_timeout_ms", 100.0);
    declare_parameter("hand_timeout_ms",  200.0);
    declare_parameter("enable_estop",    true);

    control_rate_   = get_parameter("control_rate").as_double();
    enable_logging_ = get_parameter("enable_logging").as_bool();
    enable_estop_   = get_parameter("enable_estop").as_bool();

    robot_timeout_ = std::chrono::milliseconds(
        static_cast<int>(get_parameter("robot_timeout_ms").as_double()));
    hand_timeout_ = std::chrono::milliseconds(
        static_cast<int>(get_parameter("hand_timeout_ms").as_double()));

    const urtc::PDController::Gains gains{
        .kp = get_parameter("kp").as_double(),
        .kd = get_parameter("kd").as_double(),
    };
    controller_->set_gains(gains);
  }

  void CreateSubscriptions() {
    // Assign subscriptions to cb_group_sensor_
    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = cb_group_sensor_;

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [this](sensor_msgs::msg::JointState::SharedPtr msg) {
          JointStateCallback(std::move(msg));
        },
        sub_options);

    target_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/target_joint_positions", 10,
        [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
          TargetCallback(std::move(msg));
        },
        sub_options);

    hand_state_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/hand/joint_states", 10,
        [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
          HandStateCallback(std::move(msg));
        },
        sub_options);
  }

  void CreatePublishers() {
    // Publishers are thread-safe, but assign to aux group for clarity
    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/forward_position_controller/commands", 10);
    estop_pub_ = create_publisher<std_msgs::msg::Bool>(
        "/system/estop_status", 10);
  }

  void CreateTimers() {
    const auto control_period = std::chrono::microseconds(
        static_cast<int>(1'000'000.0 / control_rate_));
    
    // Assign control_timer_ and timeout_timer_ to cb_group_rt_
    control_timer_ = create_wall_timer(
        control_period,
        [this]() { ControlLoop(); },
        cb_group_rt_);

    if (enable_estop_) {
      timeout_timer_ = create_wall_timer(
          20ms,
          [this]() { CheckTimeouts(); },
          cb_group_rt_);
    }
  }

  // ── Subscription callbacks ──────────────────────────────────────────────────
  void JointStateCallback(sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->position.size() < urtc::kNumRobotJoints) {
      return;
    }
    {
      std::lock_guard lock(state_mutex_);
      std::copy_n(msg->position.begin(), urtc::kNumRobotJoints,
                  current_positions_.begin());
      std::copy_n(msg->velocity.begin(), urtc::kNumRobotJoints,
                  current_velocities_.begin());
      last_robot_update_ = now();
      state_received_    = true;
    }
  }

  void TargetCallback(std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < urtc::kNumRobotJoints) {
      return;
    }
    {
      std::lock_guard lock(target_mutex_);
      std::copy_n(msg->data.begin(), urtc::kNumRobotJoints,
                  target_positions_.begin());
      target_received_ = true;
    }
    controller_->SetRobotTarget(target_positions_);
  }

  void HandStateCallback(std_msgs::msg::Float64MultiArray::SharedPtr /*msg*/) {
    std::lock_guard lock(hand_mutex_);
    last_hand_update_   = now();
    hand_data_received_ = true;
  }

  // ── 50 Hz watchdog (E-STOP) ─────────────────────────────────────────────────
  void CheckTimeouts() {
    const auto now_time = now();
    bool robot_timed_out = false;
    bool hand_timed_out  = false;

    {
      std::lock_guard lock(state_mutex_);
      if (state_received_) {
        robot_timed_out = (now_time - last_robot_update_) > robot_timeout_;
      }
    }
    {
      std::lock_guard lock(hand_mutex_);
      if (hand_data_received_) {
        hand_timed_out = (now_time - last_hand_update_) > hand_timeout_;
      }
    }

    if (robot_timed_out && !controller_->IsEstopped()) {
      RCLCPP_ERROR(get_logger(), "Robot data timeout — triggering E-STOP");
      controller_->TriggerEstop();
      PublishEstopStatus(true);
    }

    if (hand_timed_out) {
      controller_->SetHandEstop(true);
      if (!hand_estop_logged_) {
        RCLCPP_WARN(get_logger(), "Hand data timeout — hand E-STOP active");
        hand_estop_logged_ = true;
      }
    } else {
      if (hand_estop_logged_) {
        RCLCPP_INFO(get_logger(), "Hand data restored — hand E-STOP cleared");
        hand_estop_logged_ = false;
      }
      controller_->SetHandEstop(false);
    }
  }

  // ── 500 Hz control loop ─────────────────────────────────────────────────────
  void ControlLoop() {
    if (!state_received_ || !target_received_) {
      return;
    }

    urtc::ControllerState state{};
    {
      std::lock_guard lock(state_mutex_);
      state.robot.positions  = current_positions_;
      state.robot.velocities = current_velocities_;
    }
    {
      std::lock_guard lock(target_mutex_);
      state.robot.dt        = 1.0 / control_rate_;
      state.iteration       = loop_count_;
    }

    const urtc::ControllerOutput output = controller_->Compute(state);

    std_msgs::msg::Float64MultiArray cmd_msg;
    cmd_msg.data.assign(output.robot_commands.begin(),
                        output.robot_commands.end());
    command_pub_->publish(cmd_msg);

    if (enable_logging_ && logger_) {
      logger_->LogControlData(now().seconds(),
                              state.robot.positions,
                              target_positions_,
                              output.robot_commands);
    }

    ++loop_count_;
    if (loop_count_ % 500 == 0) {
      RCLCPP_DEBUG(get_logger(), "ControlLoop: %zu iterations", loop_count_);
    }
  }

  void PublishEstopStatus(bool estopped) {
    std_msgs::msg::Bool msg;
    msg.data = estopped;
    estop_pub_->publish(msg);
  }

  // ── ROS2 handles ────────────────────────────────────────────────────────────
  rclcpp::CallbackGroup::SharedPtr cb_group_rt_;
  rclcpp::CallbackGroup::SharedPtr cb_group_sensor_;
  rclcpp::CallbackGroup::SharedPtr cb_group_log_;
  rclcpp::CallbackGroup::SharedPtr cb_group_aux_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr      joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr  target_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr  hand_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr     command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                  estop_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;

  // ── Domain objects ──────────────────────────────────────────────────────────
  std::unique_ptr<urtc::PDController> controller_;
  std::unique_ptr<urtc::DataLogger>   logger_;

  // ── Shared state (guarded by per-domain mutexes) ────────────────────────────
  std::array<double, urtc::kNumRobotJoints> current_positions_{};
  std::array<double, urtc::kNumRobotJoints> current_velocities_{};
  std::array<double, urtc::kNumRobotJoints> target_positions_{};

  mutable std::mutex state_mutex_;
  mutable std::mutex target_mutex_;
  mutable std::mutex hand_mutex_;

  bool state_received_{false};
  bool target_received_{false};
  bool hand_data_received_{false};

  rclcpp::Time              last_robot_update_;
  rclcpp::Time              last_hand_update_;
  std::chrono::milliseconds robot_timeout_{100};
  std::chrono::milliseconds hand_timeout_{200};

  // ── Parameters ──────────────────────────────────────────────────────────────
  double control_rate_{500.0};
  bool   enable_logging_{true};
  bool   enable_estop_{true};
  bool   hand_estop_logged_{false};

  std::size_t loop_count_{0};
};

// ── Entry point ────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // 1. Lock all current and future pages in memory (prevent page faults)
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    fprintf(stderr, "[WARN] mlockall failed — page faults possible\n");
  }

  auto node = std::make_shared<CustomController>();

  // 2. Create executors for each callback group
  rclcpp::executors::SingleThreadedExecutor rt_executor;
  rclcpp::executors::SingleThreadedExecutor sensor_executor;
  rclcpp::executors::SingleThreadedExecutor log_executor;
  rclcpp::executors::SingleThreadedExecutor aux_executor;

  // 3. Add callback groups to respective executors
  rt_executor.add_callback_group(
      node->GetRtGroup(), node->get_node_base_interface());
  sensor_executor.add_callback_group(
      node->GetSensorGroup(), node->get_node_base_interface());
  log_executor.add_callback_group(
      node->GetLogGroup(), node->get_node_base_interface());
  aux_executor.add_callback_group(
      node->GetAuxGroup(), node->get_node_base_interface());

  // 4. Helper lambda to create thread with RT config
  auto make_thread = [](auto& executor, const urtc::ThreadConfig& cfg) {
    return std::thread([&executor, cfg]() {
      if (!urtc::ApplyThreadConfig(cfg)) {
        fprintf(stderr, "[WARN] Thread config failed for '%s' (need realtime permissions)\n",
                cfg.name.c_str());
      } else {
        fprintf(stdout, "[INFO] Thread '%s' configured:\n%s",
                cfg.name.c_str(),
                urtc::VerifyThreadConfig().c_str());
      }
      executor.spin();
    });
  };

  // 5. Launch threads with respective configurations
  auto t_rt     = make_thread(rt_executor,     urtc::kRtControlConfig);
  auto t_sensor = make_thread(sensor_executor, urtc::kSensorConfig);
  auto t_log    = make_thread(log_executor,    urtc::kLoggingConfig);
  auto t_aux    = make_thread(aux_executor,    urtc::kAuxConfig);

  // 6. Wait for threads to finish
  t_rt.join();
  t_sensor.join();
  t_log.join();
  t_aux.join();

  rclcpp::shutdown();
  return 0;
}
