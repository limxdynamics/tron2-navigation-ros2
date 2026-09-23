#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include "bspline_opt/uniform_bspline.h"
#include "scan_planner_msgs/msg/bspline.hpp"

namespace scan_planner
{
namespace
{
constexpr double kMaxYawRate = 1.0;

double normalizeAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

double clamp(double value, double minimum, double maximum)
{
  return std::max(minimum, std::min(maximum, value));
}

double quaternionYaw(const geometry_msgs::msg::Quaternion &orientation)
{
  const double sin_yaw = 2.0 *
      (orientation.w * orientation.z + orientation.x * orientation.y);
  const double cos_yaw = 1.0 - 2.0 *
      (orientation.y * orientation.y + orientation.z * orientation.z);
  return std::atan2(sin_yaw, cos_yaw);
}
} // namespace

class ClosedLoopController : public rclcpp::Node
{
public:
  ClosedLoopController()
  : Node("closed_loop_controller")
  {
    time_forward_ = declare_parameter<double>("time_forward", 0.8);
    lookahead_distance_ = declare_parameter<double>("lookahead_distance", 0.45);
    goal_slowdown_distance_ = declare_parameter<double>("goal_slowdown_distance", 0.50);
    heading_error_threshold_ =
        declare_parameter<double>("heading_error_threshold", 0.8);
    kp_pos_ = declare_parameter<double>("kp_pos", 0.8);
    kp_yaw_ = declare_parameter<double>("kp_yaw", 1.5);
    max_vx_ = declare_parameter<double>("max_vx", 0.08);
    max_vyaw_ = declare_parameter<double>("max_vyaw", 0.35);
    finish_dist_ = declare_parameter<double>("finish_dist", 0.15);
    odom_timeout_ = declare_parameter<double>("odom_timeout", 0.3);
    max_linear_accel_ = declare_parameter<double>("max_linear_accel", 0.5);
    max_angular_accel_ = declare_parameter<double>("max_angular_accel", 1.5);
    control_rate_ = declare_parameter<double>("control_rate", 50.0);
    forward_only_ = declare_parameter<bool>("forward_only", true);
    paused_ = declare_parameter<bool>("start_paused", false);
    body_pose_topic_ = declare_parameter<std::string>("body_pose_topic", "body_pose");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
    cancel_topic_ = declare_parameter<std::string>(
        "cancel_topic", "/scan_planner/cancel");
    preempt_topic_ = declare_parameter<std::string>(
        "preempt_topic", "/scan_planner/preempt");
    pause_topic_ = declare_parameter<std::string>(
        "pause_topic", "/scan_planner/pause");
    goal_status_topic_ = declare_parameter<std::string>(
        "goal_status_topic", "/scan_planner/local_goal_status");

    if (odom_timeout_ <= 0.0 || max_linear_accel_ <= 0.0 ||
        max_angular_accel_ <= 0.0 || control_rate_ <= 0.0 ||
        lookahead_distance_ <= 0.0 || goal_slowdown_distance_ <= 0.0 ||
        max_vx_ <= 0.0 || finish_dist_ <= 0.0)
      throw std::runtime_error("controller limits, timeout and rate must be positive");
    if (max_vyaw_ > kMaxYawRate)
    {
      RCLCPP_WARN(get_logger(), "Capping max_vyaw %.3f to %.3f rad/s",
                  max_vyaw_, kMaxYawRate);
      max_vyaw_ = kMaxYawRate;
    }

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 20);
    execution_frozen_pub_ = create_publisher<std_msgs::msg::Bool>(
        "planning/go2_execution_frozen", 10);
    bspline_sub_ = create_subscription<scan_planner_msgs::msg::Bspline>(
        "planning/bspline", 10,
        std::bind(&ClosedLoopController::bsplineCallback, this,
                  std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        body_pose_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ClosedLoopController::odomCallback, this,
                  std::placeholders::_1));
    cancel_sub_ = create_subscription<std_msgs::msg::Empty>(
        cancel_topic_, 2,
        std::bind(&ClosedLoopController::cancelCallback, this,
                  std::placeholders::_1));
    preempt_sub_ = create_subscription<std_msgs::msg::Empty>(
        preempt_topic_, 2,
        std::bind(&ClosedLoopController::cancelCallback, this,
                  std::placeholders::_1));
    pause_sub_ = create_subscription<std_msgs::msg::Bool>(
        pause_topic_, 2,
        std::bind(&ClosedLoopController::pauseCallback, this,
                  std::placeholders::_1));
    goal_status_sub_ = create_subscription<std_msgs::msg::String>(
        goal_status_topic_, rclcpp::QoS(10).reliable().transient_local(),
        std::bind(&ClosedLoopController::goalStatusCallback, this,
                  std::placeholders::_1));

    last_update_time_ = now();
    last_cmd_time_ = last_update_time_;
    const auto period = std::chrono::duration<double>(1.0 / control_rate_);
    cmd_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&ClosedLoopController::cmdCallback, this));

    publishStop();
    RCLCPP_WARN(get_logger(),
                "Ready; publishing nonholonomic commands to %s (%s)",
                cmd_vel_topic_.c_str(), paused_ ? "initially paused" : "active");
  }

  ~ClosedLoopController() override
  {
    if (rclcpp::ok())
      publishStop();
  }

  void stop()
  {
    receive_traj_ = false;
    publishExecutionFrozen(false);
    publishStop();
  }

private:
  void bsplineCallback(const scan_planner_msgs::msg::Bspline::ConstSharedPtr msg)
  {
    if (terminal_status_active_)
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Ignoring trajectory after terminal goal status");
      receive_traj_ = false;
      publishStop();
      return;
    }

    if (!msg || msg->order < 1 ||
        msg->pos_pts.size() <= static_cast<size_t>(msg->order) ||
        msg->knots.empty())
    {
      RCLCPP_ERROR(get_logger(), "Rejecting malformed B-spline trajectory");
      receive_traj_ = false;
      publishStop();
      return;
    }

    Eigen::MatrixXd position_points(3, msg->pos_pts.size());
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i)
      knots(i) = msg->knots[i];
    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
    {
      position_points(0, i) = msg->pos_pts[i].x;
      position_points(1, i) = msg->pos_pts[i].y;
      position_points(2, i) = msg->pos_pts[i].z;
    }

    UniformBspline position_traj(position_points, msg->order, 0.1);
    position_traj.setKnot(knots);
    trajectory_ = position_traj;
    traj_duration_ = trajectory_.getTimeSum();
    trajectory_samples_.clear();
    trajectory_arc_lengths_.clear();
    nearest_sample_index_ = 0;

    const int sample_count = std::max(
        2, static_cast<int>(std::ceil(traj_duration_ / 0.05)));
    trajectory_samples_.reserve(sample_count + 1);
    trajectory_arc_lengths_.reserve(sample_count + 1);
    for (int i = 0; i <= sample_count; ++i)
    {
      const double t = traj_duration_ * static_cast<double>(i) /
                       static_cast<double>(sample_count);
      const Eigen::Vector3d sample = trajectory_.evaluateDeBoorT(t);
      if (trajectory_samples_.empty())
        trajectory_arc_lengths_.push_back(0.0);
      else
        trajectory_arc_lengths_.push_back(
            trajectory_arc_lengths_.back() +
            (sample.head<2>() - trajectory_samples_.back().head<2>()).norm());
      trajectory_samples_.push_back(sample);
    }

    if (trajectory_arc_lengths_.back() < 1e-4)
    {
      receive_traj_ = false;
      rotating_in_place_ = false;
      publishExecutionFrozen(false);
      publishStop();
      RCLCPP_INFO(get_logger(), "Received stationary stop trajectory");
      return;
    }

    traj_id_ = msg->traj_id;
    last_update_time_ = now();
    receive_traj_ = true;
    RCLCPP_INFO(get_logger(), "Received B-spline traj_id=%d duration=%.3f",
                traj_id_, traj_duration_);
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;
    odom_yaw_ = quaternionYaw(msg->pose.pose.orientation);
    have_odom_ = true;
    last_odom_time_ = now();
  }

  void cancelCallback(const std_msgs::msg::Empty::ConstSharedPtr)
  {
    receive_traj_ = false;
    rotating_in_place_ = false;
    trajectory_samples_.clear();
    trajectory_arc_lengths_.clear();
    publishExecutionFrozen(false);
    publishStop();
    RCLCPP_WARN(get_logger(), "Trajectory cleared; command velocity stopped");
  }

  void pauseCallback(const std_msgs::msg::Bool::ConstSharedPtr msg)
  {
    paused_ = msg->data;
    last_update_time_ = now();
    if (paused_)
      publishStop();
    RCLCPP_WARN(get_logger(), "Execution %s", paused_ ? "paused" : "resumed");
  }

  void goalStatusCallback(const std_msgs::msg::String::ConstSharedPtr msg)
  {
    if (msg->data == "GOAL_RUNNING")
    {
      terminal_status_active_ = false;
      return;
    }
    if (msg->data != "GOAL_REACHED" && msg->data != "GOAL_FAILED" &&
        msg->data != "GOAL_CANCEL")
      return;

    terminal_status_active_ = true;
    receive_traj_ = false;
    rotating_in_place_ = false;
    trajectory_samples_.clear();
    trajectory_arc_lengths_.clear();
    publishExecutionFrozen(false);
    publishStop();
    RCLCPP_WARN(get_logger(), "Terminal status %s; command velocity stopped",
                msg->data.c_str());
  }

  double limitRate(double target, double previous, double max_rate, double dt) const
  {
    if (dt <= 0.0)
      return previous;
    const double max_delta = max_rate * dt;
    return previous + clamp(target - previous, -max_delta, max_delta);
  }

  void publishExecutionFrozen(bool frozen)
  {
    std_msgs::msg::Bool msg;
    msg.data = frozen;
    execution_frozen_pub_->publish(msg);
  }

  void publishStop(double yaw_rate = 0.0)
  {
    geometry_msgs::msg::Twist command;
    command.angular.z = clamp(yaw_rate, -max_vyaw_, max_vyaw_);
    cmd_vel_pub_->publish(command);
    last_linear_cmd_ = 0.0;
    last_angular_cmd_ = command.angular.z;
    last_cmd_time_ = now();
  }

  void publishRotationCommand(double angular, const rclcpp::Time &current_time)
  {
    double dt = (current_time - last_cmd_time_).seconds();
    if (dt <= 0.0 || dt > 0.2)
      dt = 1.0 / control_rate_;

    geometry_msgs::msg::Twist command;
    command.angular.z = limitRate(
        clamp(angular, -max_vyaw_, max_vyaw_), last_angular_cmd_,
        max_angular_accel_, dt);
    cmd_vel_pub_->publish(command);
    last_linear_cmd_ = 0.0;
    last_angular_cmd_ = command.angular.z;
    last_cmd_time_ = current_time;
  }

  void publishLimitedCommand(double linear, double angular,
                             const rclcpp::Time &current_time)
  {
    double dt = (current_time - last_cmd_time_).seconds();
    if (dt <= 0.0 || dt > 0.2)
      dt = 1.0 / control_rate_;

    geometry_msgs::msg::Twist command;
    command.linear.x = limitRate(
        linear, last_linear_cmd_, max_linear_accel_, dt);
    command.angular.z = limitRate(
        angular, last_angular_cmd_, max_angular_accel_, dt);
    cmd_vel_pub_->publish(command);
    last_linear_cmd_ = command.linear.x;
    last_angular_cmd_ = command.angular.z;
    last_cmd_time_ = current_time;
  }

  void cmdCallback()
  {
    const rclcpp::Time current_time = now();
    if (paused_)
    {
      publishExecutionFrozen(true);
      publishStop();
      last_update_time_ = current_time;
      return;
    }

    if (!receive_traj_ || !have_odom_ || trajectory_samples_.size() < 2)
    {
      publishExecutionFrozen(false);
      publishStop();
      last_update_time_ = current_time;
      return;
    }

    if ((current_time - last_odom_time_).seconds() > odom_timeout_)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                            "Odometry timeout; command velocity stopped");
      publishExecutionFrozen(true);
      publishStop();
      last_update_time_ = current_time;
      return;
    }

    size_t nearest = nearest_sample_index_;
    double nearest_distance_sq = std::numeric_limits<double>::max();
    const size_t search_begin = nearest_sample_index_ > 10
                                    ? nearest_sample_index_ - 10
                                    : 0;
    for (size_t i = search_begin; i < trajectory_samples_.size(); ++i)
    {
      const double distance_sq =
          (trajectory_samples_[i].head<2>() - odom_pos_.head<2>()).squaredNorm();
      if (distance_sq < nearest_distance_sq)
      {
        nearest_distance_sq = distance_sq;
        nearest = i;
      }
    }
    nearest_sample_index_ = std::max(nearest_sample_index_, nearest);

    const double target_arc = trajectory_arc_lengths_[nearest_sample_index_] +
                              lookahead_distance_;
    size_t lookahead_index = nearest_sample_index_;
    while (lookahead_index + 1 < trajectory_samples_.size() &&
           trajectory_arc_lengths_[lookahead_index] < target_arc)
      ++lookahead_index;

    const Eigen::Vector3d lookahead_position = trajectory_samples_[lookahead_index];
    const Eigen::Vector3d final_position = trajectory_samples_.back();
    const Eigen::Vector2d lookahead_delta =
        lookahead_position.head<2>() - odom_pos_.head<2>();
    const double lookahead_distance = lookahead_delta.norm();
    const double target_yaw = lookahead_distance > 0.05
                                  ? std::atan2(lookahead_delta(1), lookahead_delta(0))
                                  : odom_yaw_;
    const double yaw_error = normalizeAngle(target_yaw - odom_yaw_);
    const double rotation_command =
        clamp(kp_yaw_ * yaw_error, -max_vyaw_, max_vyaw_);

    const double absolute_yaw_error = std::abs(yaw_error);
    if (!rotating_in_place_ &&
        absolute_yaw_error > heading_error_threshold_)
      rotating_in_place_ = true;
    else if (rotating_in_place_ &&
             absolute_yaw_error < 0.5 * heading_error_threshold_)
      rotating_in_place_ = false;

    if (rotating_in_place_)
    {
      publishExecutionFrozen(true);
      publishRotationCommand(rotation_command, current_time);
      last_update_time_ = current_time;
      return;
    }

    publishExecutionFrozen(false);
    last_update_time_ = current_time;

    const double final_distance =
        (final_position.head<2>() - odom_pos_.head<2>()).norm();
    const double remaining_arc = trajectory_arc_lengths_.back() -
                                 trajectory_arc_lengths_[nearest_sample_index_];
    if (final_distance < finish_dist_ &&
        (nearest_sample_index_ + 1 >= trajectory_samples_.size() ||
         remaining_arc < finish_dist_))
    {
      publishStop();
      return;
    }

    const double slowdown = clamp(
        remaining_arc / goal_slowdown_distance_, 0.15, 1.0);
    double linear_command = max_vx_ * slowdown *
                            clamp(std::cos(yaw_error), 0.20, 1.0);
    if (!forward_only_ && std::abs(yaw_error) > M_PI_2)
      linear_command = -linear_command;

    const double curvature = lookahead_distance > 0.05
                                 ? 2.0 * std::sin(yaw_error) / lookahead_distance
                                 : 0.0;
    const double angular_command = clamp(
        linear_command * curvature + 0.25 * kp_yaw_ * yaw_error,
        -max_vyaw_, max_vyaw_);
    publishLimitedCommand(linear_command, angular_command, current_time);
  }

  bool receive_traj_{false};
  bool have_odom_{false};
  bool paused_{false};
  bool terminal_status_active_{false};
  bool rotating_in_place_{false};
  UniformBspline trajectory_;
  std::vector<Eigen::Vector3d> trajectory_samples_;
  std::vector<double> trajectory_arc_lengths_;
  size_t nearest_sample_index_{0};
  double traj_duration_{0.0};
  int traj_id_{0};
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  double odom_yaw_{0.0};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  double last_linear_cmd_{0.0};
  double last_angular_cmd_{0.0};

  double time_forward_{0.8};
  double lookahead_distance_{0.45};
  double goal_slowdown_distance_{0.5};
  double heading_error_threshold_{0.8};
  double kp_pos_{0.8};
  double kp_yaw_{1.5};
  double max_vx_{0.08};
  double max_vyaw_{0.35};
  double finish_dist_{0.15};
  double odom_timeout_{0.3};
  double max_linear_accel_{0.5};
  double max_angular_accel_{1.5};
  double control_rate_{50.0};
  bool forward_only_{true};
  std::string body_pose_topic_;
  std::string cmd_vel_topic_;
  std::string cancel_topic_;
  std::string preempt_topic_;
  std::string pause_topic_;
  std::string goal_status_topic_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execution_frozen_pub_;
  rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr preempt_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pause_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_status_sub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
};
} // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    auto controller = std::make_shared<scan_planner::ClosedLoopController>();
    rclcpp::spin(controller);
    controller->stop();
  }
  catch (const std::exception &error)
  {
    std::fprintf(stderr, "closed_loop_controller: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}