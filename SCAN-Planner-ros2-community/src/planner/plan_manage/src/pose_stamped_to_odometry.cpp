#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace
{
double normalizeAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

double quaternionYaw(const geometry_msgs::msg::Quaternion &orientation)
{
  const double sin_yaw = 2.0 *
      (orientation.w * orientation.z + orientation.x * orientation.y);
  const double cos_yaw = 1.0 - 2.0 *
      (orientation.y * orientation.y + orientation.z * orientation.z);
  return std::atan2(sin_yaw, cos_yaw);
}

bool finitePose(const geometry_msgs::msg::Pose &pose)
{
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

std::string normalizeFrame(const std::string &frame)
{
  return !frame.empty() && frame.front() == '/' ? frame.substr(1) : frame;
}

class PoseStampedToOdometry : public rclcpp::Node
{
public:
  PoseStampedToOdometry()
  : Node("pose_stamped_to_odometry")
  {
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/pose_stamped");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/scan/body_odom");
    frame_id_ = normalizeFrame(declare_parameter<std::string>("frame_id", "map"));
    child_frame_id_ = normalizeFrame(
        declare_parameter<std::string>("child_frame_id", "body"));
    broadcast_sensor_tf_ = declare_parameter<bool>("broadcast_sensor_tf", false);
    sensor_frame_id_ = normalizeFrame(
        declare_parameter<std::string>("sensor_frame_id", "rslidar"));
    z_offset_ = declare_parameter<double>("z_offset", 0.0);
    velocity_filter_alpha_ = std::clamp(
        declare_parameter<double>("velocity_filter_alpha", 0.35), 0.0, 1.0);
    max_pose_gap_ = declare_parameter<double>("max_pose_gap", 0.5);
    max_linear_speed_ = declare_parameter<double>("max_linear_speed", 3.0);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SensorDataQoS());
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic_, rclcpp::SensorDataQoS(),
        std::bind(&PoseStampedToOdometry::poseCallback, this,
                  std::placeholders::_1));
    if (broadcast_sensor_tf_)
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(), "%s -> %s, frame=%s child=%s z_offset=%.3f",
                pose_topic_.c_str(), odom_topic_.c_str(), frame_id_.c_str(),
                child_frame_id_.c_str(), z_offset_);
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    if (!msg || !finitePose(msg->pose))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Rejecting non-finite localization pose");
      return;
    }

    const std::string input_frame = normalizeFrame(msg->header.frame_id);
    if (input_frame.empty() || input_frame != frame_id_)
    {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Input frame '%s' does not match '%s'; no TF is applied",
          msg->header.frame_id.c_str(), frame_id_.c_str());
      return;
    }

    const double quaternion_norm = std::sqrt(
        msg->pose.orientation.x * msg->pose.orientation.x +
        msg->pose.orientation.y * msg->pose.orientation.y +
        msg->pose.orientation.z * msg->pose.orientation.z +
        msg->pose.orientation.w * msg->pose.orientation.w);
    if (quaternion_norm < 1e-6)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Rejecting zero-length orientation");
      return;
    }

    nav_msgs::msg::Odometry odom;
    odom.header = msg->header;
    odom.header.frame_id = frame_id_;
    rclcpp::Time sample_time(msg->header.stamp, get_clock()->get_clock_type());
    if (sample_time.nanoseconds() == 0)
    {
      sample_time = now();
      odom.header.stamp = sample_time;
    }
    odom.child_frame_id = child_frame_id_;
    odom.pose.pose = msg->pose;
    odom.pose.pose.position.z += z_offset_;
    odom.pose.pose.orientation.x /= quaternion_norm;
    odom.pose.pose.orientation.y /= quaternion_norm;
    odom.pose.pose.orientation.z /= quaternion_norm;
    odom.pose.pose.orientation.w /= quaternion_norm;

    const double yaw = quaternionYaw(odom.pose.pose.orientation);
    if (have_previous_)
    {
      const double dt = (sample_time - previous_time_).seconds();
      if (dt > 1e-4 && dt <= max_pose_gap_)
      {
        const double vx =
            (odom.pose.pose.position.x - previous_pose_.position.x) / dt;
        const double vy =
            (odom.pose.pose.position.y - previous_pose_.position.y) / dt;
        const double vz =
            (odom.pose.pose.position.z - previous_pose_.position.z) / dt;
        const double speed = std::sqrt(vx * vx + vy * vy + vz * vz);
        if (std::isfinite(speed) && speed <= max_linear_speed_)
        {
          filtered_vx_ = velocity_filter_alpha_ * vx +
                         (1.0 - velocity_filter_alpha_) * filtered_vx_;
          filtered_vy_ = velocity_filter_alpha_ * vy +
                         (1.0 - velocity_filter_alpha_) * filtered_vy_;
          filtered_vz_ = velocity_filter_alpha_ * vz +
                         (1.0 - velocity_filter_alpha_) * filtered_vz_;
          const double wz = normalizeAngle(yaw - previous_yaw_) / dt;
          filtered_wz_ = velocity_filter_alpha_ * wz +
                         (1.0 - velocity_filter_alpha_) * filtered_wz_;
        }
        else
        {
          resetVelocity();
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                               "Pose jump detected; resetting velocity estimate");
        }
      }
      else
      {
        resetVelocity();
      }
    }

    // SCAN consumes velocity in the fixed map frame.
    odom.twist.twist.linear.x = filtered_vx_;
    odom.twist.twist.linear.y = filtered_vy_;
    odom.twist.twist.linear.z = filtered_vz_;
    odom.twist.twist.angular.z = filtered_wz_;
    odom_pub_->publish(odom);

    if (tf_broadcaster_)
    {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odom.header;
      transform.child_frame_id = sensor_frame_id_;
      transform.transform.translation.x = odom.pose.pose.position.x;
      transform.transform.translation.y = odom.pose.pose.position.y;
      transform.transform.translation.z = odom.pose.pose.position.z;
      transform.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }

    previous_pose_ = odom.pose.pose;
    previous_time_ = sample_time;
    previous_yaw_ = yaw;
    have_previous_ = true;
  }

  void resetVelocity()
  {
    filtered_vx_ = 0.0;
    filtered_vy_ = 0.0;
    filtered_vz_ = 0.0;
    filtered_wz_ = 0.0;
  }

  std::string pose_topic_;
  std::string odom_topic_;
  std::string frame_id_;
  std::string child_frame_id_;
  std::string sensor_frame_id_;
  bool broadcast_sensor_tf_{false};
  double z_offset_{0.0};
  double velocity_filter_alpha_{0.35};
  double max_pose_gap_{0.5};
  double max_linear_speed_{3.0};
  bool have_previous_{false};
  geometry_msgs::msg::Pose previous_pose_;
  rclcpp::Time previous_time_{0, 0, RCL_ROS_TIME};
  double previous_yaw_{0.0};
  double filtered_vx_{0.0};
  double filtered_vy_{0.0};
  double filtered_vz_{0.0};
  double filtered_wz_{0.0};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
} // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseStampedToOdometry>());
  rclcpp::shutdown();
  return 0;
}