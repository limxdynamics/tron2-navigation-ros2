
#include <plan_manage/scan_replan_fsm.h>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

namespace
{
  template <typename T>
  T load_parameter(rclcpp::Node *node, const std::string &name, const T &default_value)
  {
    if (!node->has_parameter(name)) node->declare_parameter<T>(name, default_value);
    return node->get_parameter(name).get_value<T>();
  }

  Eigen::Vector3d clamp_velocity_norm(const Eigen::Vector3d &velocity, double max_norm)
  {
    const double norm = velocity.norm();
    if (max_norm > 0.0 && norm > max_norm)
      return velocity * (max_norm / norm);
    return velocity;
  }
} // namespace

namespace scan_planner
{

  void SCANReplanFSM::init(rclcpp::Node *node)
  {
    node_ = node;
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    trigger_ = false;
    have_target_ = false;
    have_odom_ = false;
    have_new_target_ = false;
    rviz_height_ready_ = false;
    rviz_goal_height_ = 0.0;
    go2_execution_frozen_ = false;
    flag_escape_emergency_ = true;
    need_hover_stop_ = false;
    replan_fail_count_ = 0;
    last_freeze_update_time_ = node_->now();
    last_replan_attempt_time_ = rclcpp::Time(
      0, 0, node_->get_clock()->get_clock_type());

    /*  fsm param  */
    navi_mode_ = load_parameter<int>(node_, "fsm.navi_mode", -1);
    replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_replan", -1.0);
    no_replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_no_replan", -1.0);
    planning_horizon_ = load_parameter<double>(node_, "fsm.planning_horizon", -1.0);
    emergency_time_ = load_parameter<double>(node_, "fsm.emergency_time", 1.0);
    enable_fail_safe_ = load_parameter<bool>(node_, "fsm.fail_safe", true);
    max_replan_fail_count_ = load_parameter<int>(node_, "fsm.max_replan_fail_count", 1000);
    replan_retry_interval_ = load_parameter<double>(node_, "fsm.replan_retry_interval", 0.1);
    self_inflation_z_up_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_up", 0.0);
    self_inflation_z_down_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_down", 0.0);
    self_double_cylinder_radius_ = load_parameter<double>(node_, "grid_map.double_cylinder_radius", 0.0);
    self_double_cylinder_offset_ = load_parameter<double>(node_, "grid_map.double_cylinder_offset", 0.0);
    body_height_ = load_parameter<double>(node_, "grid_map.body_height", 0.4);
    self_inflation_frame_id_ = load_parameter<std::string>(node_, "grid_map.frame_id", "world");
    reference_path_spacing_ = load_parameter<double>(node_, "fsm.reference_path_spacing", 0.3);
    max_reference_waypoints_ = load_parameter<int>(node_, "fsm.max_reference_waypoints", 200);
    reference_path_topic_ = load_parameter<std::string>(node_, "fsm.reference_path_topic", "initial_path");
    require_reference_path_start_near_odom_ = load_parameter<bool>(
      node_, "fsm.require_reference_path_start_near_odom", false);
    reference_path_start_tolerance_ = load_parameter<double>(
      node_, "fsm.reference_path_start_tolerance", 0.5);
    align_reference_path_z_to_odom_ = load_parameter<bool>(
      node_, "fsm.align_reference_path_z_to_odom", false);
    max_reference_path_z_alignment_ = load_parameter<double>(
      node_, "fsm.max_reference_path_z_alignment", 3.0);
    goal_tolerance_ = load_parameter<double>(node_, "fsm.goal_tolerance", 0.3);
    goal_z_tolerance_ = load_parameter<double>(node_, "fsm.goal_z_tolerance", 0.5);
    reference_goal_search_radius_ = load_parameter<double>(
      node_, "fsm.reference_goal_search_radius", 1.0);
    reference_goal_clearance_ = load_parameter<double>(
      node_, "fsm.reference_goal_clearance", 0.2);
    goal_status_topic_ = load_parameter<std::string>(
      node_, "fsm.goal_status_topic", "/scan_planner/local_goal_status");

    if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
    {
      const auto flat_waypoints = load_parameter<std::vector<double>>(node_, "fsm.waypoints", {});
      if (flat_waypoints.empty() || flat_waypoints.size() % 3 != 0)
        throw std::runtime_error("navi_mode=2 requires non-empty fsm.waypoints with x,y,z triples");
      waypoint_num_ = static_cast<int>(flat_waypoints.size() / 3);
      preset_waypoints_.resize(waypoint_num_);
      for (int i = 0; i < waypoint_num_; i++)
      {
        preset_waypoints_[i] = Eigen::Vector3d(flat_waypoints[3 * i], flat_waypoints[3 * i + 1],
                                               flat_waypoints[3 * i + 2]);
      }
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(node_));
    planner_manager_.reset(new SCANPlannerManager);
    planner_manager_->initPlanModules(node_, visualization_);

    /* callback */
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10),
                                           std::bind(&SCANReplanFSM::execFSMCallback, this));
    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                             std::bind(&SCANReplanFSM::checkCollisionCallback, this));
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&SCANReplanFSM::odometryCallback, this, std::placeholders::_1));
    go2_execution_frozen_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "planning/go2_execution_frozen", 10,
        std::bind(&SCANReplanFSM::go2ExecutionFrozenCallback, this, std::placeholders::_1));
    cancel_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
      "/scan_planner/cancel", 2,
      std::bind(&SCANReplanFSM::cancelCallback, this, std::placeholders::_1));
    preempt_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
      "/scan_planner/preempt", 2,
      std::bind(&SCANReplanFSM::preemptCallback, this, std::placeholders::_1));

    bspline_pub_ = node_->create_publisher<scan_planner_msgs::msg::Bspline>("planning/bspline", 10);
    data_disp_pub_ = node_->create_publisher<scan_planner_msgs::msg::DataDisp>("planning/data_display", 100);
    self_inflation_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "self_inflation", rclcpp::QoS(1).reliable().transient_local());
    goal_status_pub_ = node_->create_publisher<std_msgs::msg::String>(
      goal_status_topic_, rclcpp::QoS(1).reliable().transient_local());

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET)
      goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "move_base_simple/goal", 1,
          std::bind(&SCANReplanFSM::rvizGoalCallback, this, std::placeholders::_1));
    else if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
    {
      path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
        reference_path_topic_, rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&SCANReplanFSM::pathCallback, this, std::placeholders::_1));
      RCLCPP_INFO(node_->get_logger(),
            "REFERENCE_PATH mode: subscribing to %s",
            reference_path_topic_.c_str());
    }
    else if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
      RCLCPP_INFO(node_->get_logger(), "Preset waypoint mode will start after the first odometry message");
    else
      throw std::runtime_error("fsm.navi_mode must be 1, 2, or 3");
  }

  void SCANReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps = preset_waypoints_;

    for (size_t i = 0; i < wps.size(); i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
    }

    active_waypoints_ = wps;
    current_wp_ = 0;
    trigger_ = true;
    init_pt_ = odom_pos_;

    if (planNextWaypoint())
    {
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory to first preset waypoint");
    }
  }

  void SCANReplanFSM::rvizGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
  {
    if (!msg)
      return;

    if (!rviz_height_ready_)
    {
      RCLCPP_WARN(node_->get_logger(), "Ignore RViz goal before receiving initial body pose");
      return;
    }

    auto path = std::make_shared<nav_msgs::msg::Path>();
    path->header = msg->header;
    path->poses.push_back(*msg);
    waypointCallback(path);
  }

  void SCANReplanFSM::waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    if (!msg || msg->poses.empty())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Empty waypoint message; ignoring");
      return;
    }

    if (msg->poses[0].pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;
    trigger_ = true;
    init_pt_ = odom_pos_;

    bool success = false;
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, rviz_goal_height_;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success)
      success = adjustGlobalTargetIfOccupied();

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
      publishGoalStatus("GOAL_RUNNING");
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory");
      publishGoalStatus("GOAL_FAILED");
    }
  }

  bool SCANReplanFSM::planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints)
  {
    if (waypoints.empty())
    {
      RCLCPP_WARN(node_->get_logger(), "No waypoint supplied for global trajectory");
      return false;
    }

    end_pt_ = waypoints.back();

    for (size_t i = 0; i < waypoints.size(); i++)
    {
      visualization_->displayGoalPoint(waypoints[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
    }

    const Eigen::Vector3d bounded_odom_vel = clamp_velocity_norm(
      odom_vel_, planner_manager_->pp_.max_vel_);
    if ((bounded_odom_vel - odom_vel_).norm() > 1e-6)
    {
      RCLCPP_WARN(node_->get_logger(),
            "[reference path] Clamp measured start speed from %.3f to %.3f m/s",
            odom_vel_.norm(), bounded_odom_vel.norm());
    }

    bool success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_,
      bounded_odom_vel,
        Eigen::Vector3d::Zero(),
        waypoints,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from waypoints");
      return false;
    }

    if (navi_mode_ != NAVI_MODE::REFERENCE_PATH &&
      !adjustGlobalTargetIfOccupied())
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, static_cast<int>(waypoints.size()) - 1);

    return true;
  }

  bool SCANReplanFSM::planNextWaypoint()
  {
    if (current_wp_ < 0 || current_wp_ >= (int)active_waypoints_.size())
    {
      RCLCPP_WARN(node_->get_logger(), "[navi_mode=%d] No active waypoint to plan", navi_mode_);
      return false;
    }

    end_pt_ = active_waypoints_[current_wp_];
    setStartStateFromOdomOrCurrentTraj();

    bool success = planner_manager_->planGlobalTraj(
        start_pt_,
        start_vel_,
        start_acc_,
        end_pt_,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      RCLCPP_ERROR(node_->get_logger(), "[navi_mode=%d] Unable to generate trajectory to waypoint %d",
                   navi_mode_, current_wp_ + 1);
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, current_wp_);
    RCLCPP_INFO(node_->get_logger(), "[navi_mode=%d] Planning to waypoint %d/%zu: [%.2f, %.2f, %.2f]",
                navi_mode_, current_wp_ + 1, active_waypoints_.size(), end_pt_(0), end_pt_(1), end_pt_(2));

    return true;
  }

  bool SCANReplanFSM::isWaypointSequenceMode() const
  {
    return navi_mode_ == NAVI_MODE::PRESET_TARGET;
  }

  bool SCANReplanFSM::adjustGlobalTargetIfOccupied()
  {
    auto map = planner_manager_->grid_map_;
    auto &global_data = planner_manager_->global_data_;
    const double duration = global_data.global_duration_;
    if (!map || duration < 1e-3)
      return true;

    constexpr double sample_dt = 0.05;
    const int sample_num = std::max(1, static_cast<int>(std::ceil(duration / sample_dt)));
    const Eigen::Vector3d final_pt = global_data.global_traj_.evaluate(duration);
    const Eigen::Vector3d final_prev = global_data.global_traj_.evaluate(duration * (sample_num - 1) / sample_num);
    const int final_occ = map->getInflateOccupancy(final_pt, estimateYawFromSegment(final_prev, final_pt));
    if (final_occ <= 0)
      return true;

    for (int i = sample_num; i >= 0; --i)
    {
      const double t = duration * i / sample_num;
      const double prev_t = duration * std::max(0, i - 1) / sample_num;
      const Eigen::Vector3d pt = global_data.global_traj_.evaluate(t);
      const Eigen::Vector3d prev_pt = global_data.global_traj_.evaluate(prev_t);

      if (map->getInflateOccupancy(pt, estimateYawFromSegment(prev_pt, pt)) == 0)
      {
        const Eigen::Vector3d raw_end = end_pt_;
        end_pt_ = pt;
        global_data.global_duration_ = t;
        global_data.last_progress_time_ = std::min(global_data.last_progress_time_, t);
        RCLCPP_WARN(node_->get_logger(),
                    "Target [%.2f, %.2f, %.2f] is occupied; using [%.2f, %.2f, %.2f]",
                    raw_end(0), raw_end(1), raw_end(2), end_pt_(0), end_pt_(1), end_pt_(2));
        return true;
      }
    }

    RCLCPP_ERROR(node_->get_logger(),
                 "Target is occupied and no collision-free point was found on the global trajectory");
    return false;
  }

  bool SCANReplanFSM::adjustReferenceGoalIfOccupied()
  {
    auto map = planner_manager_->grid_map_;
    if (!map || reference_path_points_.size() < 2)
      return false;

    const Eigen::Vector3d previous =
        reference_path_points_[reference_path_points_.size() - 2];
    const double approach_yaw = estimateYawFromSegment(previous, end_pt_);
    const int endpoint_occupancy = map->getInflateOccupancy(end_pt_, approach_yaw);
    if (endpoint_occupancy <= 0)
      return true;

    const double resolution = map->getResolution();
    const double search_radius = std::max(reference_goal_search_radius_, resolution);
    const double required_clearance = std::max(reference_goal_clearance_, resolution);
    const Eigen::Vector3d requested_goal = end_pt_;

    auto has_clearance = [&](const Eigen::Vector3d &candidate) {
      for (double radius = 0.0; radius <= required_clearance + 1e-6;
           radius += resolution)
      {
        const int samples = radius < 0.5 * resolution
                                ? 1
                                : std::max(8, static_cast<int>(
                                                  std::ceil(2.0 * M_PI * radius / resolution)));
        for (int i = 0; i < samples; ++i)
        {
          const double angle = 2.0 * M_PI * static_cast<double>(i) / samples;
          Eigen::Vector3d probe = candidate;
          probe(0) += radius * std::cos(angle);
          probe(1) += radius * std::sin(angle);
          if (map->getInflateOccupancy(probe, approach_yaw) != 0)
            return false;
        }
      }
      return true;
    };

    bool found = false;
    Eigen::Vector3d adjusted_goal = requested_goal;
    for (double radius = resolution; radius <= search_radius + 1e-6 && !found;
         radius += resolution)
    {
      const int samples = std::max(
          24, static_cast<int>(std::ceil(2.0 * M_PI * radius / resolution)));
      double best_distance_to_robot = std::numeric_limits<double>::infinity();
      for (int i = 0; i < samples; ++i)
      {
        const double angle = 2.0 * M_PI * static_cast<double>(i) / samples;
        Eigen::Vector3d candidate = requested_goal;
        candidate(0) += radius * std::cos(angle);
        candidate(1) += radius * std::sin(angle);
        if (!has_clearance(candidate))
          continue;

        const double distance_to_robot =
            (candidate.head<2>() - odom_pos_.head<2>()).norm();
        if (distance_to_robot < best_distance_to_robot)
        {
          best_distance_to_robot = distance_to_robot;
          adjusted_goal = candidate;
          found = true;
        }
      }
    }

    if (!found)
    {
      RCLCPP_ERROR_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "[reference path] Occupied endpoint has no free replacement within %.2fm",
          search_radius);
      return false;
    }

    end_pt_ = adjusted_goal;
    reference_path_points_.back() = adjusted_goal;
    reference_path_arc_lengths_.assign(reference_path_points_.size(), 0.0);
    for (size_t i = 1; i < reference_path_points_.size(); ++i)
    {
      reference_path_arc_lengths_[i] = reference_path_arc_lengths_[i - 1] +
          (reference_path_points_[i].head<2>() -
           reference_path_points_[i - 1].head<2>()).norm();
    }

    visualization_->displayGoalPoint(
        end_pt_, Eigen::Vector4d(1.0, 0.65, 0.0, 1.0), 0.3, 0);
    RCLCPP_WARN(node_->get_logger(),
                "[reference path] Occupied endpoint [%.2f, %.2f, %.2f] "
                "replaced by [%.2f, %.2f, %.2f]",
                requested_goal(0), requested_goal(1), requested_goal(2),
                end_pt_(0), end_pt_(1), end_pt_(2));
    return true;
  }

  void SCANReplanFSM::pathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    if (!msg)
      return;

    if (msg->poses.empty())
    {
      RCLCPP_WARN(node_->get_logger(),
                  "Reference path on %s was cleared; stopping navigation",
                  reference_path_topic_.c_str());
      stopCurrentNavigation(true);
      return;
    }

    auto normalize_frame = [](const std::string &frame) {
      return !frame.empty() && frame.front() == '/' ? frame.substr(1) : frame;
    };
    const std::string planning_frame = normalize_frame(self_inflation_frame_id_);
    const std::string path_frame = msg->header.frame_id.empty()
                                       ? planning_frame
                                       : normalize_frame(msg->header.frame_id);
    if (path_frame != planning_frame)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "Path frame '%s' does not match planning frame '%s'; "
                   "TF conversion is not implicit",
                   path_frame.c_str(), planning_frame.c_str());
      publishGoalStatus("GOAL_FAILED");
      return;
    }

    if (!have_odom_)
    {
      pending_reference_path_ = std::make_shared<nav_msgs::msg::Path>(*msg);
      RCLCPP_WARN(node_->get_logger(),
                  "Caching reference path until body odometry is available");
      return;
    }

    const auto &first_position = msg->poses.front().pose.position;
    if (!std::isfinite(first_position.x) || !std::isfinite(first_position.y) ||
        !std::isfinite(first_position.z))
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "Reference path starts with a non-finite waypoint");
      publishGoalStatus("GOAL_FAILED");
      return;
    }

    const double start_xy_distance = std::hypot(
        first_position.x - odom_pos_(0), first_position.y - odom_pos_(1));
    if (require_reference_path_start_near_odom_ &&
        start_xy_distance > reference_path_start_tolerance_)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "Reject reference path: first point is %.3fm from current pose "
                   "(limit %.3fm)",
                   start_xy_distance, reference_path_start_tolerance_);
      publishGoalStatus("GOAL_FAILED");
      return;
    }

    double path_z_alignment = 0.0;
    if (align_reference_path_z_to_odom_)
    {
      path_z_alignment = odom_pos_(2) - (first_position.z + body_height_);
      if (std::abs(path_z_alignment) > max_reference_path_z_alignment_)
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "Reject reference path: required Z alignment %+.3fm exceeds %.3fm",
                     path_z_alignment, max_reference_path_z_alignment_);
        publishGoalStatus("GOAL_FAILED");
        return;
      }
      RCLCPP_INFO(node_->get_logger(),
                  "Reference start error: xy=%.3fm; apply uniform Z alignment %+.3fm",
                  start_xy_distance, path_z_alignment);
    }

    trigger_ = true;
    have_target_ = false;
    have_new_target_ = false;
    need_hover_stop_ = false;
    flag_escape_emergency_ = true;
    replan_fail_count_ = 0;
    last_replan_attempt_time_ = rclcpp::Time(
      0, 0, node_->get_clock()->get_clock_type());

    std::vector<Eigen::Vector3d> waypoints;
    waypoints.reserve(msg->poses.size());
    std::vector<Eigen::Vector3d> reference_points;
    reference_points.reserve(msg->poses.size());
    Eigen::Vector3d final_waypoint = odom_pos_;
    bool have_final_waypoint = false;

    for (size_t i = 0; i < msg->poses.size(); ++i)
    {
      const auto &pose_stamped = msg->poses[i];
      if (!pose_stamped.header.frame_id.empty() &&
          normalize_frame(pose_stamped.header.frame_id) != planning_frame)
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "Pose %zu frame '%s' does not match '%s'",
                     i, pose_stamped.header.frame_id.c_str(), planning_frame.c_str());
        publishGoalStatus("GOAL_FAILED");
        return;
      }

      Eigen::Vector3d wp;
      wp(0) = pose_stamped.pose.position.x;
      wp(1) = pose_stamped.pose.position.y;
      wp(2) = pose_stamped.pose.position.z + body_height_ + path_z_alignment;
      if (!std::isfinite(wp(0)) || !std::isfinite(wp(1)) || !std::isfinite(wp(2)))
      {
        RCLCPP_WARN(node_->get_logger(), "Skipping non-finite waypoint %zu", i);
        continue;
      }

      final_waypoint = wp;
      have_final_waypoint = true;
      if (reference_points.empty() ||
          (wp.head<2>() - reference_points.back().head<2>()).norm() >= 1e-3)
        reference_points.push_back(wp);

      if (waypoints.empty() && i + 1 < msg->poses.size() &&
          (wp - odom_pos_).norm() < 0.5 * reference_path_spacing_)
        continue;
      if (waypoints.empty() ||
          (wp - waypoints.back()).norm() >= reference_path_spacing_)
        waypoints.push_back(wp);
    }

    if (have_final_waypoint &&
        (waypoints.empty() || (final_waypoint - waypoints.back()).norm() > 1e-6))
      waypoints.push_back(final_waypoint);

    if (max_reference_waypoints_ >= 2 &&
        waypoints.size() > static_cast<size_t>(max_reference_waypoints_))
    {
      std::vector<Eigen::Vector3d> reduced;
      reduced.reserve(max_reference_waypoints_);
      const double scale = static_cast<double>(waypoints.size() - 1) /
                           static_cast<double>(max_reference_waypoints_ - 1);
      for (int i = 0; i < max_reference_waypoints_; ++i)
      {
        const size_t index = static_cast<size_t>(std::round(i * scale));
        reduced.push_back(waypoints[std::min(index, waypoints.size() - 1)]);
      }
      waypoints.swap(reduced);
    }

    if (waypoints.empty() || reference_points.size() < 2)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "Reference path contains insufficient usable waypoints");
      publishGoalStatus("GOAL_FAILED");
      return;
    }

    if ((waypoints.back().head<2>() - odom_pos_.head<2>()).norm() <= goal_tolerance_ &&
        std::abs(waypoints.back()(2) - odom_pos_(2)) <= goal_z_tolerance_)
    {
      RCLCPP_INFO(node_->get_logger(), "Reference goal is already reached");
      trigger_ = false;
      publishGoalStatus("GOAL_REACHED");
      return;
    }

    bool success = planGlobalTrajByWaypoints(waypoints);

    if (success)
    {
      reference_path_points_ = std::move(reference_points);
      reference_path_arc_lengths_.assign(reference_path_points_.size(), 0.0);
      for (size_t i = 1; i < reference_path_points_.size(); ++i)
      {
        reference_path_arc_lengths_[i] = reference_path_arc_lengths_[i - 1] +
            (reference_path_points_[i].head<2>() -
             reference_path_points_[i - 1].head<2>()).norm();
      }
      reference_progress_s_ = 0.0;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET || exec_state_ == INIT ||
          exec_state_ == EMERGENCY_STOP)
      {
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      RCLCPP_INFO(node_->get_logger(),
                  "Accepted %zu/%zu reference waypoints in frame %s",
                  waypoints.size(), msg->poses.size(), planning_frame.c_str());
      publishGoalStatus("GOAL_RUNNING");
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from reference path");
      callEmergencyStop(odom_pos_);
      trigger_ = false;
      publishGoalStatus("GOAL_FAILED");
    }
  }

  void SCANReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
  {
    const bool first_odom = !have_odom_;
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET && !rviz_height_ready_)
    {
      rviz_goal_height_ = odom_pos_(2);
      rviz_height_ready_ = true;
      RCLCPP_INFO(node_->get_logger(), "Set RViz goal height from initial body_pose z: %.3f", rviz_goal_height_);
    }

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
    publishSelfInflationMarker();

    if (first_odom && pending_reference_path_)
    {
      auto pending_path = pending_reference_path_;
      pending_reference_path_.reset();
      RCLCPP_INFO(node_->get_logger(),
                  "Body odometry ready; processing cached reference path");
      pathCallback(pending_path);
    }

    if (navi_mode_ == NAVI_MODE::PRESET_TARGET && !preset_started_)
    {
      preset_started_ = true;
      planGlobalTrajbyGivenWps();
    }
  }

  void SCANReplanFSM::go2ExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg)
  {
    go2_execution_frozen_ = msg->data;
  }

  void SCANReplanFSM::cancelCallback(const std_msgs::msg::Empty::ConstSharedPtr &)
  {
    stopCurrentNavigation(true);
  }

  void SCANReplanFSM::preemptCallback(const std_msgs::msg::Empty::ConstSharedPtr &)
  {
    stopCurrentNavigation(false);
  }

  void SCANReplanFSM::stopCurrentNavigation(bool report_cancel)
  {
    if (have_odom_)
      callEmergencyStop(odom_pos_);

    trigger_ = false;
    have_target_ = false;
    have_new_target_ = false;
    active_waypoints_.clear();
    reference_path_points_.clear();
    reference_path_arc_lengths_.clear();
    reference_progress_s_ = 0.0;
    pending_reference_path_.reset();
    current_wp_ = 0;
    need_hover_stop_ = false;
    flag_escape_emergency_ = false;
    replan_fail_count_ = 0;
    changeFSMExecState(WAIT_TARGET, report_cancel ? "CANCEL" : "PREEMPT");
    if (report_cancel)
      publishGoalStatus("GOAL_CANCEL");
    RCLCPP_WARN(node_->get_logger(), "Navigation %s",
                report_cancel ? "cancelled" : "preempted");
  }

  void SCANReplanFSM::publishGoalStatus(const std::string &status)
  {
    std_msgs::msg::String msg;
    msg.data = status;
    goal_status_pub_->publish(msg);
  }

  void SCANReplanFSM::updateLocalTrajTimeFreeze()
  {
    const rclcpp::Time now = node_->now();
    double dt = (now - last_freeze_update_time_).seconds();
    last_freeze_update_time_ = now;

    if (dt <= 0.0 || dt > 0.2)
      return;

    LocalTrajData *info = &planner_manager_->local_data_;
    if (go2_execution_frozen_ && info->start_time_.seconds() > 1e-5)
      info->start_time_ += rclcpp::Duration::from_seconds(dt);
  }

  double SCANReplanFSM::getOdomYaw() const
  {
    Eigen::Vector3d heading = odom_orient_.toRotationMatrix().col(0);
    if (heading.head<2>().squaredNorm() < 1e-8)
      return 0.0;
    return std::atan2(heading(1), heading(0));
  }

  double SCANReplanFSM::estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const
  {
    Eigen::Vector2d diff(to(0) - from(0), to(1) - from(1));
    if (diff.squaredNorm() < 1e-8)
      return getOdomYaw();
    return std::atan2(diff(1), diff(0));
  }

  void SCANReplanFSM::publishSelfInflationMarker()
  {
    const double radius = std::max(0.0, self_double_cylinder_radius_);
    const double z_up = std::max(0.0, self_inflation_z_up_);
    const double z_down = std::max(0.0, self_inflation_z_down_);
    const double height = std::max(1e-3, z_up + z_down);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = self_inflation_frame_id_.empty() ? "world" : self_inflation_frame_id_;
    marker.header.stamp = node_->now();
    marker.ns = "self_inflation";
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    marker.scale.z = height;
    marker.color.r = 0.1;
    marker.color.g = 0.6;
    marker.color.b = 1.0;
    marker.color.a = 0.4;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    Eigen::Vector3d center = odom_pos_;
    center(2) += 0.5 * (z_up - z_down);

    Eigen::Vector3d heading(std::cos(getOdomYaw()), std::sin(getOdomYaw()), 0.0);
    Eigen::Vector3d front = center + self_double_cylinder_offset_ * heading;
    Eigen::Vector3d rear = center - self_double_cylinder_offset_ * heading;

    marker.id = 0;
    marker.pose.position.x = front(0);
    marker.pose.position.y = front(1);
    marker.pose.position.z = front(2);
    self_inflation_pub_->publish(marker);

    marker.id = 1;
    marker.pose.position.x = rear(0);
    marker.pose.position.y = rear(1);
    marker.pose.position.z = rear(2);
    self_inflation_pub_->publish(marker);
  }

  void SCANReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continuously_called_times_++;
    else
      continuously_called_times_ = 1;

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> SCANReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continuously_called_times_, exec_state_);
  }

  void SCANReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void SCANReplanFSM::execFSMCallback()
  {
    updateLocalTrajTimeFreeze();

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      const rclcpp::Time now = node_->now();
      if (last_replan_attempt_time_.nanoseconds() != 0 &&
          (now - last_replan_attempt_time_).seconds() < replan_retry_interval_)
        break;
      last_replan_attempt_time_ = now;

      setStartStateFromOdomOrCurrentTraj();

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      bool success = callReboundReplan(true, flag_random_poly_init);
      if (success)
      {

        replan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      const rclcpp::Time now = node_->now();
      if (last_replan_attempt_time_.nanoseconds() != 0 &&
          (now - last_replan_attempt_time_).seconds() < replan_retry_interval_)
        break;
      last_replan_attempt_time_ = now;

      if (planFromCurrentTraj())
      {
        replan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        replan_fail_count_++;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = node_->now();
      double t_cur = (time_now - info->start_time_).seconds();
      t_cur = std::min(info->duration_, std::max(0.0, t_cur));

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
      {
        const double xy_error = (end_pt_.head<2>() - odom_pos_.head<2>()).norm();
        const double z_error = std::abs(end_pt_(2) - odom_pos_(2));
        if (xy_error <= goal_tolerance_ && z_error <= goal_z_tolerance_)
        {
          have_target_ = false;
          trigger_ = false;
          publishGoalStatus("GOAL_REACHED");
          changeFSMExecState(WAIT_TARGET, "FSM");
          return;
        }

        if (go2_execution_frozen_)
          return;

        const double traveled_xy =
            (info->start_pos_.head<2>() - odom_pos_.head<2>()).norm();
        const double local_remaining_xy =
            (local_target_pt_.head<2>() - odom_pos_.head<2>()).norm();
        const double local_segment_xy =
            (local_target_pt_.head<2>() - info->start_pos_.head<2>()).norm();
        // Keep the endpoint safeguard consistent with the configured travel
        // threshold. For a 3.0 m horizon and a 2.5 m threshold this leaves a
        // 0.5 m overlap, instead of replanning early at 1.95 m traveled.
        const double replan_margin =
            std::max(0.30, planning_horizon_ - replan_thresh_);
        const double minimum_segment_progress = std::min(0.15, 0.5 * local_segment_xy);
        const bool approaching_local_endpoint =
            local_remaining_xy <= replan_margin &&
            traveled_xy >= minimum_segment_progress;

        if (traveled_xy >= replan_thresh_ || approaching_local_endpoint ||
            t_cur > info->duration_ - 1e-2)
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        return;
      }

      if (isWaypointSequenceMode() &&
          current_wp_ + 1 < (int)active_waypoints_.size() &&
          (end_pt_ - odom_pos_).norm() < 0.5)
      {
        current_wp_++;
        if (planNextWaypoint())
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }
        replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        return;
      }

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2)
      {
        if (isWaypointSequenceMode() && current_wp_ + 1 < (int)active_waypoints_.size())
        {
          current_wp_++;
          if (planNextWaypoint())
          {
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            return;
          }
          replan_fail_count_++;
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }

        if (isWaypointSequenceMode())
        {
          active_waypoints_.clear();
          current_wp_ = 0;
        }

        const double xy_error = (end_pt_.head<2>() - odom_pos_.head<2>()).norm();
        const double z_error = std::abs(end_pt_(2) - odom_pos_(2));
        if (xy_error <= goal_tolerance_ && z_error <= goal_z_tolerance_)
        {
          have_target_ = false;
          trigger_ = false;
          publishGoalStatus("GOAL_REACHED");
          changeFSMExecState(WAIT_TARGET, "FSM");
        }
        else
        {
          RCLCPP_WARN_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 1000,
              "Local trajectory ended but goal error is xy=%.3f z=%.3f; replanning",
              xy_error, z_error);
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
        return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        // cout << "near end" << endl;
        return;
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
      {
        // cout << "near start" << endl;
        return;
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && !need_hover_stop_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        else if (enable_fail_safe_ && need_hover_stop_ && odom_vel_.norm() < 0.1)
        {
          RCLCPP_INFO(node_->get_logger(),
                      "Exiting EMERGENCY_STOP; switching to WAIT_TARGET for a new target");
          need_hover_stop_ = false;
          have_target_ = false;
          trigger_ = false;
          changeFSMExecState(WAIT_TARGET, "EMERGENCY_EXIT");
        }
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    finishProcess();

    data_disp_.header.stamp = node_->now();
    data_disp_pub_->publish(data_disp_);
  }

  void SCANReplanFSM::finishProcess()
  {
    if (replan_fail_count_ >= max_replan_fail_count_)
    {
      RCLCPP_WARN(node_->get_logger(),
                  "Replan failed %d times; emergency stop and wait for a new target", replan_fail_count_);
      replan_fail_count_ = 0;
      need_hover_stop_ = true;
      flag_escape_emergency_ = true;
      publishGoalStatus("GOAL_FAILED");
      changeFSMExecState(EMERGENCY_STOP, "finishProcess");
    }
  }

  bool SCANReplanFSM::planFromCurrentTraj()
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    rclcpp::Time time_now = node_->now();
    double t_cur = (time_now - info->start_time_).seconds();
    t_cur = std::min(std::max(t_cur, 0.0), info->duration_);

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    start_pt_ = odom_pos_;
    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
    {
      start_vel_ = clamp_velocity_norm(odom_vel_, planner_manager_->pp_.max_vel_);
      if ((start_vel_ - odom_vel_).norm() > 1e-6)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "[reference path] Clamp measured replan speed from %.3f to %.3f m/s",
                    odom_vel_.norm(), start_vel_.norm());
      }
      start_acc_.setZero();
    }
    else
    {
      start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);
    }

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }

    // In reference mode global_data_ is the PCT path.  Replacing it here with
    // a start-to-goal polynomial would make every replan forget the PCT route.
    if (navi_mode_ != NAVI_MODE::REFERENCE_PATH &&
      !planner_manager_->planGlobalTraj(
            start_pt_,
            start_vel_,
            start_acc_,
            end_pt_,
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero()))
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "[navi_mode=%d] Unable to refresh global trajectory from odom to current target", navi_mode_);
      return false;
    }

    if (navi_mode_ != NAVI_MODE::REFERENCE_PATH &&
      !adjustGlobalTargetIfOccupied())
      return false;

    bool success = callReboundReplan(true, false);
    if (!success)
    {
      success = callReboundReplan(true, true);
      if (!success)
        return false;
    }

    return true;
  }

  void SCANReplanFSM::setStartStateFromOdomOrCurrentTraj()
  {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    LocalTrajData *info = &planner_manager_->local_data_;
    if (info->start_time_.seconds() < 1e-5 || info->duration_ <= 1e-5)
      return;

    const double raw_t_cur = (node_->now() - info->start_time_).seconds();
    if (raw_t_cur < -1e-3 || raw_t_cur > info->duration_ + 0.2)
      return;

    const double t_cur = std::min(std::max(raw_t_cur, 0.0), info->duration_);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }
  }

  void SCANReplanFSM::checkCollisionCallback()
  {
    updateLocalTrajTimeFreeze();

    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ != EXEC_TRAJ || info->start_time_.seconds() < 1e-5)
      return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = std::max(0.0, (node_->now() - info->start_time_).seconds());
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t);
      Eigen::Vector3d pos_next = info->position_traj_.evaluateDeBoorT(std::min(t + time_step, info->duration_));
      if (map->getInflateOccupancy(pos, estimateYawFromSegment(pos, pos_next)))
      {
        const double time_to_collision = t - t_cur;
        const rclcpp::Time now = node_->now();
        const bool retry_ready = last_replan_attempt_time_.nanoseconds() == 0 ||
            (now - last_replan_attempt_time_).seconds() >= replan_retry_interval_;

        if (retry_ready)
        {
          last_replan_attempt_time_ = now;
          if (planFromCurrentTraj())
          {
            changeFSMExecState(EXEC_TRAJ, "SAFETY");
            return;
          }
        }

        if (time_to_collision < emergency_time_)
        {
          RCLCPP_WARN(node_->get_logger(),
                      "Obstacle discovered; emergency stop in %.3fs",
                      time_to_collision);
          changeFSMExecState(EMERGENCY_STOP, "SAFETY");
        }
        else
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        return;
      }
    }
  }

  bool SCANReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    if (!getLocalTarget())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "No collision-free local target is currently available");
      return false;
    }

    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {

      auto info = &planner_manager_->local_data_;

      /* publish traj */
      scan_planner_msgs::msg::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::msg::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      bspline_pub_->publish(bspline);

      visualization_->displayOptimalTraj(info->position_traj_, 0);
    }

    return plan_success;
  }

  bool SCANReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    scan_planner_msgs::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_->publish(bspline);

    return true;
  }

  bool SCANReplanFSM::getLocalTarget()
  {
    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH &&
        reference_path_points_.size() >= 2 &&
        reference_path_arc_lengths_.size() == reference_path_points_.size())
    {
      const double endpoint_adjust_distance =
          planning_horizon_ + std::max(reference_goal_search_radius_, 0.0);
      if ((end_pt_.head<2>() - odom_pos_.head<2>()).norm() <= endpoint_adjust_distance &&
          !adjustReferenceGoalIfOccupied())
        return false;

      const double total_s = reference_path_arc_lengths_.back();
      auto interpolate_path = [&](double query_s) {
        query_s = std::min(std::max(query_s, 0.0), total_s);
        const auto upper = std::upper_bound(
            reference_path_arc_lengths_.begin(),
            reference_path_arc_lengths_.end(), query_s);
        if (upper == reference_path_arc_lengths_.begin())
          return reference_path_points_.front();
        if (upper == reference_path_arc_lengths_.end())
          return reference_path_points_.back();

        const size_t next = static_cast<size_t>(
            std::distance(reference_path_arc_lengths_.begin(), upper));
        const size_t previous = next - 1;
        const double segment_length =
            reference_path_arc_lengths_[next] - reference_path_arc_lengths_[previous];
        if (segment_length <= 1e-6)
          return reference_path_points_[next];
        const double ratio =
            (query_s - reference_path_arc_lengths_[previous]) / segment_length;
        return (reference_path_points_[previous] +
                ratio * (reference_path_points_[next] -
                         reference_path_points_[previous])).eval();
      };

      double nearest_s = reference_progress_s_;
      double nearest_distance = std::numeric_limits<double>::infinity();
      for (size_t i = 0; i + 1 < reference_path_points_.size(); ++i)
      {
        const Eigen::Vector2d segment =
            reference_path_points_[i + 1].head<2>() -
            reference_path_points_[i].head<2>();
        const double segment_length_sq = segment.squaredNorm();
        if (segment_length_sq <= 1e-10)
          continue;

        double ratio =
            (start_pt_.head<2>() - reference_path_points_[i].head<2>()).dot(segment) /
            segment_length_sq;
        ratio = std::min(std::max(ratio, 0.0), 1.0);
        const double candidate_s = reference_path_arc_lengths_[i] +
            ratio * (reference_path_arc_lengths_[i + 1] -
                     reference_path_arc_lengths_[i]);
        if (candidate_s + 1e-6 < reference_progress_s_)
          continue;

        const Eigen::Vector2d projection =
            reference_path_points_[i].head<2>() + ratio * segment;
        const double distance = (start_pt_.head<2>() - projection).norm();
        if (distance < nearest_distance - 1e-6 ||
            (std::abs(distance - nearest_distance) <= 1e-6 &&
             candidate_s > nearest_s))
        {
          nearest_distance = distance;
          nearest_s = candidate_s;
        }
      }

      reference_progress_s_ = std::max(reference_progress_s_, nearest_s);
      double target_s = std::min(total_s, reference_progress_s_ + planning_horizon_);
      local_target_pt_ = interpolate_path(target_s);
      bool target_adjusted_for_obstacle = false;

      auto path_occupancy = [&](double query_s) {
        const Eigen::Vector3d pt = interpolate_path(query_s);
        const Eigen::Vector3d previous = interpolate_path(std::max(0.0, query_s - 0.05));
        const Eigen::Vector3d next = interpolate_path(std::min(total_s, query_s + 0.05));
        return planner_manager_->grid_map_->getInflateOccupancy(
            pt, estimateYawFromSegment(previous, next));
      };

      const double terminal_support_length =
          std::max(0.3, 2.0 * planner_manager_->pp_.ctrl_pt_dist);
      auto has_free_support = [&](double candidate_s, bool forward) {
        const double support_end = forward
            ? std::min(total_s, candidate_s + terminal_support_length)
            : std::max(reference_progress_s_, candidate_s - terminal_support_length);
        const double direction = forward ? 1.0 : -1.0;
        for (double check_s = candidate_s;
             forward ? check_s <= support_end + 1e-6
                     : check_s >= support_end - 1e-6;
             check_s += direction * 0.05)
        {
          if (path_occupancy(check_s) != 0)
            return false;
        }
        return true;
      };

      if (path_occupancy(target_s) != 0)
      {
        bool found_free_target = false;
        constexpr double search_step = 0.1;
        const double occupied_target_s = target_s;

        for (double forward_s = occupied_target_s + search_step;
             forward_s <= total_s + 1e-6; forward_s += search_step)
        {
          if (has_free_support(forward_s, true))
          {
            local_target_pt_ = interpolate_path(forward_s);
            target_s = forward_s;
            found_free_target = true;
            target_adjusted_for_obstacle = true;
            break;
          }
        }

        if (!found_free_target)
        {
          const double minimum_local_target_distance =
              std::max(0.30, 1.5 * planner_manager_->pp_.ctrl_pt_dist);
          for (double backward_s = occupied_target_s - search_step;
               backward_s >= reference_progress_s_ - 1e-6;
               backward_s -= search_step)
          {
            const Eigen::Vector3d backward_target = interpolate_path(backward_s);
            if ((backward_target.head<2>() - start_pt_.head<2>()).norm() <
                minimum_local_target_distance)
              continue;
            if (has_free_support(backward_s, false))
            {
              local_target_pt_ = backward_target;
              target_s = backward_s;
              found_free_target = true;
              target_adjusted_for_obstacle = true;
              break;
            }
          }
        }

        if (!found_free_target)
        {
          RCLCPP_WARN_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 1000,
              "Reference-path local target is occupied and no safe replacement was found");
          return false;
        }
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "Reference target %.2fm is occupied; using safe target %.2fm",
            occupied_target_s, target_s);
      }

      if (target_s >= total_s - 1e-3 || target_adjusted_for_obstacle)
      {
        if (target_s >= total_s - 1e-3)
          local_target_pt_ = end_pt_;
        local_target_vel_.setZero();
      }
      else
      {
        const Eigen::Vector3d next_pt =
            interpolate_path(std::min(total_s, target_s + 0.1));
        const Eigen::Vector3d direction = next_pt - local_target_pt_;
        if (direction.norm() > 1e-6)
          local_target_vel_ =
              direction.normalized() * planner_manager_->pp_.max_vel_;
        else
          local_target_vel_.setZero();
      }

      RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "[reference path] progress=%.2f/%.2fm error=%.2fm target=%.2fm",
          reference_progress_s_, total_s, nearest_distance, target_s);
      return true;
    }

    double t;

    const double t_step = planning_horizon_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;
    double target_t = planner_manager_->global_data_.global_duration_;
    local_target_pt_ = end_pt_;
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizon_)
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "Local target progress mismatch: distance=%.3f horizon=%.3f progress_time=%.3f",
                     dist, planning_horizon_, planner_manager_->global_data_.last_progress_time_);
        local_target_pt_ = pos_t;
        target_t = t;
        planner_manager_->global_data_.last_progress_time_ = t;
        break;
      }
      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }
      if (dist >= planning_horizon_)
      {
        local_target_pt_ = pos_t;
        target_t = t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }
    if (t >= planner_manager_->global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
      target_t = planner_manager_->global_data_.global_duration_;
    }

    auto targetOccupancy = [&](const Eigen::Vector3d &pt) {
      return planner_manager_->grid_map_->getInflateOccupancy(pt, estimateYawFromSegment(odom_pos_, pt));
    };

    if (targetOccupancy(local_target_pt_) != 0)
    {
      bool found_free_target = false;
      double adjusted_t = target_t;

      for (double dt = 0.0; dt <= planner_manager_->global_data_.global_duration_; dt += t_step)
      {
        double t_forward = target_t + dt;
        if (t_forward <= planner_manager_->global_data_.global_duration_)
        {
          Eigen::Vector3d pt = planner_manager_->global_data_.getPosition(t_forward);
          if (targetOccupancy(pt) == 0)
          {
            local_target_pt_ = pt;
            adjusted_t = t_forward;
            found_free_target = true;
            break;
          }
        }

        double t_backward = target_t - dt;
        if (t_backward >= std::max(0.0, dist_min_t))
        {
          Eigen::Vector3d pt = planner_manager_->global_data_.getPosition(t_backward);
          if (targetOccupancy(pt) == 0)
          {
            local_target_pt_ = pt;
            adjusted_t = t_backward;
            found_free_target = true;
            break;
          }
        }
      }

      if (found_free_target)
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Local target was adjusted to a nearby collision-free point");
        target_t = adjusted_t;
      }
      else
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Local target is in collision and no nearby free target was found");
        return false;
      }
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(target_t);
      // cout << "AA" << endl;
    }
    return true;
  }

} // namespace scan_planner
