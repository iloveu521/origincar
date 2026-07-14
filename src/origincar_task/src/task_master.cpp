#include "origincar_task/task_master.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "origincar_task/mission_policy.hpp"
#include "tf2/time.h"

namespace origincar_task {
namespace {

double GetOptionalDouble(const YAML::Node &node, const char *key,
                         double default_value) {
  return node[key] ? node[key].as<double>() : default_value;
}

double Clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

double NormalizeAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

double GetOptionalRadius(const YAML::Node &node, const char *key) {
  return node[key] ? node[key].as<double>() : 0.0;
}

std::vector<Waypoint> LoadRoute(const YAML::Node &routes, const char *name) {
  std::vector<Waypoint> path;
  if (!routes || !routes[name]) {
    return path;
  }
  for (const auto &pt : routes[name]) {
    Waypoint wp;
    wp.x = pt["x"].as<double>();
    wp.y = pt["y"].as<double>();
    wp.yaw = GetOptionalDouble(pt, "yaw", 0.0);
    wp.pause_sec = GetOptionalDouble(pt, "pause", 0.0);

    // Phase 3: motion field
    if (!pt["motion"]) {
      throw std::runtime_error("Missing required motion field in route '" +
                               std::string(name) + "'.");
    }
    const auto motion = pt["motion"].as<std::string>();
    if (motion != "forward" && motion != "reverse") {
      throw std::runtime_error("Invalid motion field '" + motion +
                               "' in route '" + name +
                               "'. Must be 'forward' or 'reverse'.");
    }
    wp.motion = motion;
    wp.reverse = (motion == "reverse");

    wp.pass_radius = GetOptionalRadius(pt, "pass_radius");
    wp.reverse_pass_radius = GetOptionalRadius(pt, "reverse_pass_radius");

    // Phase 3: per-waypoint gains
    wp.speed_gain = GetOptionalDouble(pt, "speed_gain", 1.0);
    wp.angular_gain = GetOptionalDouble(pt, "angular_gain", 1.0);

    // Phase 3: behavior flags
    wp.capture = pt["capture"] ? pt["capture"].as<bool>() : false;
    wp.qr_scan = pt["qr_scan"] ? pt["qr_scan"].as<bool>() : false;
    wp.qr_deadline = pt["qr_deadline"] ? pt["qr_deadline"].as<bool>() : false;

    path.push_back(wp);
  }
  return path;
}

} // namespace

// ============================================================================
// Constructor
// ============================================================================

TaskMaster::TaskMaster()
    : Node("task_master"), qr_decision_state_(QrDecisionState::PENDING),
      qr_direction_(0), robot_x_(0.0), robot_y_(0.0), robot_yaw_(0.0),
      current_state_(TaskState::WAIT_TF), nav_status_(NavStatus::IDLE),
      custom_rpp_index_(0), custom_rpp_active_speed_(0.0),
      obstacle_warning_count_(0), motion_switch_pending_(false),
      paused_waypoint_index_(std::numeric_limits<size_t>::max()),
      pause_until_time_sec_(std::numeric_limits<double>::quiet_NaN()),
      obstacle_state_(ObstacleState::CLEAR), avoid_direction_locked_(0.0),
      backup_start_x_(0.0), backup_start_y_(0.0), backup_executed_(false),
      obstacle_clear_since_sec_(std::numeric_limits<double>::quiet_NaN()),
      backup_start_time_sec_(std::numeric_limits<double>::quiet_NaN()),
      startup_tf_stable_count_(0), has_last_custom_cmd_(false),
      last_valid_pose_time_(std::numeric_limits<double>::quiet_NaN()) {
  // ── Unified speed params (Phase 6) ──
  cruise_speed_ = this->declare_parameter<double>("cruise_speed", 0.85);
  qr_scan_speed_gain_ =
      this->declare_parameter<double>("qr_scan_speed_gain", 0.85);
  turn_speed_gain_ = this->declare_parameter<double>("turn_speed_gain", 0.50);
  turn_angular_gain_ =
      this->declare_parameter<double>("turn_angular_gain", 1.50);
  capture_speed_gain_ =
      this->declare_parameter<double>("capture_speed_gain", 0.60);
  capture_radius_ = this->declare_parameter<double>("capture_radius", 0.25);
  default_qr_direction_ =
      this->declare_parameter<std::string>("default_qr_direction", "cw");

  // ── Obstacle fused params (Phase 9) ──
  cone_confidence_threshold_ =
      this->declare_parameter<double>("cone_confidence_threshold", 0.35);
  obstacle_backup_speed_gain_ =
      this->declare_parameter<double>("obstacle_backup_speed_gain", 0.70);
  obstacle_backup_distance_ =
      this->declare_parameter<double>("obstacle_backup_distance", 0.40);
  obstacle_backup_timeout_sec_ =
      this->declare_parameter<double>("obstacle_backup_timeout_sec", 1.50);
  rear_clearance_distance_ =
      this->declare_parameter<double>("rear_clearance_distance", 0.25);
  cone_detection_timeout_sec_ =
      this->declare_parameter<double>("cone_detection_timeout_sec", 0.25);
  obstacle_clear_hold_sec_ =
      this->declare_parameter<double>("obstacle_clear_hold_sec", 0.40);
  avoid_proximity_max_angular_z_ =
      this->declare_parameter<double>("avoid_proximity_max_angular_z", 1.10);
  obstacle_backup_distance_threshold_ = this->declare_parameter<double>(
      "obstacle_backup_distance_threshold", 0.20);
  obstacle_slow_distance_ =
      this->declare_parameter<double>("obstacle_slow_distance", 0.45);
  obstacle_avoid_distance_ =
      this->declare_parameter<double>("obstacle_avoid_distance", 0.45);
  cone_image_width_ =
      this->declare_parameter<double>("cone_image_width", 640.0);

  // ── RPP params ──
  custom_rpp_lookahead_dist_ =
      this->declare_parameter<double>("lookahead_dist", 0.25);
  custom_rpp_pass_radius_ =
      this->declare_parameter<double>("pass_radius", 0.35);
  custom_rpp_reverse_pass_radius_ =
      this->declare_parameter<double>("reverse_pass_radius", 0.30);
  custom_rpp_goal_tolerance_ =
      this->declare_parameter<double>("goal_tolerance", 0.25);
  custom_rpp_max_angular_z_ =
      this->declare_parameter<double>("max_angular_z", 1.8);
  custom_rpp_turn_gain_ =
      this->declare_parameter<double>("path_turn_gain", 1.40);
  custom_rpp_enable_lookahead_progress_ =
      this->declare_parameter<bool>("enable_lookahead_progress", true);
  custom_rpp_enable_obstacle_avoidance_ =
      this->declare_parameter<bool>("enable_obstacle_avoidance", true);
  custom_rpp_obstacle_enable_from_waypoint_ =
      this->declare_parameter<int>("obstacle_enable_from_waypoint", 0);
  custom_rpp_min_obstacle_range_ =
      this->declare_parameter<double>("min_obstacle_range", 0.05);
  custom_rpp_front_angle_ =
      this->declare_parameter<double>("front_angle_deg", 30.0) * M_PI / 180.0;
  custom_rpp_side_angle_ =
      this->declare_parameter<double>("side_angle_deg", 90.0) * M_PI / 180.0;
  custom_rpp_backup_speed_ = cruise_speed_ * obstacle_backup_speed_gain_;

  // ── WAIT_TF params ──
  startup_tf_required_frames_ =
      this->declare_parameter<int>("startup_tf_required_frames", 5);
  startup_tf_check_timeout_sec_ =
      this->declare_parameter<double>("startup_tf_check_timeout_sec", 10.0);
  startup_tf_max_age_sec_ =
      this->declare_parameter<double>("startup_tf_max_age_sec", 0.3);

  // ── TF-loss hold ──
  custom_rpp_pose_hold_sec_ =
      this->declare_parameter<double>("custom_rpp_pose_hold_sec", 0.5);

  // ── Scan freshness ──
  custom_rpp_scan_timeout_sec_ =
      this->declare_parameter<double>("scan_timeout_sec", 0.30);

  // ── Reacquire ──
  custom_rpp_reacquire_window_ =
      this->declare_parameter<int>("reacquire_window", 20);

  const auto valid_gain = [](double value) {
    return std::isfinite(value) && value > 0.0 && value <= 1.0;
  };
  if (!std::isfinite(cruise_speed_) || cruise_speed_ <= 0.0 ||
      !valid_gain(qr_scan_speed_gain_) || !valid_gain(turn_speed_gain_) ||
      !valid_gain(capture_speed_gain_) ||
      !valid_gain(obstacle_backup_speed_gain_) || turn_angular_gain_ <= 0.0 ||
      custom_rpp_max_angular_z_ <= 0.0 ||
      avoid_proximity_max_angular_z_ <= 0.0 ||
      cone_confidence_threshold_ < 0.0 || cone_confidence_threshold_ > 1.0 ||
      cone_image_width_ <= 0.0 || obstacle_backup_distance_threshold_ <= 0.0 ||
      obstacle_avoid_distance_ < obstacle_backup_distance_threshold_ ||
      obstacle_slow_distance_ < obstacle_backup_distance_threshold_ ||
      obstacle_slow_distance_ < obstacle_avoid_distance_ ||
      obstacle_backup_distance_ <= 0.0 || obstacle_backup_timeout_sec_ <= 0.0 ||
      rear_clearance_distance_ <= 0.0 || capture_radius_ <= 0.0 ||
      custom_rpp_scan_timeout_sec_ <= 0.0 ||
      custom_rpp_reacquire_window_ <= 0 || custom_rpp_lookahead_dist_ <= 0.0 ||
      custom_rpp_pass_radius_ <= 0.0 ||
      custom_rpp_reverse_pass_radius_ <= 0.0 ||
      custom_rpp_goal_tolerance_ <= 0.0 || startup_tf_required_frames_ <= 0 ||
      startup_tf_check_timeout_sec_ <= 0.0 || startup_tf_max_age_sec_ <= 0.0) {
    throw std::invalid_argument("Invalid TaskMaster competition parameters");
  }
  std::string default_direction = default_qr_direction_;
  std::transform(default_direction.begin(), default_direction.end(),
                 default_direction.begin(), [](unsigned char c) {
                   return static_cast<char>(std::tolower(c));
                 });
  if (default_direction != "cw" && default_direction != "clockwise" &&
      default_direction != "ccw" && default_direction != "counterclockwise" &&
      default_qr_direction_ != "顺时针" && default_qr_direction_ != "逆时针") {
    throw std::invalid_argument("default_qr_direction must be cw or ccw");
  }

  // ── Subscriptions ──
  const auto qr_direction_topic = this->declare_parameter<std::string>(
      "qr_direction_topic", "/qr_direction");
  qr_result_sub_ = this->create_subscription<std_msgs::msg::String>(
      qr_direction_topic, 10,
      std::bind(&TaskMaster::qr_result_callback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom_combined", rclcpp::SensorDataQoS(),
      std::bind(&TaskMaster::odom_callback, this, std::placeholders::_1));
  const auto scan_topic =
      this->declare_parameter<std::string>("scan_topic", "/scan");
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
        const std::lock_guard<std::mutex> lock(state_mutex_);
        latest_scan_ = msg;
        latest_scan_time_ = this->now();
      });
  const auto cone_topic = this->declare_parameter<std::string>(
      "cone_detection_topic", "/racing_obstacle_detection");
  cone_sub_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>(
      cone_topic, rclcpp::SensorDataQoS(),
      std::bind(&TaskMaster::cone_result_callback, this,
                std::placeholders::_1));

  // ── Publishers ──
  const auto announcement_topic = this->declare_parameter<std::string>(
      "announcement_topic", "/announcement");
  announcement_pub_ =
      this->create_publisher<std_msgs::msg::String>(announcement_topic, 10);
  capture_trigger_pub_ =
      this->create_publisher<std_msgs::msg::Empty>("/capture_trigger", 10);
  cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // ── TF ──
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  load_waypoints();

  // ── Timers ──
  // FSM: 10 Hz — state transitions, goal dispatch
  fsm_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(100),
                              std::bind(&TaskMaster::state_machine_loop, this));

  // Controller: 20 Hz — pure pursuit + obstacle avoidance
  const double control_frequency =
      this->declare_parameter<double>("control_frequency", 20.0);
  if (!std::isfinite(control_frequency) || control_frequency <= 0.0) {
    throw std::invalid_argument("control_frequency must be positive");
  }
  control_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(
                                  1000.0 / std::max(control_frequency, 1.0))),
                              std::bind(&TaskMaster::control_loop, this));

  state_start_time_ = this->now();
  RCLCPP_INFO(this->get_logger(),
              "TaskMaster initialized. FSM=10Hz, Controller=%.0fHz. "
              "State: WAIT_TF (need %d consecutive TF frames)",
              control_frequency, startup_tf_required_frames_);
}

// ============================================================================
// Waypoint Loading
// ============================================================================

void TaskMaster::load_waypoints() {
  try {
    std::string config_path =
        this->declare_parameter<std::string>("waypoints_file", "");
    if (config_path.empty()) {
      config_path =
          ament_index_cpp::get_package_share_directory("origincar_task") +
          "/config/waypoints_flowpath_custom_rpp.yaml";
    }

    const YAML::Node config = YAML::LoadFile(config_path);
    const auto wp = config["waypoints"];

    // P point (start/park)
    wp_p_ = {wp["point_P"]["x"].as<double>(),
             wp["point_P"]["y"].as<double>(),
             GetOptionalDouble(wp["point_P"], "yaw", 0.0),
             0.0,
             "forward",
             false,
             0.35,
             0.30,
             1.0,
             1.0,
             false,
             false,
             false};

    // Phase 4: P→QR→B merged route
    const auto routes = config["routes"];
    route_p_to_qr_b_ = LoadRoute(routes, "p_to_qr_to_b_rpp");
    if (route_p_to_qr_b_.empty()) {
      throw std::runtime_error(
          "Required route 'p_to_qr_to_b_rpp' is missing or empty");
    }

    // C zone ring paths
    ring_ccw_ = LoadRoute(routes, "ring_ccw");
    ring_cw_ = LoadRoute(routes, "ring_cw");
    if (ring_cw_.empty() || ring_ccw_.empty()) {
      throw std::runtime_error("Both ring_cw and ring_ccw are required");
    }

    const auto validate_path = [](const std::vector<Waypoint> &path,
                                  const char *name) {
      for (size_t i = 0; i < path.size(); ++i) {
        const auto &point = path[i];
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.yaw) || point.speed_gain <= 0.0 ||
            point.speed_gain > 1.0 || point.angular_gain <= 0.0) {
          throw std::runtime_error(std::string("Invalid waypoint in ") + name);
        }
        if (i > 0) {
          const double gap =
              std::hypot(point.x - path[i - 1].x, point.y - path[i - 1].y);
          if (gap <= 0.0 || gap > 0.30) {
            throw std::runtime_error(std::string("Invalid waypoint gap in ") +
                                     name + ": " + std::to_string(gap));
          }
        }
      }
    };
    validate_path(route_p_to_qr_b_, "p_to_qr_to_b_rpp");
    validate_path(ring_cw_, "ring_cw");
    validate_path(ring_ccw_, "ring_ccw");
    constexpr double kJoinTolerance = 1e-6;
    const auto &join = route_p_to_qr_b_.back();
    if (std::any_of(route_p_to_qr_b_.begin(), route_p_to_qr_b_.end(),
                    [](const Waypoint &point) { return point.reverse; })) {
      throw std::runtime_error("P→QR→B route must be entirely forward");
    }
    if (std::hypot(join.x - ring_cw_.front().x, join.y - ring_cw_.front().y) >
            kJoinTolerance ||
        std::hypot(join.x - ring_ccw_.front().x, join.y - ring_ccw_.front().y) >
            kJoinTolerance) {
      throw std::runtime_error(
          "P→QR→B endpoint must equal both ring entry points");
    }

    route_return_to_p_ = LoadRoute(routes, "return_to_p_rpp");
    if (route_return_to_p_.empty()) {
      route_return_to_p_ = {wp_p_};
    }

    const auto apply_turn_policy = [this](std::vector<Waypoint> &path) {
      for (auto &point : path) {
        if (point.angular_gain > 1.0) {
          point.speed_gain = std::min(point.speed_gain, turn_speed_gain_);
          point.angular_gain = std::max(point.angular_gain, turn_angular_gain_);
        }
      }
    };
    apply_turn_policy(route_p_to_qr_b_);
    apply_turn_policy(ring_cw_);
    apply_turn_policy(ring_ccw_);
    apply_turn_policy(route_return_to_p_);

    RCLCPP_INFO(this->get_logger(),
                "Waypoints loaded: P(%.2f, %.2f), "
                "p_to_qr_to_b=%zu, ring_cw=%zu, ring_ccw=%zu, "
                "return_to_p=%zu",
                wp_p_.x, wp_p_.y, route_p_to_qr_b_.size(), ring_cw_.size(),
                ring_ccw_.size(), route_return_to_p_.size());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load waypoints: %s", e.what());
    nav_status_ = NavStatus::FAILED;
    current_state_ = TaskState::PARK;
  }
}

// ============================================================================
// FSM (10 Hz)
// ============================================================================

void TaskMaster::state_machine_loop() {
  const std::lock_guard<std::mutex> lock(state_mutex_);
  switch (current_state_) {
  case TaskState::WAIT_TF:
    handle_wait_tf();
    break;
  case TaskState::IDLE:
    handle_idle();
    break;
  case TaskState::NAV_P_QR_B:
    handle_nav_p_qr_b();
    break;
  case TaskState::RING_NAV:
    handle_ring_nav();
    break;
  case TaskState::NAV_TO_P:
    handle_nav_to_p();
    break;
  case TaskState::PARK:
    handle_park();
    break;
  case TaskState::DONE:
    break;
  }
}

void TaskMaster::transition_to(TaskState new_state) {
  RCLCPP_INFO(this->get_logger(), "Transition: %d -> %d",
              static_cast<int>(current_state_), static_cast<int>(new_state));
  current_state_ = new_state;
  state_start_time_ = this->now();
}

void TaskMaster::handle_wait_tf() {
  stop_robot();

  const double elapsed = (this->now() - state_start_time_).seconds();
  if (elapsed > startup_tf_check_timeout_sec_) {
    RCLCPP_ERROR(this->get_logger(),
                 "WAIT_TF timeout after %.1fs (stable_count=%d/%d), "
                 "remaining stopped until TF is stable",
                 elapsed, startup_tf_stable_count_,
                 startup_tf_required_frames_);
    state_start_time_ = this->now();
    return;
  }

  if (is_map_tf_stable_once()) {
    ++startup_tf_stable_count_;
    RCLCPP_INFO(this->get_logger(), "WAIT_TF stable count: %d/%d",
                startup_tf_stable_count_, startup_tf_required_frames_);
  } else {
    if (startup_tf_stable_count_ > 0) {
      RCLCPP_WARN(this->get_logger(),
                  "WAIT_TF lost stability, resetting count (was %d)",
                  startup_tf_stable_count_);
    }
    startup_tf_stable_count_ = 0;
  }

  if (startup_tf_stable_count_ >= startup_tf_required_frames_) {
    RCLCPP_INFO(
        this->get_logger(),
        "WAIT_TF stable for %d consecutive frames, transitioning to IDLE",
        startup_tf_stable_count_);
    transition_to(TaskState::IDLE);
  }
}

bool TaskMaster::is_map_tf_stable_once() {
  try {
    const auto transform = tf_buffer_->lookupTransform("map", "base_footprint",
                                                       tf2::TimePointZero);
    const double age = (this->now() - transform.header.stamp).seconds();
    if (age > startup_tf_max_age_sec_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "map->base_footprint TF too old: %.2fs > %.2fs", age,
                           startup_tf_max_age_sec_);
      return false;
    }
    return true;
  } catch (const std::exception &e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "map->base_footprint TF not yet available: %s",
                         e.what());
    return false;
  }
}

void TaskMaster::handle_idle() {
  if ((this->now() - state_start_time_).seconds() > 2.0) {
    announce("任务开始");
    transition_to(TaskState::NAV_P_QR_B);
    start_custom_rpp(build_p_to_qr_b_path(), cruise_speed_, "NAV_P_QR_B");
  }
}

void TaskMaster::handle_nav_p_qr_b() {
  if (navigation_succeeded()) {
    lock_default_qr_direction("route completed without a QR decision");
    // Seamless transition to ring nav — no zero velocity
    const auto &ring = (qr_direction_ == 2) ? ring_ccw_ : ring_cw_;
    const std::string dir_str = (qr_direction_ == 1) ? "顺时针" : "逆时针";
    announce("QR扫描完成，" + dir_str);
    transition_to(TaskState::RING_NAV);
    start_custom_rpp(ring, cruise_speed_, "RING_NAV");
    return;
  }

  if (navigation_failed()) {
    RCLCPP_ERROR(this->get_logger(), "NAV_P_QR_B custom RPP failed, parking");
    transition_to(TaskState::PARK);
  }
}

void TaskMaster::handle_ring_nav() {
  if (navigation_succeeded()) {
    transition_to(TaskState::NAV_TO_P);
    start_custom_rpp(trim_path_from_robot_pose(route_return_to_p_),
                     cruise_speed_, "NAV_TO_P");
    return;
  }

  if (navigation_failed()) {
    RCLCPP_ERROR(this->get_logger(), "RING_NAV custom RPP failed, parking");
    transition_to(TaskState::PARK);
  }
}

void TaskMaster::handle_nav_to_p() {
  if (navigation_succeeded()) {
    transition_to(TaskState::PARK);
    return;
  }

  if (navigation_failed()) {
    RCLCPP_ERROR(this->get_logger(), "NAV_TO_P custom RPP failed, parking");
    transition_to(TaskState::PARK);
  }
}

void TaskMaster::handle_park() {
  stop_robot();
  if ((this->now() - state_start_time_).seconds() > PARK_STOP_SEC) {
    announce("任务完成");
    transition_to(TaskState::DONE);
  }
}

// ============================================================================
// Control Loop (20 Hz) — runs independently of FSM
// ============================================================================

void TaskMaster::control_loop() {
  const std::lock_guard<std::mutex> lock(state_mutex_);
  if (nav_status_ != NavStatus::RUNNING) {
    if (current_state_ == TaskState::NAV_P_QR_B &&
        nav_status_ == NavStatus::SUCCEEDED && has_last_custom_cmd_) {
      cmd_vel_pub_->publish(last_custom_cmd_);
    }
    return;
  }

  // Per-phase timeout: if the robot cannot reach the final waypoint,
  // force FAILED so the FSM can park instead of looping forever.
  const double phase_elapsed = (this->now() - nav_start_time_).seconds();
  constexpr double kPhaseTimeoutSec = 90.0;
  if (phase_elapsed > kPhaseTimeoutSec) {
    RCLCPP_ERROR(this->get_logger(),
                 "Custom RPP %s timed out after %.1fs, forcing FAILED",
                 active_custom_phase_.c_str(), phase_elapsed);
    stop_robot();
    nav_status_ = NavStatus::FAILED;
    return;
  }

  update_custom_rpp();

  // Capture trigger (checked in control loop for responsiveness)
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  if (get_robot_pose(x, y, yaw)) {
    trigger_custom_capture_if_needed(x, y);
  }
}

// ============================================================================
// Navigation Status Helpers
// ============================================================================

bool TaskMaster::navigation_succeeded() const {
  return nav_status_ == NavStatus::SUCCEEDED;
}

bool TaskMaster::navigation_failed() {
  return nav_status_ == NavStatus::FAILED;
}

// ============================================================================
// Path Builders
// ============================================================================

std::vector<Waypoint> TaskMaster::build_p_to_qr_b_path() {
  return trim_path_from_robot_pose(route_p_to_qr_b_);
}

// ============================================================================
// Custom RPP — Lifecycle
// ============================================================================

bool TaskMaster::start_custom_rpp(const std::vector<Waypoint> &path,
                                  double speed, const char *phase_name) {
  if (path.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Custom RPP %s path is empty", phase_name);
    nav_status_ = NavStatus::FAILED;
    return false;
  }

  active_custom_path_ = path;
  custom_capture_triggered_.assign(active_custom_path_.size(), false);
  custom_pause_completed_.assign(active_custom_path_.size(), false);
  active_custom_phase_ = phase_name;
  custom_rpp_index_ = 0;
  custom_rpp_active_speed_ = speed;
  nav_status_ = NavStatus::RUNNING;
  nav_start_time_ = this->now();
  has_last_custom_cmd_ = false;
  last_valid_pose_time_ = std::numeric_limits<double>::quiet_NaN();
  motion_switch_pending_ = false;
  paused_waypoint_index_ = std::numeric_limits<size_t>::max();
  pause_until_time_sec_ = std::numeric_limits<double>::quiet_NaN();
  obstacle_state_ = ObstacleState::CLEAR;
  avoid_direction_locked_ = 0.0;
  backup_executed_ = false;
  obstacle_clear_since_sec_ = std::numeric_limits<double>::quiet_NaN();
  backup_start_time_sec_ = std::numeric_limits<double>::quiet_NaN();

  RCLCPP_INFO(this->get_logger(),
              "Starting custom RPP %s: %zu waypoints, speed=%.2f",
              active_custom_phase_.c_str(), active_custom_path_.size(),
              custom_rpp_active_speed_);
  return true;
}

// ============================================================================
// Custom RPP — Main Update (called by control_loop at 20 Hz)
// ============================================================================

void TaskMaster::update_custom_rpp() {
  if (active_custom_path_.empty()) {
    return;
  }

  double robot_x = 0.0;
  double robot_y = 0.0;
  double robot_yaw = 0.0;
  if (!get_robot_pose_map(robot_x, robot_y, robot_yaw)) {
    if (obstacle_state_ == ObstacleState::BACKUP_ONCE) {
      stop_robot();
      has_last_custom_cmd_ = false;
      return;
    }
    // TF 短暂丢失：保持上一条 cmd
    const double now = this->now().seconds();
    if (has_last_custom_cmd_ && std::isfinite(last_valid_pose_time_) &&
        (now - last_valid_pose_time_) < custom_rpp_pose_hold_sec_) {
      cmd_vel_pub_->publish(last_custom_cmd_);
    } else {
      stop_robot();
    }
    return;
  }
  last_valid_pose_time_ = this->now().seconds();

  if (handle_custom_pause()) {
    return;
  }

  const bool freeze_progress = obstacle_state_ == ObstacleState::BACKUP_ONCE ||
                               obstacle_state_ == ObstacleState::WAIT_CLEAR;
  if (!freeze_progress) {
    // Reacquire: jump to nearest remaining point if we drifted.
    reacquire_custom_progress(robot_x, robot_y);
    advance_custom_waypoints(robot_x, robot_y);
  }
  if (motion_switch_pending_) {
    stop_robot();
    has_last_custom_cmd_ = false;
    motion_switch_pending_ = false;
    return;
  }

  // Check completion
  const auto &final = active_custom_path_.back();
  const double final_distance =
      std::hypot(final.x - robot_x, final.y - robot_y);
  if (obstacle_state_ == ObstacleState::CLEAR &&
      custom_rpp_index_ >= active_custom_path_.size() - 1 &&
      final_distance <= custom_rpp_goal_tolerance_) {
    RCLCPP_INFO(this->get_logger(), "Custom RPP %s complete",
                active_custom_phase_.c_str());
    // NAV_P_QR_B → RING_NAV 无缝衔接，不停车
    const bool seamless_ring_transition =
        current_state_ == TaskState::NAV_P_QR_B;
    if (!seamless_ring_transition) {
      stop_robot();
    }
    active_custom_path_.clear();
    if (!seamless_ring_transition) {
      has_last_custom_cmd_ = false;
    }
    nav_status_ = NavStatus::SUCCEEDED;
    return;
  }

  // Compute control command
  const size_t target_index = select_custom_target(robot_x, robot_y);
  commit_custom_lookahead_progress(target_index);
  const auto &target = active_custom_path_[target_index];
  const bool capture_zone = is_custom_capture_zone(robot_x, robot_y);
  const double active_speed = active_custom_speed(target, capture_zone);

  const double dx = target.x - robot_x;
  const double dy = target.y - robot_y;
  const double cos_yaw = std::cos(robot_yaw);
  const double sin_yaw = std::sin(robot_yaw);
  const double target_x_base = cos_yaw * dx + sin_yaw * dy;
  const double target_y_base = -sin_yaw * dx + cos_yaw * dy;
  const double distance =
      std::max(std::hypot(target_x_base, target_y_base), 0.001);

  const double curvature = 2.0 * target_y_base / (distance * distance);
  geometry_msgs::msg::Twist cmd;

  if (target.reverse) {
    if (target_x_base > 0.0) {
      stop_robot();
      has_last_custom_cmd_ = false;
      return;
    }
    cmd.linear.x = -active_speed;
    cmd.angular.z = Clamp(
        -cmd.linear.x * curvature * custom_rpp_turn_gain_ * target.angular_gain,
        -custom_rpp_max_angular_z_, custom_rpp_max_angular_z_);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                         "RPP REVERSE phase=%s idx=%zu/%zu reverse=1 "
                         "cmd.linear.x=%.3f cmd.angular.z=%.3f "
                         "target_x_base=%.3f target_y_base=%.3f dist=%.3f",
                         active_custom_phase_.c_str(), target_index + 1,
                         active_custom_path_.size(), cmd.linear.x,
                         cmd.angular.z, target_x_base, target_y_base, distance);
  } else {
    if (target_x_base < 0.0) {
      stop_robot();
      has_last_custom_cmd_ = false;
      return;
    }
    cmd.linear.x = active_speed;
    cmd.angular.z = Clamp(
        active_speed * curvature * custom_rpp_turn_gain_ * target.angular_gain,
        -custom_rpp_max_angular_z_, custom_rpp_max_angular_z_);
  }

  cmd = apply_custom_obstacle_avoidance(cmd, target.reverse, robot_x, robot_y);
  last_custom_cmd_ = cmd;
  has_last_custom_cmd_ = true;
  cmd_vel_pub_->publish(cmd);
}

// ============================================================================
// Custom RPP — Waypoint Advancement
// ============================================================================

void TaskMaster::advance_custom_waypoints(double robot_x, double robot_y) {
  while (custom_rpp_index_ < active_custom_path_.size() - 1) {
    const auto &waypoint = active_custom_path_[custom_rpp_index_];
    const double distance =
        std::hypot(waypoint.x - robot_x, waypoint.y - robot_y);
    if (distance > custom_pass_radius(waypoint, custom_rpp_index_)) {
      break;
    }
    if (waypoint.qr_deadline) {
      lock_default_qr_direction("QR deadline reached");
    }
    if (waypoint.pause_sec > 0.0 &&
        custom_rpp_index_ < custom_pause_completed_.size() &&
        !custom_pause_completed_[custom_rpp_index_]) {
      paused_waypoint_index_ = custom_rpp_index_;
      pause_until_time_sec_ = this->now().seconds() + waypoint.pause_sec;
      stop_robot();
      return;
    }
    RCLCPP_INFO(this->get_logger(),
                "Custom RPP passed waypoint %zu/%zu at distance %.2f",
                custom_rpp_index_ + 1, active_custom_path_.size(), distance);
    const bool motion_changes =
        active_custom_path_[custom_rpp_index_ + 1].reverse != waypoint.reverse;
    ++custom_rpp_index_;
    if (motion_changes) {
      motion_switch_pending_ = true;
      return;
    }
  }
}

bool TaskMaster::handle_custom_pause() {
  if (!std::isfinite(pause_until_time_sec_)) {
    return false;
  }
  if (this->now().seconds() < pause_until_time_sec_) {
    stop_robot();
    return true;
  }
  if (paused_waypoint_index_ < custom_pause_completed_.size()) {
    custom_pause_completed_[paused_waypoint_index_] = true;
  }
  paused_waypoint_index_ = std::numeric_limits<size_t>::max();
  pause_until_time_sec_ = std::numeric_limits<double>::quiet_NaN();
  return false;
}

size_t TaskMaster::current_motion_segment_end() const {
  if (active_custom_path_.empty() ||
      custom_rpp_index_ >= active_custom_path_.size()) {
    return custom_rpp_index_;
  }
  const bool reverse = active_custom_path_[custom_rpp_index_].reverse;
  size_t end = custom_rpp_index_;
  while (end + 1 < active_custom_path_.size() &&
         active_custom_path_[end + 1].reverse == reverse) {
    ++end;
  }
  return end;
}

double TaskMaster::custom_pass_radius(const Waypoint &waypoint,
                                      size_t index) const {
  if (index >= active_custom_path_.size() - 1) {
    return custom_rpp_goal_tolerance_;
  }
  const bool behavior_point = waypoint.qr_scan || waypoint.qr_deadline ||
                              waypoint.pause_sec > 0.0 ||
                              index == current_motion_segment_end();
  if (behavior_point) {
    const double configured =
        waypoint.reverse
            ? (waypoint.reverse_pass_radius > 0.0
                   ? waypoint.reverse_pass_radius
                   : custom_rpp_reverse_pass_radius_)
            : (waypoint.pass_radius > 0.0 ? waypoint.pass_radius
                                          : custom_rpp_pass_radius_);
    return std::min(configured, 0.10);
  }
  if (waypoint.reverse) {
    return waypoint.reverse_pass_radius > 0.0 ? waypoint.reverse_pass_radius
                                              : custom_rpp_reverse_pass_radius_;
  }
  return waypoint.pass_radius > 0.0 ? waypoint.pass_radius
                                    : custom_rpp_pass_radius_;
}

// ============================================================================
// Custom RPP — Lookahead / Target Selection
// ============================================================================

size_t TaskMaster::select_custom_target(double robot_x, double robot_y) const {
  if (custom_rpp_index_ < active_custom_path_.size() &&
      is_custom_precision_waypoint(custom_rpp_index_)) {
    return custom_rpp_index_;
  }

  // Adaptive lookahead based on upcoming curvature
  const size_t segment_end = current_motion_segment_end();
  const size_t lookahead_scan_end =
      std::min(custom_rpp_index_ + 10, segment_end + 1);
  double total_yaw_delta = 0.0;
  for (size_t i = custom_rpp_index_ + 1; i < lookahead_scan_end; ++i) {
    total_yaw_delta += std::abs(NormalizeAngle(active_custom_path_[i].yaw -
                                               active_custom_path_[i - 1].yaw));
  }
  double adaptive_lookahead =
      total_yaw_delta >= 1.0   ? custom_rpp_lookahead_dist_ * 0.55
      : total_yaw_delta >= 0.5 ? custom_rpp_lookahead_dist_ * 0.78
                               : custom_rpp_lookahead_dist_;

  size_t target_index = custom_rpp_index_;
  for (size_t index = custom_rpp_index_; index <= segment_end; ++index) {
    const auto &waypoint = active_custom_path_[index];
    double lookahead = adaptive_lookahead;
    if (waypoint.reverse) {
      lookahead = std::min(lookahead, custom_rpp_reverse_pass_radius_);
    }
    const double distance =
        std::hypot(waypoint.x - robot_x, waypoint.y - robot_y);
    target_index = index;
    if (distance >= lookahead || index == segment_end) {
      break;
    }
  }
  return target_index;
}

void TaskMaster::commit_custom_lookahead_progress(size_t target_index) {
  if (!custom_rpp_enable_lookahead_progress_ ||
      target_index <= custom_rpp_index_) {
    return;
  }

  target_index = std::min(target_index, active_custom_path_.size() - 1);
  target_index = std::min(target_index, current_motion_segment_end());
  while (custom_rpp_index_ < target_index) {
    if (active_custom_path_[custom_rpp_index_].qr_deadline) {
      lock_default_qr_direction("QR deadline crossed by lookahead");
    }
    RCLCPP_INFO(this->get_logger(),
                "Custom RPP lookahead committed waypoint %zu/%zu; "
                "tracking waypoint %zu/%zu",
                custom_rpp_index_ + 1, active_custom_path_.size(),
                target_index + 1, active_custom_path_.size());
    ++custom_rpp_index_;
  }
}

bool TaskMaster::is_custom_precision_waypoint(size_t index) const {
  if (index >= active_custom_path_.size()) {
    return false;
  }
  const auto &waypoint = active_custom_path_[index];
  return index < 2 || index == active_custom_path_.size() - 1 ||
         index == current_motion_segment_end() || waypoint.qr_scan ||
         waypoint.qr_deadline || waypoint.capture || waypoint.pause_sec > 0.0;
}

// ============================================================================
// Custom RPP — Reacquire (nearest remaining point)
// ============================================================================

size_t TaskMaster::find_nearest_progress_index(double robot_x,
                                               double robot_y) const {
  if (active_custom_path_.empty() ||
      custom_rpp_index_ >= active_custom_path_.size()) {
    return custom_rpp_index_;
  }

  size_t nearest = custom_rpp_index_;
  double nearest_dist = std::numeric_limits<double>::max();

  size_t end =
      std::min(std::min(custom_rpp_index_ + custom_rpp_reacquire_window_,
                        active_custom_path_.size()),
               current_motion_segment_end() + 1);
  for (size_t i = custom_rpp_index_ + 1; i < end; ++i) {
    if (is_custom_precision_waypoint(i)) {
      end = i + 1;
      break;
    }
  }
  for (size_t i = custom_rpp_index_; i < end; ++i) {
    const double dist = std::hypot(active_custom_path_[i].x - robot_x,
                                   active_custom_path_[i].y - robot_y);
    if (dist < nearest_dist) {
      nearest_dist = dist;
      nearest = i;
    }
  }
  return nearest;
}

void TaskMaster::reacquire_custom_progress(double robot_x, double robot_y) {
  const size_t nearest = find_nearest_progress_index(robot_x, robot_y);
  if (nearest > custom_rpp_index_) {
    RCLCPP_INFO(this->get_logger(),
                "Custom RPP reacquire: jump index %zu -> %zu (nearest "
                "remaining point at dist=%.2f)",
                custom_rpp_index_, nearest,
                std::hypot(active_custom_path_[nearest].x - robot_x,
                           active_custom_path_[nearest].y - robot_y));
    custom_rpp_index_ = nearest;
  }
}

// ============================================================================
// Custom RPP — Speed Selection
// ============================================================================

double TaskMaster::active_custom_speed(const Waypoint &target,
                                       bool capture_zone) const {
  return cruise_speed_ * compute_unified_speed_gain(target, capture_zone);
}

double TaskMaster::compute_unified_speed_gain(const Waypoint &target,
                                              bool capture_zone) const {
  return mission_policy::ComputeSpeedGain(
      target.speed_gain, qr_decision_state_ == QrDecisionState::PENDING,
      target.qr_scan, qr_scan_speed_gain_, capture_zone, capture_speed_gain_);
}

// ============================================================================
// Custom RPP — Obstacle Avoidance
// ============================================================================

geometry_msgs::msg::Twist
TaskMaster::apply_custom_obstacle_avoidance(geometry_msgs::msg::Twist cmd,
                                            bool reverse, double robot_x,
                                            double robot_y) {
  if (!custom_rpp_enable_obstacle_avoidance_) {
    return cmd;
  }
  if (static_cast<int>(custom_rpp_index_) + 1 <
      custom_rpp_obstacle_enable_from_waypoint_) {
    return cmd;
  }
  const double now = this->now().seconds();
  const bool cone_fresh = latest_cone_.valid &&
                          last_cone_detection_time_.nanoseconds() != 0 &&
                          (this->now() - last_cone_detection_time_).seconds() <=
                              cone_detection_timeout_sec_;

  if (!latest_scan_ || !scan_is_fresh()) {
    if (obstacle_state_ != ObstacleState::CLEAR || cone_fresh) {
      log_custom_obstacle_warning(
          "LaserScan unavailable during cone encounter; stopping");
      return geometry_msgs::msg::Twist();
    }
    return cmd;
  }

  double front_min = 0.0;
  double left_min = 0.0;
  double right_min = 0.0;
  if (reverse) {
    front_min = custom_min_range(M_PI - custom_rpp_front_angle_,
                                 M_PI + custom_rpp_front_angle_);
    left_min = custom_min_range(M_PI, M_PI + custom_rpp_side_angle_);
    right_min = custom_min_range(M_PI - custom_rpp_side_angle_, M_PI);
  } else {
    front_min =
        custom_min_range(-custom_rpp_front_angle_, custom_rpp_front_angle_);
    left_min = custom_min_range(0.0, custom_rpp_side_angle_);
    right_min = custom_min_range(-custom_rpp_side_angle_, 0.0);
  }

  if (!std::isfinite(front_min)) {
    if (obstacle_state_ != ObstacleState::CLEAR || cone_fresh) {
      log_custom_obstacle_warning(
          "No valid range in active obstacle sector; stopping");
      return geometry_msgs::msg::Twist();
    }
    return cmd;
  }

  if (reverse) {
    if (front_min <= obstacle_backup_distance_threshold_) {
      log_custom_obstacle_warning("Reverse path blocked; stopping");
      return geometry_msgs::msg::Twist();
    }
    return cmd;
  }

  if (obstacle_state_ == ObstacleState::BACKUP_ONCE) {
    if (!custom_rear_is_clear()) {
      obstacle_state_ = ObstacleState::WAIT_CLEAR;
      log_custom_obstacle_warning(
          "Rear clearance lost during backup; stopping");
      return geometry_msgs::msg::Twist();
    }
    const double backed_distance =
        std::hypot(robot_x - backup_start_x_, robot_y - backup_start_y_);
    const double backed_time = now - backup_start_time_sec_;
    if (backed_distance >= obstacle_backup_distance_ ||
        backed_time >= obstacle_backup_timeout_sec_) {
      obstacle_state_ = front_min <= obstacle_backup_distance_threshold_
                            ? ObstacleState::WAIT_CLEAR
                            : ObstacleState::AVOIDING;
      log_custom_obstacle_warning(backed_distance >= obstacle_backup_distance_
                                      ? "Backup distance reached"
                                      : "Backup timeout reached");
      return geometry_msgs::msg::Twist();
    }
    return custom_backup_cmd();
  }

  const bool confirmed_clear =
      !cone_fresh && front_min > obstacle_avoid_distance_;
  if (confirmed_clear) {
    if (!std::isfinite(obstacle_clear_since_sec_)) {
      obstacle_clear_since_sec_ = now;
    }
    if (obstacle_state_ != ObstacleState::CLEAR &&
        now - obstacle_clear_since_sec_ >= obstacle_clear_hold_sec_) {
      obstacle_state_ = ObstacleState::REACQUIRE_PATH;
      reacquire_custom_progress(robot_x, robot_y);
      obstacle_state_ = ObstacleState::CLEAR;
      avoid_direction_locked_ = 0.0;
      backup_executed_ = false;
      backup_start_time_sec_ = std::numeric_limits<double>::quiet_NaN();
      obstacle_clear_since_sec_ = std::numeric_limits<double>::quiet_NaN();
      log_custom_obstacle_warning("Cone cleared; path reacquired");
    } else if (obstacle_state_ != ObstacleState::CLEAR) {
      return geometry_msgs::msg::Twist();
    }
  } else {
    obstacle_clear_since_sec_ = std::numeric_limits<double>::quiet_NaN();
  }

  if (obstacle_state_ == ObstacleState::WAIT_CLEAR) {
    return geometry_msgs::msg::Twist();
  }

  // A lidar-only return inside the emergency distance remains a safety stop,
  // but only a visually confirmed cone may initiate an avoidance maneuver.
  if (!cone_fresh && obstacle_state_ == ObstacleState::CLEAR) {
    if (front_min <= obstacle_backup_distance_threshold_) {
      log_custom_obstacle_warning("Unclassified obstacle too close; stopping");
      return geometry_msgs::msg::Twist();
    }
    return cmd;
  }

  if (front_min <= obstacle_avoid_distance_ && avoid_direction_locked_ == 0.0) {
    avoid_direction_locked_ = mission_policy::ChooseAvoidDirection(
        left_min, right_min, latest_cone_.center_x_normalized);
  }

  if (front_min <= obstacle_backup_distance_threshold_) {
    if (backup_executed_ || !custom_rear_is_clear()) {
      obstacle_state_ = ObstacleState::WAIT_CLEAR;
      log_custom_obstacle_warning(
          backup_executed_ ? "Second backup prohibited; waiting clear"
                           : "Rear is not confirmed clear; backup prohibited");
      return geometry_msgs::msg::Twist();
    }
    backup_executed_ = true;
    backup_start_x_ = robot_x;
    backup_start_y_ = robot_y;
    backup_start_time_sec_ = now;
    obstacle_state_ = ObstacleState::BACKUP_ONCE;
    log_custom_obstacle_warning("Cone too close; stopping before one backup");
    return geometry_msgs::msg::Twist();
  }

  if (front_min <= obstacle_avoid_distance_) {
    obstacle_state_ = ObstacleState::AVOIDING;
    const double proximity_gain =
        Clamp((obstacle_avoid_distance_ - front_min) /
                  std::max(obstacle_avoid_distance_ -
                               obstacle_backup_distance_threshold_,
                           0.01),
              0.0, 1.0);
    const double slow_scale =
        Clamp((front_min - obstacle_backup_distance_threshold_) /
                  std::max(obstacle_slow_distance_ -
                               obstacle_backup_distance_threshold_,
                           0.01),
              0.20, 1.0);
    cmd.linear.x *= slow_scale;
    cmd.angular.z = Clamp(
        cmd.angular.z + avoid_direction_locked_ *
                            avoid_proximity_max_angular_z_ * proximity_gain,
        -custom_rpp_max_angular_z_, custom_rpp_max_angular_z_);
    log_custom_obstacle_warning("Confirmed cone soft avoid");
  }

  return cmd;
}

geometry_msgs::msg::Twist TaskMaster::custom_backup_cmd() const {
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = -std::abs(custom_rpp_backup_speed_);
  cmd.angular.z = 0.0;
  return cmd;
}

bool TaskMaster::custom_rear_is_clear() const {
  const double rear_left =
      custom_min_range(M_PI - custom_rpp_front_angle_, M_PI);
  const double rear_right =
      custom_min_range(-M_PI, -M_PI + custom_rpp_front_angle_);
  const double rear_min = std::min(rear_left, rear_right);
  return std::isfinite(rear_min) && rear_min > rear_clearance_distance_;
}

double TaskMaster::custom_min_range(double min_angle, double max_angle) const {
  if (!latest_scan_) {
    return std::numeric_limits<double>::infinity();
  }
  const auto &scan = *latest_scan_;
  double result = std::numeric_limits<double>::infinity();
  double angle = scan.angle_min;
  const double min_valid_range = std::max(static_cast<double>(scan.range_min),
                                          custom_rpp_min_obstacle_range_);
  min_angle = NormalizeAngle(min_angle);
  max_angle = NormalizeAngle(max_angle);
  for (const auto value : scan.ranges) {
    const double scan_angle = NormalizeAngle(angle);
    const bool in_sector =
        min_angle <= max_angle
            ? (min_angle <= scan_angle && scan_angle <= max_angle)
            : (scan_angle >= min_angle || scan_angle <= max_angle);
    if (in_sector) {
      if (std::isfinite(value) && min_valid_range <= value &&
          value <= scan.range_max) {
        result = std::min(result, static_cast<double>(value));
      } else if (std::isinf(value) && value > 0.0 &&
                 std::isfinite(scan.range_max)) {
        // Positive infinity is a valid LaserScan "clear beyond range_max"
        // reading. NaN or a sector outside the scanner FOV remains unknown.
        result = std::min(result, static_cast<double>(scan.range_max));
      }
    }
    angle += scan.angle_increment;
  }
  return result;
}

void TaskMaster::log_custom_obstacle_warning(const std::string &message) {
  ++obstacle_warning_count_;
  if (obstacle_warning_count_ % 10 == 1) {
    RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
  }
}

// ============================================================================
// Custom RPP — Capture Trigger
// ============================================================================

void TaskMaster::trigger_custom_capture_if_needed(double robot_x,
                                                  double robot_y) {
  for (size_t i = 0; i < active_custom_path_.size(); ++i) {
    if (i >= custom_capture_triggered_.size() || custom_capture_triggered_[i]) {
      continue;
    }
    const auto &waypoint = active_custom_path_[i];
    if (!waypoint.capture) {
      continue;
    }
    if (std::hypot(waypoint.x - robot_x, waypoint.y - robot_y) >
        capture_radius_) {
      continue;
    }
    custom_capture_triggered_[i] = true;
    capture_trigger_pub_->publish(std_msgs::msg::Empty());
    RCLCPP_INFO(this->get_logger(),
                "Capture trigger at waypoint %zu (%.2f, %.2f)", i, waypoint.x,
                waypoint.y);
  }
}

bool TaskMaster::is_custom_capture_zone(double robot_x, double robot_y) const {
  for (size_t i = 0; i < active_custom_path_.size(); ++i) {
    if (!active_custom_path_[i].capture) {
      continue;
    }
    if (std::hypot(active_custom_path_[i].x - robot_x,
                   active_custom_path_[i].y - robot_y) <= capture_radius_) {
      return true;
    }
  }
  return false;
}

// ============================================================================
// Path Trimming
// ============================================================================

std::vector<Waypoint>
TaskMaster::trim_path_from_robot_pose(const std::vector<Waypoint> &path) {
  if (path.size() <= 2) {
    return path;
  }

  double robot_x = robot_x_;
  double robot_y = robot_y_;
  double robot_yaw = robot_yaw_;
  get_robot_pose(robot_x, robot_y, robot_yaw);

  size_t nearest_index = 0;
  double nearest_distance = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path.size(); ++i) {
    const double distance =
        std::hypot(robot_x - path[i].x, robot_y - path[i].y);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_index = i;
    }
  }

  if (nearest_index > 0) {
    --nearest_index;
  }
  std::vector<Waypoint> trimmed(path.begin() + nearest_index, path.end());
  if (!trimmed.empty() && nearest_distance > 0.25) {
    const double heading =
        std::atan2(trimmed.front().y - robot_y, trimmed.front().x - robot_x);
    Waypoint connector;
    connector.x = robot_x;
    connector.y = robot_y;
    connector.yaw = heading;
    connector.pause_sec = 0.0;
    connector.motion = trimmed.front().motion;
    connector.reverse = trimmed.front().reverse;
    connector.pass_radius = trimmed.front().pass_radius;
    connector.reverse_pass_radius = trimmed.front().reverse_pass_radius;
    trimmed.insert(trimmed.begin(), connector);
  }
  RCLCPP_INFO(this->get_logger(),
              "Trim custom path from waypoint %zu/%zu near robot (%.2f, %.2f), "
              "nearest=%.2f",
              nearest_index, path.size(), robot_x, robot_y, nearest_distance);
  return trimmed;
}

// ============================================================================
// Callbacks
// ============================================================================

void TaskMaster::qr_result_callback(
    const std_msgs::msg::String::SharedPtr msg) {
  const std::lock_guard<std::mutex> lock(state_mutex_);
  RCLCPP_INFO(this->get_logger(), "QR result: %s", msg->data.c_str());

  if (current_state_ != TaskState::NAV_P_QR_B ||
      qr_decision_state_ != QrDecisionState::PENDING) {
    RCLCPP_INFO(this->get_logger(),
                "Ignoring QR result outside pending P→QR→B scan");
    return;
  }

  std::string data = msg->data;
  const int direction = mission_policy::ParseQrDirection(data);
  if (direction == 0) {
    RCLCPP_WARN(this->get_logger(), "Unknown QR direction payload: %s",
                data.c_str());
    return;
  }

  qr_direction_ = direction;
  qr_decision_state_ =
      direction == 1 ? QrDecisionState::CW_LOCKED : QrDecisionState::CCW_LOCKED;
  RCLCPP_INFO(this->get_logger(),
              "QR locked: direction=%d, current_state=%d; continuing to B.",
              qr_direction_, static_cast<int>(current_state_));
}

void TaskMaster::lock_default_qr_direction(const char *reason) {
  if (qr_decision_state_ != QrDecisionState::PENDING) {
    return;
  }
  std::string lower = default_qr_direction_;
  std::transform(
      lower.begin(), lower.end(), lower.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  const bool ccw =
      lower == "ccw" || lower == "counterclockwise" || lower == "逆时针";
  qr_direction_ = ccw ? 2 : 1;
  qr_decision_state_ =
      ccw ? QrDecisionState::CCW_LOCKED : QrDecisionState::CW_LOCKED;
  RCLCPP_WARN(this->get_logger(), "QR default locked to %s: %s",
              ccw ? "CCW" : "CW", reason);
}

void TaskMaster::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  const std::lock_guard<std::mutex> lock(state_mutex_);
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;

  const auto &q = msg->pose.pose.orientation;
  robot_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

void TaskMaster::cone_result_callback(
    const ai_msgs::msg::PerceptionTargets::SharedPtr msg) {
  const std::lock_guard<std::mutex> lock(state_mutex_);
  ConeObservation best;
  best.stamp = this->now();
  for (const auto &target : msg->targets) {
    if (target.type != "construction_cone") {
      continue;
    }
    for (const auto &roi : target.rois) {
      if (roi.confidence < cone_confidence_threshold_ || roi.rect.width == 0 ||
          roi.rect.height == 0) {
        continue;
      }
      const double height = static_cast<double>(roi.rect.height);
      const double bottom = static_cast<double>(roi.rect.y_offset) + height;
      if (best.valid && height < best.box_height &&
          std::abs(height - best.box_height) > 1.0) {
        continue;
      }
      if (best.valid && std::abs(height - best.box_height) <= 1.0 &&
          bottom <= best.box_bottom) {
        continue;
      }
      best.valid = true;
      best.confidence = roi.confidence;
      best.box_height = height;
      best.box_bottom = bottom;
      const double center = static_cast<double>(roi.rect.x_offset) +
                            0.5 * static_cast<double>(roi.rect.width);
      best.center_x_normalized =
          Clamp(center / std::max(cone_image_width_, 1.0), 0.0, 1.0);
    }
  }
  latest_cone_ = best;
  last_cone_detection_time_ = best.stamp;
}

// ============================================================================
// Utilities
// ============================================================================

void TaskMaster::announce(const std::string &text) {
  auto msg = std_msgs::msg::String();
  msg.data = text;
  announcement_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Announce: %s", text.c_str());
}

void TaskMaster::stop_robot() {
  if (cmd_vel_pub_) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
  }
}

bool TaskMaster::get_robot_pose_map(double &x, double &y, double &yaw) {
  try {
    const auto transform = tf_buffer_->lookupTransform("map", "base_footprint",
                                                       tf2::TimePointZero);
    x = transform.transform.translation.x;
    y = transform.transform.translation.y;

    const auto &q = transform.transform.rotation;
    yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    return true;
  } catch (const std::exception &e) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "map->base_footprint TF lookup failed (no fallback): %s", e.what());
    return false;
  }
}

bool TaskMaster::get_robot_pose(double &x, double &y, double &yaw) {
  try {
    const auto transform = tf_buffer_->lookupTransform("map", "base_footprint",
                                                       tf2::TimePointZero);
    x = transform.transform.translation.x;
    y = transform.transform.translation.y;

    const auto &q = transform.transform.rotation;
    yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    return true;
  } catch (const std::exception &e) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Failed to lookup map->base_footprint TF, fallback to odom cache: %s",
        e.what());
  }

  x = robot_x_;
  y = robot_y_;
  yaw = robot_yaw_;
  return true;
}

bool TaskMaster::scan_is_fresh() const {
  if (latest_scan_time_.nanoseconds() == 0) {
    return false;
  }
  const double age = (this->now() - latest_scan_time_).seconds();
  return age <= custom_rpp_scan_timeout_sec_;
}

} // namespace origincar_task

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<origincar_task::TaskMaster>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
