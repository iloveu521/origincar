/*
 * TaskMaster - Competition FSM Orchestrator for OriginCar
 * 21st National University Student Intelligent Car Competition - D-Robotics
 */

#ifndef ORIGINCAR_TASK__TASK_MASTER_HPP_
#define ORIGINCAR_TASK__TASK_MASTER_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "ai_msgs/msg/perception_targets.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace origincar_task {

struct Waypoint {
  double x;
  double y;
  double yaw;
  double pause_sec;
  // ---- motion control (Phase 3) ----
  std::string motion = "forward"; // "forward" or "reverse"
  bool reverse = false;           // derived from motion, kept for compatibility
  double pass_radius = 0.0;
  double reverse_pass_radius = 0.0;
  // ---- per-waypoint gains (Phase 3) ----
  double speed_gain = 1.0;   // per-waypoint speed gain, 1.0 = no change
  double angular_gain = 1.0; // per-waypoint angular gain, 1.0 = no change
  // ---- behavior flags (Phase 3) ----
  bool capture = false;     // 触发拍照
  bool qr_scan = false;     // QR 扫描窗口
  bool qr_deadline = false; // QR 截止点
};

struct ConeObservation {
  bool valid = false;
  double confidence = 0.0;
  double center_x_normalized = 0.5;
  double box_height = 0.0;
  double box_bottom = 0.0;
  rclcpp::Time stamp;
};

enum class TaskState {
  WAIT_TF, // Wait for stable map->base_footprint TF before starting
  IDLE,
  NAV_P_QR_B, // P→QR→B continuous path (Phase 4)
  RING_NAV,   // C zone ring navigation
  NAV_TO_P,   // Return to P
  PARK,
  DONE
};

enum class NavStatus { IDLE, RUNNING, SUCCEEDED, FAILED };

// QR 方向状态 (Phase 5)
enum class QrDecisionState {
  PENDING,   // 尚未识别
  CW_LOCKED, // 顺时针已锁定
  CCW_LOCKED // 逆时针已锁定
};

// 障碍物状态 (Phase 9)
enum class ObstacleState {
  CLEAR,         // 无障碍
  AVOIDING,      // 减速绕行
  BACKUP_ONCE,   // 一次性倒车
  WAIT_CLEAR,    // 倒车后等待
  REACQUIRE_PATH // 重新追点
};

class TaskMaster : public rclcpp::Node {
public:
  TaskMaster();
  ~TaskMaster() = default;

private:
  // ── FSM (10 Hz) ──
  void state_machine_loop();
  void transition_to(TaskState new_state);

  void handle_wait_tf();
  void handle_idle();
  void handle_nav_p_qr_b(); // P→QR→B continuous navigation (Phase 4)
  void handle_ring_nav();
  void handle_nav_to_p();
  void handle_park();

  // ── Controller (20 Hz) ──
  void control_loop();

  bool navigation_succeeded() const;
  bool navigation_failed();

  // ── Subscriptions ──
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr qr_result_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr cone_sub_;

  void qr_result_callback(const std_msgs::msg::String::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void
  cone_result_callback(const ai_msgs::msg::PerceptionTargets::SharedPtr msg);

  // ── Publishers ──
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr announcement_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr capture_trigger_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  void announce(const std::string &text);

  // ── TF / Pose ──
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool get_robot_pose_map(double &x, double &y, double &yaw);
  bool get_robot_pose(double &x, double &y, double &yaw);

  bool is_map_tf_stable_once();

  // ── Waypoints ──
  void load_waypoints();
  std::vector<Waypoint> build_p_to_qr_b_path(); // P→QR→B merged path (Phase 4)

  // ── Custom RPP ──
  bool start_custom_rpp(const std::vector<Waypoint> &path, double speed,
                        const char *phase_name);
  void update_custom_rpp();
  void stop_robot();
  double active_custom_speed(const Waypoint &target, bool capture_zone) const;
  geometry_msgs::msg::Twist
  apply_custom_obstacle_avoidance(geometry_msgs::msg::Twist cmd, bool reverse,
                                  double robot_x, double robot_y);
  geometry_msgs::msg::Twist custom_backup_cmd() const;
  bool custom_rear_is_clear() const;
  double custom_min_range(double min_angle, double max_angle) const;
  void log_custom_obstacle_warning(const std::string &message);
  void advance_custom_waypoints(double robot_x, double robot_y);
  size_t select_custom_target(double robot_x, double robot_y) const;
  void commit_custom_lookahead_progress(size_t target_index);
  bool is_custom_precision_waypoint(size_t index) const;
  void trigger_custom_capture_if_needed(double robot_x, double robot_y);
  bool is_custom_capture_zone(double robot_x, double robot_y) const;
  bool handle_custom_pause();
  void lock_default_qr_direction(const char *reason);
  size_t current_motion_segment_end() const;
  double custom_pass_radius(const Waypoint &waypoint, size_t index) const;

  size_t find_nearest_progress_index(double robot_x, double robot_y) const;
  void reacquire_custom_progress(double robot_x, double robot_y);

  bool scan_is_fresh() const;

  // Unified speed gain computation (Phase 6)
  double compute_unified_speed_gain(const Waypoint &target,
                                    bool capture_zone) const;

  std::vector<Waypoint>
  trim_path_from_robot_pose(const std::vector<Waypoint> &path);

  // ── Key waypoints ──
  Waypoint wp_p_;
  std::vector<Waypoint> route_p_to_qr_b_; // P→QR→B merged route (Phase 4)
  std::vector<Waypoint> ring_cw_;
  std::vector<Waypoint> ring_ccw_;
  std::vector<Waypoint> route_return_to_p_;

  // ── QR (Phase 5) ──
  QrDecisionState qr_decision_state_; // QR 方向锁存状态
  int qr_direction_;                  // 0=unknown, 1=CW (odd), 2=CCW (even)

  // ── Robot state ──
  double robot_x_;
  double robot_y_;
  double robot_yaw_;

  TaskState current_state_;
  rclcpp::Time state_start_time_;
  rclcpp::Time nav_start_time_;
  NavStatus nav_status_;

  // ── Custom RPP state ──
  std::vector<Waypoint> active_custom_path_;
  std::vector<bool> custom_capture_triggered_;
  std::vector<bool> custom_pause_completed_;
  std::string active_custom_phase_;
  size_t custom_rpp_index_;
  double custom_rpp_active_speed_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  int obstacle_warning_count_;
  bool motion_switch_pending_;
  size_t paused_waypoint_index_;
  double pause_until_time_sec_;

  // ── Obstacle fused avoidance (Phase 9) ──
  ObstacleState obstacle_state_;
  double avoid_direction_locked_;         // 锁定绕行方向
  double backup_start_x_;                 // 倒车起始里程X
  double backup_start_y_;                 // 倒车起始里程Y
  bool backup_executed_;                  // 当前锥桶是否已倒车
  double obstacle_clear_since_sec_;       // 障碍物清除起始时间
  rclcpp::Time last_cone_detection_time_; // 最近锥桶检测时间
  ConeObservation latest_cone_;
  double backup_start_time_sec_;

  // ── Timers ──
  rclcpp::TimerBase::SharedPtr fsm_timer_;     // 10 Hz — state machine
  rclcpp::TimerBase::SharedPtr control_timer_; // 20 Hz — custom RPP controller

  // ── Constants ──
  static constexpr double PARK_STOP_SEC = 1.0;

  // ── WAIT_TF startup params ──
  int startup_tf_stable_count_;
  int startup_tf_required_frames_;
  double startup_tf_check_timeout_sec_;
  double startup_tf_max_age_sec_;

  // ── TF-loss hold params ──
  double custom_rpp_pose_hold_sec_;
  geometry_msgs::msg::Twist last_custom_cmd_;
  bool has_last_custom_cmd_;
  double last_valid_pose_time_;

  // ── Scan freshness ──
  rclcpp::Time latest_scan_time_;
  double custom_rpp_scan_timeout_sec_;

  // ── Reacquire window ──
  int custom_rpp_reacquire_window_;

  // ── Unified speed & gain params (Phase 6) ──
  double cruise_speed_;       // 基础巡航速度 0.85 m/s
  double qr_scan_speed_gain_; // QR 扫描增益
  double turn_speed_gain_;    // 弯道增益
  double turn_angular_gain_;  // 弯道角速度增益
  double capture_speed_gain_; // 拍照速度增益
  double capture_radius_;     // 拍照触发半径

  // ── Obstacle fused params (Phase 9) ──
  double cone_confidence_threshold_;
  double obstacle_backup_speed_gain_;
  double obstacle_backup_distance_;
  double obstacle_backup_timeout_sec_;
  double rear_clearance_distance_;
  double cone_detection_timeout_sec_;
  double obstacle_clear_hold_sec_;
  double avoid_proximity_max_angular_z_;
  double obstacle_backup_distance_threshold_;
  double obstacle_slow_distance_;
  double obstacle_avoid_distance_;
  double cone_image_width_;
  std::string default_qr_direction_;

  // Protects all mutable task state if a MultiThreadedExecutor is used.
  mutable std::mutex state_mutex_;

  // ── RPP params ──
  double custom_rpp_lookahead_dist_;
  double custom_rpp_pass_radius_;
  double custom_rpp_reverse_pass_radius_;
  double custom_rpp_goal_tolerance_;
  double custom_rpp_max_angular_z_;
  double custom_rpp_turn_gain_;
  bool custom_rpp_enable_lookahead_progress_;
  bool custom_rpp_enable_obstacle_avoidance_;
  int custom_rpp_obstacle_enable_from_waypoint_;
  double custom_rpp_min_obstacle_range_;
  double custom_rpp_front_angle_;
  double custom_rpp_side_angle_;
  double custom_rpp_backup_speed_;
};

} // namespace origincar_task

#endif // ORIGINCAR_TASK__TASK_MASTER_HPP_
