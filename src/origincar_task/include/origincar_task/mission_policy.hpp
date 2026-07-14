#ifndef ORIGINCAR_TASK__MISSION_POLICY_HPP_
#define ORIGINCAR_TASK__MISSION_POLICY_HPP_

#include <algorithm>
#include <cctype>
#include <cmath>
#include <string>

namespace origincar_task::mission_policy {

inline int ParseQrDirection(const std::string &payload) {
  std::string lower = payload;
  std::transform(
      lower.begin(), lower.end(), lower.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (lower.find("ccw") != std::string::npos ||
      lower.find("counterclockwise") != std::string::npos ||
      payload.find("逆时针") != std::string::npos) {
    return 2;
  }
  if (lower == "cw" || lower.find("clockwise") != std::string::npos ||
      payload.find("顺时针") != std::string::npos) {
    return 1;
  }
  try {
    size_t parsed = 0;
    const int number = std::stoi(payload, &parsed);
    if (parsed != payload.size()) {
      return 0;
    }
    return number % 2 != 0 ? 1 : 2;
  } catch (const std::exception &) {
    return 0;
  }
}

inline double ComputeSpeedGain(double waypoint_gain, bool qr_pending,
                               bool qr_scan, double qr_gain, bool capture_zone,
                               double capture_gain) {
  double gain = std::min(1.0, waypoint_gain);
  if (qr_pending && qr_scan) {
    gain = std::min(gain, qr_gain);
  }
  if (capture_zone) {
    gain = std::min(gain, capture_gain);
  }
  return gain;
}

inline double ChooseAvoidDirection(double left_clearance,
                                   double right_clearance,
                                   double cone_center_normalized) {
  const bool left_valid = std::isfinite(left_clearance);
  const bool right_valid = std::isfinite(right_clearance);
  if (left_valid && right_valid &&
      std::abs(left_clearance - right_clearance) >= 0.05) {
    return left_clearance > right_clearance ? 1.0 : -1.0;
  }
  if (left_valid != right_valid) {
    return left_valid ? 1.0 : -1.0;
  }
  return cone_center_normalized < 0.5 ? -1.0 : 1.0;
}

} // namespace origincar_task::mission_policy

#endif // ORIGINCAR_TASK__MISSION_POLICY_HPP_
