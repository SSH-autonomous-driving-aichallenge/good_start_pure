#ifndef SIMPLE_PURE_PURSUITHPP
#define SIMPLE_PURE_PURSUITHPP

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>

namespace simple_pure_pursuit {

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PointStamped;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;

class SimplePurePursuit : public rclcpp::Node {
 public:
  explicit SimplePurePursuit();

  // subscribers
  rclcpp::Subscription<Odometry>::SharedPtr subkinematics;
  rclcpp::Subscription<Trajectory>::SharedPtr subtrajectory;

  // publishers
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pubcmd;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_rawcmd;
  rclcpp::Publisher<PointStamped>::SharedPtr pub_lookaheadpoint;

  // timer
  rclcpp::TimerBase::SharedPtr timer;

  // updated by subscribers
  Trajectory::SharedPtr trajectory;
  Odometry::SharedPtr odometry_;

  // pure pursuit parameters
  const double wheelbase;
  const double lookaheadgain;
  const double lookahead_mindistance;
  const double speed_proportionalgain;
  const bool use_external_targetvel;
  const double external_targetvel;
  const double steering_tire_anglegain;

private:
  void onTimer();
  bool subscribeMessageAvailable();

  // 追加メンバ変数
  size_t forced_minindex;         // kの値。指定した整数
  bool forced_indexskipped;       // 1度kを超えたらtrueにして処理解除
};

}  // namespace simple_pure_pursuit

#endif  // SIMPLE_PURE_PURSUITHPP