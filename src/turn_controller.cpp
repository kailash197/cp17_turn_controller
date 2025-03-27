#include <Eigen/Dense>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>

using namespace std::chrono_literals;

class PID {
public:
  PID() : kp_(0.0), ki_(0.0), kd_(0.0), dt_(0.0) {}

  PID(double kp, double ki, double kd, double dt)
      : kp_(kp), ki_(ki), kd_(kd), dt_(dt), integral_(0.0), prev_error_(0.0) {}

  double compute(double error) {
    integral_ += error * dt_;
    integral_ = std::clamp(integral_, -0.5, 0.5); // Anti-windup
    double derivative = (error - prev_error_) / dt_;
    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    prev_error_ = error;
    return output;
  }

  void reset_() {
    integral_ = 0.0;
    prev_error_ = 0.0;
  }

private:
  double kp_, ki_, kd_, dt_;
  double integral_;
  double prev_error_;
};

// Function to normalize angle to the range -pi to pi
double normalize_angle(double angle) {
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

class TurnController : public rclcpp::Node {
private:
  int scene_number_;
  double max_velocity_;
  double max_ang_velocity_;
  std::vector<std::vector<double>> waypoints_; //{dx, dy, dphi}

  void SelectWaypoints();
  void pid_controller();
  std::vector<double> velocity2twist(double vx, double vy, double avz);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_grp_;
  rclcpp::CallbackGroup::SharedPtr odom_cb_grp_;

  geometry_msgs::msg::Point current_position_;
  double phi; // current_yaw_

  PID pid_z_;
  double kp_sim, ki_sim, kd_sim;
  double kp_real, ki_real, kd_real;
  double time_step;

public:
  TurnController(int scene_number);
  ~TurnController();
};

TurnController::~TurnController() {
  RCLCPP_INFO(this->get_logger(), "Turn Controller Terminated.");
}

void TurnController::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Extract position
  current_position_ = msg->pose.pose.position;

  // Extract orientation (quaternion)
  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

  // Convert quaternion to Euler angles (roll, pitch, yaw)
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  phi = yaw;

  // Log the position and orientation for debugging
  RCLCPP_DEBUG(this->get_logger(),
               "Position: [x: %f, y: %f, z: %f], Orientation (yaw): %f",
               current_position_.x, current_position_.y, current_position_.z,
               phi);
}

TurnController::TurnController(int scene_number)
    : Node("turn_controller_node"), scene_number_(scene_number) {

  timer_cb_grp_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  odom_cb_grp_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = odom_cb_grp_;

  cmd_vel_publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", 10,
      std::bind(&TurnController::odom_callback, this, std::placeholders::_1),
      options);

  // PID Parameters
  kp_sim = 2.0, ki_sim = 0.01, kd_sim = 0.30;
  kp_real = 2.0, ki_real = 0.01, kd_real = 0.30;
  time_step = 0.01; // in milliseconds
  SelectWaypoints();

  RCLCPP_INFO(this->get_logger(), "Turn Controller Initialized.");

  timer_ = this->create_wall_timer(
      1s, std::bind(&TurnController::pid_controller, this), timer_cb_grp_);
}

void TurnController::pid_controller() {
  double dx, dy, dphi;
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  RCLCPP_INFO(this->get_logger(), "Trajectory started.");

  // Loop through each waypoint
  int index = 0;
  for (const auto &waypoint : waypoints_) {
    pid_z_.reset_();
    rclcpp::Rate rate(int(1 / time_step)); // Control loop frequency

    dx = waypoint[0];
    dy = waypoint[1];
    dphi = waypoint[2];
    RCLCPP_INFO(this->get_logger(), "WP%u: [%.2f, %.2f, %.2f]", ++index, dx, dy,
                dphi);

    double target_z = normalize_angle(phi + dphi);
    double error_z = std::numeric_limits<double>::max();

    while (fabs(error_z) > 0.007) {
      if (!rclcpp::ok()) { // Check if ROS is still running
        RCLCPP_WARN(this->get_logger(), "Trajectory Canceled.");
        timer_->cancel();         // Stop the timer
        odom_subscriber_.reset(); // Kill the odometry subscription
        rclcpp::shutdown();
        return;
      }

      // Calculate error
      error_z = target_z - phi;
      error_z =
          atan2(sin(error_z), cos(error_z)); // Normalize error to [-pi, pi]
      RCLCPP_DEBUG(this->get_logger(), "Angle to target: %.2f", error_z);

      // PID control
      double angular_vel = pid_z_.compute(error_z);
      angular_vel =
          std::clamp(angular_vel, -max_ang_velocity_, max_ang_velocity_);
      cmd_vel.angular.z = angular_vel;
      cmd_vel_publisher_->publish(cmd_vel);
      RCLCPP_DEBUG(this->get_logger(), "Angular vel: %.3f", cmd_vel.angular.z);

      rate.sleep(); // Maintain loop frequency
    }
    // Now stop the bot
    cmd_vel.angular.z = 0.0;
    cmd_vel_publisher_->publish(cmd_vel);
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }

  RCLCPP_INFO(this->get_logger(), "Trajectory completed.");
  timer_->cancel();         // Stop the timer
  odom_subscriber_.reset(); // Kill the odometry subscription
  rclcpp::shutdown();
}

void TurnController::SelectWaypoints() {
  switch (scene_number_) {
  case 1: // Simulation
    // Assign waypoints for Simulation
    RCLCPP_INFO(this->get_logger(), "Welcome to Simulation!");
    /* https://husarion.com/manuals/rosbot-xl/
    Maximum translational velocity = 0.8 m/s
    Maximum rotational velocity = 180 deg/s (3.14 rad/s)
    */
    max_velocity_ = 1.0;
    max_ang_velocity_ = 3.0;
    // Waypoints: {dx,dy,dphi}
    waypoints_ = {
        {0.0, 0.0, -1.1254}, // w1
        {0.0, 0.0, +1.0021}, // w2
        {0.0, 0.0, +1.0572}, // w3
        {0.0, 0.0, -0.9339}, // w4
    };
    pid_z_ = PID(kp_sim, ki_sim, kd_sim, time_step);
    break;

  case 2: // CyberWorld
    // Assign waypoints for CyberWorld
    RCLCPP_INFO(this->get_logger(), "Welcome to CyberWorld!");
    /* https://husarion.com/manuals/rosbot-xl/
    Maximum translational velocity = 0.8 m/s
    Maximum rotational velocity = 180 deg/s (3.14 rad/s)
    */
    max_velocity_ = 0.5;
    max_ang_velocity_ = 0.5;
    waypoints_ = {
        {0.0, 0.0, -0.5236}, // w1
        {0.0, 0.0, -0.7854}, // w2
        {0.0, 0.0, +1.3090}, // w3
    };
    pid_z_ = PID(kp_real, ki_real, kd_real, time_step);
    break;

  default:
    RCLCPP_ERROR(this->get_logger(), "Invalid Scene Number: %d", scene_number_);
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Check if a scene number argument is provided
  int scene_number = 1; // Default scene number to simulation
  if (argc > 1) {
    scene_number = std::atoi(argv[1]);
  }
  // Check if the scene number is valid before creating the node
  if (scene_number != 1 && scene_number != 2) {
    std::cerr << "Error: Invalid Scene Number -- " << scene_number << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  // Add as input variable the scene number
  auto node = std::make_shared<TurnController>(scene_number);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
