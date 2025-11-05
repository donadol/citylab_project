#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_patrol/action/go_to_pose.hpp"

using namespace std::chrono_literals;

class GoToPose : public rclcpp::Node {
   public:
    using GoToPoseAction = robot_patrol::action::GoToPose;
    using GoalHandleGoToPoseAction =
        rclcpp_action::ServerGoalHandle<GoToPoseAction>;

    explicit GoToPose(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("go_to_pose_action", options) {
        using namespace std::placeholders;

        // Create the action server
        this->action_server_ = rclcpp_action::create_server<GoToPoseAction>(
            this, "/go_to_pose", std::bind(&GoToPose::handle_goal, this, _1, _2),
            std::bind(&GoToPose::handle_cancel, this, _1),
            std::bind(&GoToPose::handle_accepted, this, _1));

        // Subscribe to odometry to get the robot's current position
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&GoToPose::odom_callback, this, _1));

        // Create publisher for velocity commands
        cmd_vel_publisher_ =
            this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Create a timer for controlling the robot's movement (10 Hz)
        control_timer_ = this->create_wall_timer(
            100ms, std::bind(&GoToPose::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "GoToPose action server is ready");
    }

   private:
    rclcpp_action::Server<GoToPoseAction>::SharedPtr action_server_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    geometry_msgs::msg::Pose2D desired_pos_;
    geometry_msgs::msg::Pose2D current_pos_;
    bool goal_active_ = false;

    // Control parameters - tuned for real robot
    const double LINEAR_VELOCITY = 0.1;       // m/s
    const double ANGULAR_GAIN = 0.8;          // Proportional gain for rotation
    const double MAX_ANGULAR_VELOCITY = 1.5;  // rad/s
    const double MIN_ANGULAR_VELOCITY =
        0.4;                                   // rad/s (minimum to overcome friction)
    const double POSITION_TOLERANCE = 0.1;     // m (10 cm)
    const double ORIENTATION_TOLERANCE = 0.5;  // rad

    // Callback for odometry - updates current robot position
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Update current x and y positions
        current_pos_.x = msg->pose.pose.position.x;
        current_pos_.y = msg->pose.pose.position.y;

        // Convert quaternion to Euler angles to get theta (yaw)
        tf2::Quaternion q(
            msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_pos_.theta = yaw;
    }

    // Handle incoming goal requests
    rclcpp_action::GoalResponse
    handle_goal(const rclcpp_action::GoalUUID& uuid,
                std::shared_ptr<const GoToPoseAction::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Action Called: x=%.2f, y=%.2f, theta=%.2f",
                    goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.theta);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Handle cancellation requests
    rclcpp_action::CancelResponse
    handle_cancel(const std::shared_ptr<GoalHandleGoToPoseAction> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // Handle accepted goals
    void
    handle_accepted(const std::shared_ptr<GoalHandleGoToPoseAction> goal_handle) {
        using namespace std::placeholders;
        // Execute in a new thread to avoid blocking the executor
        std::thread{std::bind(&GoToPose::execute, this, _1), goal_handle}.detach();
    }

    // Execute the goal
    void execute(const std::shared_ptr<GoalHandleGoToPoseAction> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal");

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<GoToPoseAction::Feedback>();
        auto result = std::make_shared<GoToPoseAction::Result>();

        // Store the desired position and activate goal
        desired_pos_ = goal->goal_pos;
        // Convert theta from degrees to radians
        desired_pos_.theta = goal->goal_pos.theta * M_PI / 180.0;
        goal_active_ = true;

        RCLCPP_INFO(this->get_logger(),
                    "Goal position set: x=%.2f, y=%.2f, theta=%.2f rad",
                    desired_pos_.x, desired_pos_.y, desired_pos_.theta);

        // Publish feedback 10hz
        rclcpp::Rate loop_rate(10);

        while (rclcpp::ok() && goal_active_) {
            // Handle cancellation
            if (goal_handle->is_canceling()) {
                stop_robot();
                result->status = false;
                goal_handle->canceled(result);
                goal_active_ = false;
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            // Calculate distance to goal
            double dx = desired_pos_.x - current_pos_.x;
            double dy = desired_pos_.y - current_pos_.y;
            double distance = std::sqrt(dx * dx + dy * dy);

            // Calculate orientation error
            double orientation_error =
                normalize_angle(desired_pos_.theta - current_pos_.theta);

            // Publish feedback
            feedback->current_pos = current_pos_;
            goal_handle->publish_feedback(feedback);

            RCLCPP_INFO(this->get_logger(),
                        "Feedback - Current: [%.2f, %.2f, %.2f], Distance: %.2f m, "
                        "Orientation error: %.2f rad",
                        current_pos_.x, current_pos_.y, current_pos_.theta, distance,
                        orientation_error);

            // Check if goal is reached
            if (distance < POSITION_TOLERANCE &&
                std::fabs(orientation_error) < ORIENTATION_TOLERANCE) {
                stop_robot();
                result->status = true;
                goal_handle->succeed(result);
                goal_active_ = false;
                RCLCPP_INFO(this->get_logger(), "Action Completed");
                return;
            }

            loop_rate.sleep();
        }
    }

    void control_loop() {
        // Return early if no active goal
        if (!goal_active_) {
            return;
        }

        // Calculate distance to goal
        double dx = desired_pos_.x - current_pos_.x;
        double dy = desired_pos_.y - current_pos_.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        // Phase 2: Rotate in place when close to goal
        if (distance < POSITION_TOLERANCE) {
            double orientation_error =
                normalize_angle(desired_pos_.theta - current_pos_.theta);

            // Apply gain and clamp angular velocity
            double angular_vel =
                clamp_angular_velocity(ANGULAR_GAIN * orientation_error);

            publish_velocity(0.0, angular_vel);
            RCLCPP_DEBUG(this->get_logger(),
                         "Final rotation - orientation_error=%.2f, angular_vel=%.2f",
                         orientation_error, angular_vel);
            return;
        }

        // Phase 1: Navigate to position
        double desired_angle = std::atan2(dy, dx);
        double angle_diff = normalize_angle(desired_angle - current_pos_.theta);

        // Apply gain and clamp angular velocity
        double angular_vel = clamp_angular_velocity(ANGULAR_GAIN * angle_diff);

        publish_velocity(LINEAR_VELOCITY, angular_vel);
        RCLCPP_DEBUG(this->get_logger(),
                     "Moving to position - distance=%.2f, angle_diff=%.2f",
                     distance, angle_diff);
    }

    // Helper method to normalize angles to [-pi, pi]
    double normalize_angle(double angle) {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }

    // Helper method to clamp angular velocity to safe limits and apply minimum
    // threshold
    double clamp_angular_velocity(double angular_vel) {
        // Apply maximum limits
        if (angular_vel > MAX_ANGULAR_VELOCITY) {
            return MAX_ANGULAR_VELOCITY;
        }
        if (angular_vel < -MAX_ANGULAR_VELOCITY) {
            return -MAX_ANGULAR_VELOCITY;
        }

        // Apply minimum threshold to overcome motor deadband
        if (angular_vel > 0.0 && angular_vel < MIN_ANGULAR_VELOCITY) {
            return MIN_ANGULAR_VELOCITY;
        }
        if (angular_vel < 0.0 && angular_vel > -MIN_ANGULAR_VELOCITY) {
            return -MIN_ANGULAR_VELOCITY;
        }

        return angular_vel;
    }

    // Helper method to publish velocity commands
    void publish_velocity(double linear_x, double angular_z) {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = linear_x;
        twist_msg.angular.z = angular_z;
        cmd_vel_publisher_->publish(twist_msg);
    }

    // Helper method to stop the robot
    void stop_robot() { publish_velocity(0.0, 0.0); }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoToPose>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
