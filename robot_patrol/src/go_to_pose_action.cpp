#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_patrol_msg/action/go_to_pose.hpp"

using namespace std::chrono_literals;

class GoToPose : public rclcpp::Node {
   public:
    using GoToPoseAction = robot_patrol_msg::action::GoToPose;
    using GoalHandleGoToPoseAction = rclcpp_action::ServerGoalHandle<GoToPoseAction>;

    explicit GoToPose(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("go_to_pose_action", options) {
        using namespace std::placeholders;

        // Create callback groups for handling subscriptions and timers
        odom_sub_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        RCLCPP_INFO(this->get_logger(), "Initialized Callback Groups !");

        // Create the action server
        this->action_server_ = rclcpp_action::create_server<GoToPoseAction>(
            this,
            "/go_to_pose",
            std::bind(&GoToPose::handle_goal, this, _1, _2),
            std::bind(&GoToPose::handle_cancel, this, _1),
            std::bind(&GoToPose::handle_accepted, this, _1));

        // Subscribe to odometry to get the robot's current position
        rclcpp::SubscriptionOptions odom_sub_options_;
        odom_sub_options_.callback_group = odom_sub_group_;
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/fastbot_1/odom", 10,
            std::bind(&GoToPose::odom_callback, this, _1), odom_sub_options_);

        // Create publisher for velocity commands
        cmd_vel_publisher_ =
            this->create_publisher<geometry_msgs::msg::Twist>("/fastbot_1/cmd_vel", 10);

        // Create a timer for controlling the robot's movement
        control_timer_ = this->create_wall_timer(
            100ms, std::bind(&GoToPose::control_loop, this), timer_group_);

        RCLCPP_INFO(this->get_logger(), "GoToPose action server is ready");
    }

   private:
    rclcpp::CallbackGroup::SharedPtr odom_sub_group_;
    rclcpp::CallbackGroup::SharedPtr timer_group_;
    rclcpp_action::Server<GoToPoseAction>::SharedPtr action_server_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    geometry_msgs::msg::Pose2D desired_pos_;
    geometry_msgs::msg::Pose2D current_pos_;
    bool goal_active_ = false;
    std::shared_ptr<GoalHandleGoToPoseAction> active_goal_handle_;

    // Callback for odometry - updates current robot position
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Update current x and y positions
        current_pos_.x = msg->pose.pose.position.x;
        current_pos_.y = msg->pose.pose.position.y;

        // Convert quaternion to Euler angles to get theta (yaw)
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // Store theta (yaw) in radians
        current_pos_.theta = yaw;
    }

    // Handle incoming goal requests
    rclcpp_action::GoalResponse
    handle_goal(const rclcpp_action::GoalUUID& uuid,
                std::shared_ptr<const GoToPoseAction::Goal> goal) {
        RCLCPP_INFO(this->get_logger(),
                    "Received goal request: x=%.2f, y=%.2f, theta=%.2f",
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
    void handle_accepted(const std::shared_ptr<GoalHandleGoToPoseAction> goal_handle) {
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
        goal_active_ = true;
        active_goal_handle_ = goal_handle;

        RCLCPP_INFO(this->get_logger(),
                    "Goal position set: x=%.2f, y=%.2f, theta=%.2f",
                    desired_pos_.x, desired_pos_.y, desired_pos_.theta);

        // Position tolerance (meters) and orientation tolerance (radians)
        const double position_tolerance = 0.1;     // 10 cm
        const double orientation_tolerance = 0.1;  // ~5.7 degrees

        // Publish feedback every 1 second
        rclcpp::Rate loop_rate(1);  // 1 Hz for feedback

        while (rclcpp::ok() && goal_active_) {
            // Check if there is a cancel request
            if (goal_handle->is_canceling()) {
                result->status = false;
                goal_handle->canceled(result);
                goal_active_ = false;
                RCLCPP_INFO(this->get_logger(), "Goal canceled");

                // Stop the robot
                auto stop_msg = geometry_msgs::msg::Twist();
                cmd_vel_publisher_->publish(stop_msg);
                return;
            }

            // Calculate distance to goal
            double dx = desired_pos_.x - current_pos_.x;
            double dy = desired_pos_.y - current_pos_.y;
            double distance = std::sqrt(dx * dx + dy * dy);

            // Calculate orientation error
            double orientation_error = desired_pos_.theta - current_pos_.theta;
            // Normalize to [-pi, pi]
            while (orientation_error > M_PI) orientation_error -= 2.0 * M_PI;
            while (orientation_error < -M_PI) orientation_error += 2.0 * M_PI;

            // Publish feedback with current position every second
            feedback->current_pos = current_pos_;
            goal_handle->publish_feedback(feedback);

            RCLCPP_INFO(this->get_logger(),
                        "Feedback - Current: [%.2f, %.2f, %.2f], Distance: %.2f m",
                        current_pos_.x, current_pos_.y, current_pos_.theta, distance);

            // Check if goal is reached
            if (distance < position_tolerance && std::abs(orientation_error) < orientation_tolerance) {
                result->status = true;
                goal_handle->succeed(result);
                goal_active_ = false;

                // Stop the robot
                auto stop_msg = geometry_msgs::msg::Twist();
                cmd_vel_publisher_->publish(stop_msg);

                RCLCPP_INFO(this->get_logger(), "Goal reached successfully!");
                return;
            }

            loop_rate.sleep();
        }
    }

    void control_loop() {
        // Only execute control if a goal is active
        if (!goal_active_) {
            return;
        }

        auto twist_msg = geometry_msgs::msg::Twist();

        // 1. Get current position (current_pos_) - updated by odom callback
        // 2. Get desired position (desired_pos_) - set by execute()
        // 3. Compute the difference between the two
        double dx = desired_pos_.x - current_pos_.x;
        double dy = desired_pos_.y - current_pos_.y;

        // 4. This difference defines a vector with the direction we need to move
        // Calculate the desired angle to reach the goal
        double desired_angle = std::atan2(dy, dx);

        // Calculate the difference between current orientation and desired angle
        double angle_diff = desired_angle - current_pos_.theta;

        // Normalize angle difference to [-pi, pi]
        while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

        // 5. Convert the vector direction into angular speed (angular.z)
        // Use proportional control: angular velocity proportional to angle error
        twist_msg.angular.z = angle_diff;

        // 6. Send the speed to /cmd_vel with fixed linear.x of 0.2 m/s
        twist_msg.linear.x = 0.2;

        // Publish the velocity command
        cmd_vel_publisher_->publish(twist_msg);

        RCLCPP_DEBUG(this->get_logger(),
                     "Control: dx=%.2f, dy=%.2f, desired_angle=%.2f, angle_diff=%.2f",
                     dx, dy, desired_angle, angle_diff);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoToPose>();

    // Create a multi-threaded executor to handle callbacks concurrently
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
