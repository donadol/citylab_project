#include <chrono>
#include <csignal>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

class Patrol : public rclcpp::Node {
   public:
    Patrol() : Node("patrol_node"), direction_(0.0) {
        laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&Patrol::laser_scan_callback, this, _1));

        cmd_vel_publisher_ =
            this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&Patrol::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Patrol node initialized");
    }

    void stop_robot() {
        auto stop_msg = geometry_msgs::msg::Twist();
        // All velocities default to 0.0
        cmd_vel_publisher_->publish(stop_msg);
        RCLCPP_INFO(this->get_logger(), "Robot stopped - shutting down");
    }

   private:
    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // For real robot with 0 to 2π range:
        // Front 180° = -90° to +90° = 270° to 90° in 0-2π notation
        // = 3π/2 to π/2 (wrapping through 0)
        const double FRONT_RIGHT_MIN = 3.0 * M_PI / 2.0;  // 270° = 3π/2
        const double FRONT_LEFT_MAX = M_PI / 2.0;         // 90° = π/2
        const double MIN_CLEARANCE = 0.35;
        const double CENTER_ANGLE_TOLERANCE = 0.3;

        float max_distance = 0.0;
        float safest_angle = 0.0;
        float min_front_distance = MIN_CLEARANCE + 0.1;  // Initialize above clearance

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float angle = msg->angle_min + (i * msg->angle_increment);

            // Only consider front 180 degrees
            // Only consider front 180 degrees: 270° to 360° OR 0° to 90°
            bool is_front = (angle >= FRONT_RIGHT_MIN) || (angle <= FRONT_LEFT_MAX);
            if (!is_front) {
                continue;
            }

            float distance = msg->ranges[i];

            // Skip invalid readings
            if (std::isnan(distance) || std::isinf(distance)) {
                continue;
            }

            // Clamp to valid range
            distance = std::max(msg->range_min, std::min(distance, msg->range_max));

            // Track minimum front distance (near center at 0°)
            // Center is at 0° (or 2π which wraps around)
            if (angle <= CENTER_ANGLE_TOLERANCE || angle >= (2.0 * M_PI - CENTER_ANGLE_TOLERANCE)) {
                min_front_distance = std::min(min_front_distance, distance);
            }

            // Only consider directions with at least 0.35m clearance
            // Find safest (maximum distance) direction
            if (distance > max_distance && distance >= MIN_CLEARANCE) {
                max_distance = distance;
                safest_angle = angle;
            }
        }

        // Clear ahead - go straight!
        if (min_front_distance > MIN_CLEARANCE) {
            direction_ = 0.0;
            RCLCPP_DEBUG(this->get_logger(), "Clear ahead!");
            return;
        }

        // Obstacle ahead! Turn toward safest direction
        // Convert angle to steering command: 0° = straight, need to map to [-π, π] for control
        float steering_angle = safest_angle;
        if (steering_angle > M_PI) {
            steering_angle -= 2.0 * M_PI;
        }

        // Apply smoothing - don't change direction too abruptly
        direction_ = steering_angle;

        RCLCPP_INFO(this->get_logger(),
                    "Obstacle at %.2fm! Direction: %.2f rad (%.1f deg)",
                    min_front_distance, direction_, direction_ * 180.0 / M_PI);
    }

    void control_loop() {
        auto msg = geometry_msgs::msg::Twist();

        msg.linear.x = 0.1;              // Always move forward
        msg.angular.z = direction_ / 2;  // Turn toward safest direction

        cmd_vel_publisher_->publish(msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    float direction_;
};

// Global node pointer for signal handler
std::shared_ptr<Patrol> patrol_node;

// Signal handler for Ctrl+C
void signal_handler(int signum) {
    if (patrol_node) {
        patrol_node->stop_robot();
    }
    rclcpp::shutdown();
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    patrol_node = std::make_shared<Patrol>();

    // Register signal handler for Ctrl+C
    signal(SIGINT, signal_handler);

    rclcpp::spin(patrol_node);
    rclcpp::shutdown();
    return 0;
}