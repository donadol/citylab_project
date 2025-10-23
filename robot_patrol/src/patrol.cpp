#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

class Patrol : public rclcpp::Node {
   public:
    Patrol() : Node("patrol_node"), direction_(0.0) {
        laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/fastbot_1/scan", 10,
            std::bind(&Patrol::laser_scan_callback, this, _1));

        cmd_vel_publisher_ =
            this->create_publisher<geometry_msgs::msg::Twist>("/fastbot_1/cmd_vel", 10);

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Patrol::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Patrol node initialized");
    }

   private:
    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // We need front 180 degrees: from -π/2 to +π/2
        const double FRONT_MIN_ANGLE = -M_PI / 2.0;  // -90 degrees
        const double FRONT_MAX_ANGLE = M_PI / 2.0;   // +90 degrees
        const double MIN_CLEARANCE = 0.35;           // 35 cm obstacle detection threshold
        const double CENTER_ANGLE_TOLERANCE = 0.3;   // ~11 degrees - center front zone

        float max_distance = 0.0;
        float safest_angle = 0.0;
        float min_front_distance = MIN_CLEARANCE + 0.1;  // Initialize above clearance

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float angle = msg->angle_min + (i * msg->angle_increment);

            // Only consider front 180 degrees
            if (angle < FRONT_MIN_ANGLE || angle > FRONT_MAX_ANGLE) {
                continue;
            }

            float distance = msg->ranges[i];

            // Skip invalid readings
            if (std::isnan(distance) || std::isinf(distance)) {
                continue;
            }

            // Clamp to valid range
            distance = std::max(msg->range_min, std::min(distance, msg->range_max));

            // Check distance directly in front (for obstacle detection)
            if (std::abs(angle) < CENTER_ANGLE_TOLERANCE) {
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
            RCLCPP_WARN(this->get_logger(), "Clear ahead!");
            return;
        }

        // Obstacle ahead! Turn toward safest direction
        direction_ = safest_angle;
        RCLCPP_INFO(this->get_logger(),
                    "Obstacle at %.2fm! Turning to angle: %.2f rad (%.1f deg)",
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

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Patrol>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}