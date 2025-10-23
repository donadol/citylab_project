#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

class Patrol : public rclcpp::Node {
   public:
    Patrol() : Node("patrol_node") {
        laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/fastbot_1/scan", 10,
            std::bind(&Patrol::laser_scan_callback, this, _1));

        cmd_vel_publisher_ =
            this->create_publisher<geometry_msgs::msg::Twist>("/fastbot_1/cmd_vel", 10);

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Patrol::control_loop, this));
    }

   private:
    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        float max_distance = 0.0;
        float safest_angle = 0.0;

        for (size_t i = 0; i < msg->ranges.size(); i++) {
            float distance = msg->ranges[i];
            float angle = msg->angle_min + (i * msg->angle_increment);

            // Skip invalid readings
            if (distance < msg->range_min || distance > msg->range_max ||
                std::isnan(distance) || std::isinf(distance)) {
                continue;
            }

            // Only process front 180° rays
            if (angle <= -M_PI / 2 || angle >= M_PI / 2) {
                continue;
            }

            // Find maximum distance direction
            if (distance > max_distance) {
                max_distance = distance;
                safest_angle = angle;
            }
        }

        direction_ = safest_angle;
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
    float direction_ = 0.0;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Patrol>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}