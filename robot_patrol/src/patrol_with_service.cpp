#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_patrol_msg/srv/get_direction.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

class PatrolWithService : public rclcpp::Node {
   public:
    PatrolWithService() : Node("patrol_with_service_node"), direction_("") {
        laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/fastbot_1/scan", 10,
            std::bind(&PatrolWithService::laser_scan_callback, this, _1));

        cmd_vel_publisher_ =
            this->create_publisher<geometry_msgs::msg::Twist>("/fastbot_1/cmd_vel", 10);

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PatrolWithService::control_loop, this));

        std::string name_service = "/direction_service";
        client_ = this->create_client<robot_patrol_msg::srv::GetDirection>(name_service);

        // Wait for the service to be available (checks every second)
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(),
                             "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(),
                        "Service %s not available, waiting again...",
                        name_service.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "PatrolWithService node initialized");
    }

    void stop_robot() {
        auto stop_msg = geometry_msgs::msg::Twist();
        // All velocities default to 0.0
        cmd_vel_publisher_->publish(stop_msg);
        RCLCPP_INFO(this->get_logger(), "Robot stopped - shutting down");
    }

   private:
    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        auto request = std::make_shared<robot_patrol_msg::srv::GetDirection::Request>();
        request->laser_data = *msg;

        // Send the request asynchronously
        auto result = client_->async_send_request(
            request,
            [this](rclcpp::Client<robot_patrol_msg::srv::GetDirection>::SharedFuture
                       future_response) {
                auto response = future_response.get();
                RCLCPP_INFO(this->get_logger(), "Direction: %s",
                            response->direction.c_str());

                direction_ = response->direction;
            });
    }

    void control_loop() {
        auto msg = geometry_msgs::msg::Twist();

        msg.linear.x = 0.1;  // Always move forward

        if (direction_ == "forward") {
            msg.angular.z = 0.0;
        } else if (direction_ == "left") {
            msg.angular.z = 0.5;
        } else if (direction_ == "right") {
            msg.angular.z = -0.5;
        } else {
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
        }

        RCLCPP_INFO(this->get_logger(),
                    "Direction: %s → cmd_vel linear: %.2f angular: %.2f",
                    direction.c_str(), cmd.linear.x, cmd.angular.z);

        cmd_vel_publisher_->publish(msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Client<robot_patrol_msg::srv::GetDirection>::SharedPtr client_;
    string direction_;
};

// Global node pointer for signal handler
std::shared_ptr<PatrolWithService> patrol_node;

// Signal handler for Ctrl+C
void signal_handler(int signum) {
    if (patrol_node) {
        patrol_node->stop_robot();
    }
    rclcpp::shutdown();
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    patrol_node = std::make_shared<PatrolWithService>();

    // Register signal handler for Ctrl+C
    signal(SIGINT, signal_handler);

    rclcpp::spin(patrol_node);
    rclcpp::shutdown();
    return 0;
}