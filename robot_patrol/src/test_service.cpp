#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

class TestService : public rclcpp::Node {
   public:
    TestService() : Node("test_service_node") {
        laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/fastbot_1/scan", 10,
            std::bind(&TestService::laser_scan_callback, this, _1));

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

        RCLCPP_INFO(this->get_logger(), "TestService node initialized");
    }

   private:
    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        auto request = std::make_shared<robot_patrol_msg::srv::GetDirection::Request>();
        request->laser_data = msg;

        // Send the request asynchronously
        auto result_future = client_->async_send_request(request);

        // Wait for the result
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                               result_future) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            auto response = result_future.get();
            // Log the service response
            RCLCPP_INFO(this->get_logger(), "Direction: %s", response->direction);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Client<robot_patrol_msg::srv::GetDirection>::SharedPtr client_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<TestService> node = std::make_shared<TestService>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}