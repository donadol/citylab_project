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
  }

private:
  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {}

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  return 0;
}