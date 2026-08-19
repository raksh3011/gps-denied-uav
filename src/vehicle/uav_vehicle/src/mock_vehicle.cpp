// MockVehicle: consumes VehicleCommand from Safety and logs what would be
// sent to PX4. Rejects any command where `valid == false`. This proves the
// Safety -> PX4 Interface contract before the real uXRCE-DDS bridge and
// Gazebo/PX4 SITL integration exist.
//
// accepted_count/rejected_count are published (not just logged) so tests
// can observe the contract over topics rather than reaching into node
// internals.
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "uav_interfaces/msg/vehicle_command.hpp"

using uav_interfaces::msg::VehicleCommand;
using std_msgs::msg::UInt32;

class MockVehicle : public rclcpp::Node
{
public:
  MockVehicle() : Node("mock_vehicle")
  {
    rclcpp::QoS qos(rclcpp::KeepLast(5));
    qos.best_effort();

    cmd_sub_ = create_subscription<VehicleCommand>(
      "/safety/vehicle_command", qos,
      std::bind(&MockVehicle::on_cmd, this, std::placeholders::_1));

    accepted_pub_ = create_publisher<UInt32>("/vehicle/accepted_count", qos);
    rejected_pub_ = create_publisher<UInt32>("/vehicle/rejected_count", qos);
  }

private:
  void on_cmd(const VehicleCommand::SharedPtr msg)
  {
    if (!msg->valid) {
      ++rejected_;
      RCLCPP_WARN(
        get_logger(),
        "Rejected invalid VehicleCommand (mode=%u), holding last safe state.", msg->mode);
      UInt32 count;
      count.data = rejected_;
      rejected_pub_->publish(count);
      return;
    }
    ++accepted_;
    RCLCPP_INFO(
      get_logger(),
      "Would forward to PX4: mode=%u pos=(%.2f,%.2f,%.2f) yaw=%.2f",
      msg->mode, msg->position.x, msg->position.y, msg->position.z, msg->yaw);
    UInt32 count;
    count.data = accepted_;
    accepted_pub_->publish(count);
  }

  rclcpp::Subscription<VehicleCommand>::SharedPtr cmd_sub_;
  rclcpp::Publisher<UInt32>::SharedPtr accepted_pub_;
  rclcpp::Publisher<UInt32>::SharedPtr rejected_pub_;
  uint32_t accepted_{0};
  uint32_t rejected_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockVehicle>());
  rclcpp::shutdown();
  return 0;
}
