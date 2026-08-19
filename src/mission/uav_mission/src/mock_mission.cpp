// MockMission: publishes a fixed, latched Mission on startup and logs
// PlannerStatus, so the rest of the pipeline can be exercised without a
// real mission file loader or operator input UI.
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "uav_interfaces/msg/mission.hpp"
#include "uav_interfaces/msg/waypoint.hpp"
#include "uav_interfaces/msg/planner_status.hpp"

using namespace std::chrono_literals;
using uav_interfaces::msg::Mission;
using uav_interfaces::msg::Waypoint;
using uav_interfaces::msg::PlannerStatus;

class MockMission : public rclcpp::Node
{
public:
  MockMission() : Node("mock_mission")
  {
    rclcpp::QoS mission_qos(rclcpp::KeepLast(1));
    mission_qos.reliable().transient_local();
    rclcpp::QoS sensor_qos(rclcpp::KeepLast(5));
    sensor_qos.best_effort();

    mission_pub_ = create_publisher<Mission>("/mission/current", mission_qos);
    status_sub_ = create_subscription<PlannerStatus>(
      "/planning/status", sensor_qos,
      [this](const PlannerStatus::SharedPtr msg) {
        RCLCPP_DEBUG(
          get_logger(), "planner state=%u progress=%.2f", msg->state, msg->progress);
      });

    publish_mission();
    timer_ = create_wall_timer(1s, std::bind(&MockMission::publish_mission, this));
  }

private:
  void publish_mission()
  {
    Mission m;
    m.header.stamp = this->now();
    m.header.frame_id = "map";
    m.mission_id = "golden-scenario-01";

    Waypoint wp;
    wp.position.x = 10.0;
    wp.position.y = 0.0;
    wp.position.z = 3.0;
    wp.acceptance_radius = 0.5F;
    wp.waypoint_type = Waypoint::TYPE_TARGET;
    m.waypoints.push_back(wp);

    m.max_speed = 3.0F;
    m.boundary_radius = 50.0F;
    m.min_altitude = 1.0F;
    m.max_altitude = 20.0F;
    mission_pub_->publish(m);
  }

  rclcpp::Publisher<Mission>::SharedPtr mission_pub_;
  rclcpp::Subscription<PlannerStatus>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockMission>());
  rclcpp::shutdown();
  return 0;
}
