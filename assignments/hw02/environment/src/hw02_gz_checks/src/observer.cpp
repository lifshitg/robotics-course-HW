// Observe every physics step; short obstacle contacts stay latched until restart.
// The command relay lives in Gazebo, so a dead ROS helper cannot keep driving.
#include <chrono>
#include <cmath>
#include <mutex>
#include <algorithm>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/contacts.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>

namespace hw02 {
class Observer : public gz::sim::System, public gz::sim::ISystemPreUpdate,
                 public gz::sim::ISystemPostUpdate {
  gz::transport::Node node;
  gz::transport::Node::Publisher drive = node.Advertise<gz::msgs::Twist>("/model/hw02_robot/cmd_vel");
  gz::transport::Node::Publisher poses = node.Advertise<gz::msgs::Pose>("/hw02/robot_pose");
  gz::transport::Node::Publisher velocities = node.Advertise<gz::msgs::Twist>("/hw02/robot_velocity");
  gz::transport::Node::Publisher contacts = node.Advertise<gz::msgs::Contacts>("/hw02/contacts");
  gz::transport::Node::Publisher collision = node.Advertise<gz::msgs::Boolean>("/hw02/collision");
  std::mutex guard;
  gz::msgs::Twist command;
  std::chrono::steady_clock::time_point received{};
  std::chrono::steady_clock::duration last{};
  gz::math::Pose3d previous;
  bool havePrevious = false;
  gz::msgs::Contacts history;

 public:
  Observer() { node.Subscribe("/hw02/command", &Observer::Command, this); }
  void Command(const gz::msgs::Twist &msg) {
    std::lock_guard<std::mutex> lock(guard);
    command.Clear();
    if (std::isfinite(msg.linear().x()) && std::isfinite(msg.angular().z())) {
      command.mutable_linear()->set_x(std::clamp(msg.linear().x(), -0.2, 0.2));
      command.mutable_angular()->set_z(std::clamp(msg.angular().z(), -0.8, 0.8));
    }
    received = std::chrono::steady_clock::now();
  }
  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &) override {
    gz::msgs::Twist output;
    { std::lock_guard<std::mutex> lock(guard);
      if (!info.paused && history.contact_size() == 0 &&
          std::chrono::steady_clock::now()-received < std::chrono::milliseconds(500)) output = command;
    }
    drive.Publish(output);
  }
  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override {
    if (info.paused) return;
    // Inspect contacts BEFORE rate-limiting publication. Ignore supporting floor.
    ecm.Each<gz::sim::components::Collision, gz::sim::components::Name,
             gz::sim::components::ContactSensorData>(
      [&](const gz::sim::Entity &, const gz::sim::components::Collision *,
          const gz::sim::components::Name *name, const gz::sim::components::ContactSensorData *data) {
        if (name->Data().rfind("robot_", 0) != 0 || history.contact_size() > 0) return true;
        for (const auto &c : data->Data().contact()) {
          auto names = c.collision1().name()+" "+c.collision2().name();
          if (names.find("pillar_") != std::string::npos || names.find("wall_") != std::string::npos) {
            history.add_contact()->CopyFrom(c);
            break;
          }
        }
        return true;
      });
    if (info.simTime < last) { last = {}; havePrevious = false; }
    if (info.simTime-last < std::chrono::milliseconds(50)) return;
    auto base = ecm.EntityByComponents(gz::sim::components::Link(), gz::sim::components::Name("base_link"));
    if (base == gz::sim::kNullEntity) return;
    auto pose = gz::sim::worldPose(base, ecm);
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    gz::msgs::Pose message;
    message.mutable_header()->mutable_stamp()->set_sec(ns/1000000000);
    message.mutable_header()->mutable_stamp()->set_nsec(ns%1000000000);
    auto frame = message.mutable_header()->add_data(); frame->set_key("frame_id"); frame->add_value("map");
    message.mutable_position()->set_x(pose.Pos().X());
    message.mutable_position()->set_y(pose.Pos().Y());
    message.mutable_position()->set_z(pose.Pos().Z());
    message.mutable_orientation()->set_x(pose.Rot().X());
    message.mutable_orientation()->set_y(pose.Rot().Y());
    message.mutable_orientation()->set_z(pose.Rot().Z());
    message.mutable_orientation()->set_w(pose.Rot().W());
    poses.Publish(message);
    gz::msgs::Twist speed;
    if (havePrevious) {
      double dt = std::chrono::duration<double>(info.simTime-last).count();
      speed.mutable_linear()->set_x((pose.Pos().X()-previous.Pos().X())/dt);
      speed.mutable_linear()->set_y((pose.Pos().Y()-previous.Pos().Y())/dt);
      double angle = pose.Rot().Yaw()-previous.Rot().Yaw();
      speed.mutable_angular()->set_z(std::atan2(std::sin(angle), std::cos(angle))/dt);
    }
    velocities.Publish(speed);
    previous = pose; havePrevious = true; last = info.simTime;
    history.mutable_header()->mutable_stamp()->set_sec(ns/1000000000);
    history.mutable_header()->mutable_stamp()->set_nsec(ns%1000000000);
    contacts.Publish(history);
    gz::msgs::Boolean hit; hit.set_data(history.contact_size() > 0); collision.Publish(hit);
  }
};
}
GZ_ADD_PLUGIN(hw02::Observer, gz::sim::System, gz::sim::ISystemPreUpdate, gz::sim::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(hw02::Observer, "hw02::Observer")
