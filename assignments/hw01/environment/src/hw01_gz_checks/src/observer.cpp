// Read physics state directly. Publish empty samples too, so silence fails checks.
#include <chrono>
#include <map>
#include <string>
#include <gz/msgs/contacts.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/transport/Node.hh>

namespace hw01 {
class Observer : public gz::sim::System, public gz::sim::ISystemPostUpdate {
  gz::transport::Node node;
  gz::transport::Node::Publisher platform = node.Advertise<gz::msgs::Contacts>("/hw01/platform_contacts");
  gz::transport::Node::Publisher marker = node.Advertise<gz::msgs::Contacts>("/hw01/marker_contacts");
  gz::transport::Node::Publisher tip = node.Advertise<gz::msgs::Pose>("/hw01/measured_tip");
  std::chrono::steady_clock::duration last{};

 public:
  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override {
    if (info.paused || info.simTime - last < std::chrono::milliseconds(100)) return;
    last = info.simTime;
    std::map<std::string, gz::msgs::Contacts> samples;
    // ContactSensorData is supplied by Gazebo Physics for sensor-enabled collisions.
    ecm.Each<gz::sim::components::Collision, gz::sim::components::Name,
             gz::sim::components::ParentEntity, gz::sim::components::ContactSensorData>(
      [&](const gz::sim::Entity &, const gz::sim::components::Collision *,
          const gz::sim::components::Name *name, const gz::sim::components::ParentEntity *,
          const gz::sim::components::ContactSensorData *data) {
        if (name->Data() == "platform_collision") samples["platform"] = data->Data();
        if (name->Data() == "marker_collision") samples["marker"] = data->Data();
        return true;
      });
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    for (auto &entry : samples) {
      auto stamp = entry.second.mutable_header()->mutable_stamp();
      stamp->set_sec(ns / 1000000000); stamp->set_nsec(ns % 1000000000);
      if (entry.first == "platform") platform.Publish(entry.second);
      else marker.Publish(entry.second);
    }
    auto base = ecm.EntityByComponents(gz::sim::components::Link(), gz::sim::components::Name("base_link"));
    auto link = ecm.EntityByComponents(gz::sim::components::Link(), gz::sim::components::Name("link2"));
    if (base == gz::sim::kNullEntity || link == gz::sim::kNullEntity) return;
    auto pose = gz::sim::worldPose(base, ecm).Inverse() * gz::sim::worldPose(link, ecm) *
                gz::math::Pose3d(0.25, 0, -0.08, 0, 0, 0);
    gz::msgs::Pose message;
    message.set_name("tool0");
    message.mutable_header()->mutable_stamp()->set_sec(ns / 1000000000);
    message.mutable_header()->mutable_stamp()->set_nsec(ns % 1000000000);
    auto frame = message.mutable_header()->add_data();
    frame->set_key("frame_id"); frame->add_value("base_link");
    message.mutable_position()->set_x(pose.Pos().X());
    message.mutable_position()->set_y(pose.Pos().Y());
    message.mutable_position()->set_z(pose.Pos().Z());
    message.mutable_orientation()->set_x(pose.Rot().X());
    message.mutable_orientation()->set_y(pose.Rot().Y());
    message.mutable_orientation()->set_z(pose.Rot().Z());
    message.mutable_orientation()->set_w(pose.Rot().W());
    tip.Publish(message);
  }
};
}
GZ_ADD_PLUGIN(hw01::Observer, gz::sim::System, gz::sim::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(hw01::Observer, "hw01::Observer")
