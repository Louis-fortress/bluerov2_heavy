#include <chrono>
#include <memory>
#include <random>
#include <string>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/msgs/fluid_pressure.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

namespace bluerov2
{
class PressureSensor
  : public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPostUpdate
{
public:
  void Configure(const ignition::gazebo::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 ignition::gazebo::EntityComponentManager &_ecm,
                 ignition::gazebo::EventManager &) override
  {
    ignition::gazebo::Model model(_entity);
    const auto linkName = _sdf->Get<std::string>("link", "base_link").first;
    this->link = ignition::gazebo::Link(model.LinkByName(_ecm, linkName));
    if (this->link.Entity() == ignition::gazebo::kNullEntity)
    {
      ignerr << "[PressureSensor] link [" << linkName << "] not found\n";
      return;
    }

    this->offset = _sdf->Get<ignition::math::Vector3d>(
        "offset", ignition::math::Vector3d::Zero).first;
    this->frameId = _sdf->Get<std::string>("frame_id", linkName).first;
    this->surfaceZ = _sdf->Get<double>("surface_z", 0.0).first;
    this->patm = _sdf->Get<double>("atm_pressure", 101325.0).first;
    this->rho = _sdf->Get<double>("fluid_density", 1000.0).first;
    this->stddev = _sdf->Get<double>("stddev", 20.0).first;  // Pa
    const double rate = _sdf->Get<double>("update_rate", 20.0).first;
    this->period = std::chrono::duration_cast<std::chrono::steady_clock::duration>
        (std::chrono::duration<double>(1.0 / rate));

    const auto topic = _sdf->Get<std::string>("topic", "pressure").first;
    this->pub = this->node.Advertise<ignition::msgs::FluidPressure>(topic);
    this->noise = std::normal_distribution<double>(0.0, this->stddev);
    this->ready = true;
  }

  void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                  const ignition::gazebo::EntityComponentManager &_ecm) override
  {
    if (!this->ready || _info.paused) return;
    if (_info.simTime - this->lastPub < this->period) return;
    this->lastPub = _info.simTime;

    const auto pose = ignition::gazebo::worldPose(this->link.Entity(), _ecm);
    const auto sensorPos = pose.Pos() + pose.Rot().RotateVector(this->offset);
    const double depth = this->surfaceZ - sensorPos.Z();

    const double p = this->patm + this->rho * 9.80665 * depth
                     + this->noise(this->gen);

    ignition::msgs::FluidPressure msg;
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(_info.simTime);
    const auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(
        _info.simTime - sec);
    msg.mutable_header()->mutable_stamp()->set_sec(sec.count());
    msg.mutable_header()->mutable_stamp()->set_nsec(nsec.count());
    
    auto *frame = msg.mutable_header()->add_data();
    frame->set_key("frame_id");
    frame->add_value(this->frameId);
    msg.set_pressure(p);
    msg.set_variance(this->stddev * this->stddev);
    this->pub.Publish(msg);
  }

private:
  ignition::gazebo::Link link{ignition::gazebo::kNullEntity};
  ignition::math::Vector3d offset;
  ignition::transport::Node node;
  ignition::transport::Node::Publisher pub;
  std::string frameId;
  double surfaceZ{0}, patm{101325}, rho{1000}, stddev{20};
  std::chrono::steady_clock::duration period{};
  std::chrono::steady_clock::duration lastPub{};
  std::mt19937 gen{std::random_device{}()};
  std::normal_distribution<double> noise;
  bool ready{false};
};
}  // namespace bluerov2

IGNITION_ADD_PLUGIN(bluerov2::PressureSensor,
                    ignition::gazebo::System,
                    bluerov2::PressureSensor::ISystemConfigure,
                    bluerov2::PressureSensor::ISystemPostUpdate)
IGNITION_ADD_PLUGIN_ALIAS(bluerov2::PressureSensor, "bluerov2::PressureSensor")