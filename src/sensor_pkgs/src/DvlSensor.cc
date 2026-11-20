#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <algorithm>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/msgs/twist.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/double.pb.h>

namespace bluerov2
{
class DvlSensor
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
      ignerr << "[DvlSensor] link [" << linkName << "] not found\n";
      return;
    }
    // Needed so the physics system fills in link velocities
    this->link.EnableVelocityChecks(_ecm, true);

    this->offset = _sdf->Get<ignition::math::Vector3d>(
        "offset", ignition::math::Vector3d::Zero).first; //Sensor mounting offset x,y,z relative to the link
    const auto rpy = _sdf->Get<ignition::math::Vector3d>(
        "rpy", ignition::math::Vector3d::Zero).first;   //Sensor mounting offset orientation relative to the link
    this->sensorRot = ignition::math::Quaterniond(rpy.X(), rpy.Y(), rpy.Z());

    // Noise parameters (same meaning as Gazebo's IMU <noise> block), in m/s
    this->mean = _sdf->Get<double>("mean", 0.0).first;
    this->stddev = _sdf->Get<double>("stddev", 0.005).first;
    this->biasMean = _sdf->Get<double>("bias_mean", 0.0).first;
    this->scaleStddev = _sdf->Get<double>("scale_stddev", 0.0005).first;
    this->scaleErr = this->scaleStddev * this->normal(this->gen);
    this->biasStddev = _sdf->Get<double>("bias_stddev", 0.002).first;
    this->dynBiasStddev = _sdf->Get<double>("dynamic_bias_stddev", 0.0001).first;
    this->corrTime = _sdf->Get<double>("dynamic_bias_correlation_time", 300.0).first;
    this->precision = _sdf->Get<double>("precision", 0.0).first;

    // Optional bottom-lock model: dropout outside [min_altitude, max_altitude]
    this->bottomZ = _sdf->Get<double>("bottom_z", -10.0).first;
    this->minAlt = _sdf->Get<double>("min_altitude", 0.05).first;
    this->maxAlt = _sdf->Get<double>("max_altitude", 0.0).first;  // 0 = disabled

    const double rate = _sdf->Get<double>("update_rate", 10.0).first;
    this->period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / rate));

    // Turn-on bias, drawn once per axis. Gazebo flips the sign at random.
    std::uniform_int_distribution<int> coin(0, 1);
    for (int i = 0; i < 3; ++i)
    {
      this->bias[i] = this->biasMean + this->biasStddev * this->normal(this->gen);
      if (coin(this->gen) == 0) this->bias[i] *= -1.0;
    }

    const auto topic = _sdf->Get<std::string>("topic", "dvl").first;
    const auto altTopic = _sdf->Get<std::string>("altitude_topic", topic + "/altitude").first;
    this->pub = this->node.Advertise<ignition::msgs::Twist>(topic);
    this->altPub = this->node.Advertise<ignition::msgs::Double>(altTopic);
    this->altStddev = _sdf->Get<double>("altitude_stddev", 0.01).first;
    this->ready = true;
  }

  void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                  const ignition::gazebo::EntityComponentManager &_ecm) override
  {
    if (!this->ready || _info.paused) return;
    if (_info.simTime - this->lastPub < this->period) return;

    double dt = std::chrono::duration<double>(_info.simTime - this->lastPub).count();
    if (this->firstUpdate)
    {
      dt = std::chrono::duration<double>(this->period).count();
      this->firstUpdate = false;
    }
    this->lastPub = _info.simTime;

    const auto vLin = this->link.WorldLinearVelocity(_ecm);
    const auto vAng = this->link.WorldAngularVelocity(_ecm);
    if (!vLin || !vAng) return;

    const auto pose = ignition::gazebo::worldPose(this->link.Entity(), _ecm);
    const auto r = pose.Rot().RotateVector(this->offset);

    // Bottom lock (straight-down altitude, ignores beam geometry)
 
    const double alt = (pose.Pos() + r).Z() - this->bottomZ;
    
    if (this->maxAlt > 0.0 && (alt < this->minAlt || alt > this->maxAlt)) return;

    // Velocity of the sensor point, expressed in the sensor frame
    const auto vWorld = *vLin + vAng->Cross(r);
    const auto q = pose.Rot() * this->sensorRot;
    const auto vSensor = q.RotateVectorReverse(vWorld);

    const double in[3] = {vSensor.X(), vSensor.Y(), vSensor.Z()};
    double out[3];
    for (int i = 0; i < 3; ++i) out[i] = this->Noisy(i, in[i], dt);

    ignition::msgs::Twist msg;
    msg.mutable_linear()->set_x(out[0]);
    msg.mutable_linear()->set_y(out[1]);
    msg.mutable_linear()->set_z(out[2]);
    this->pub.Publish(msg);

    ignition::msgs::Double altmsg;
    altmsg.set_data(std::max(0.0, alt + this->altStddev * this->normal(this->gen)));
    this->altPub.Publish(altmsg);
  }

private:
  double Noisy(int i, double v, double dt)
  {
    // Dynamic bias: first-order Gauss-Markov, same discretisation as Gazebo
    if (this->dynBiasStddev > 0.0 && this->corrTime > 0.0 && dt > 0.0)
    {
      const double beta = -1.0 / this->corrTime;
      const double sigmaD = std::sqrt(
          this->dynBiasStddev * this->dynBiasStddev / (2.0 * beta) *
          (std::exp(2.0 * beta * dt) - 1.0));
      const double phi = std::exp(beta * dt);
      this->bias[i] = phi * this->bias[i] + sigmaD * this->normal(this->gen);
    }

    double out = v * (1.0 + this->scaleErr) + this->mean + this->bias[i] +
                 this->stddev * this->normal(this->gen);
    if (this->precision > 0.0)
      out = std::round(out / this->precision) * this->precision;
    return out;
  }

  ignition::gazebo::Link link{ignition::gazebo::kNullEntity};
  ignition::math::Vector3d offset;
  ignition::math::Quaterniond sensorRot;
  ignition::transport::Node node;
  ignition::transport::Node::Publisher pub;
  ignition::transport::Node::Publisher altPub;
  double altStddev{0.01};

  double mean{0}, stddev{0.005};
  double biasMean{0}, biasStddev{0.002};
  double scaleStddev{0.0005}, scaleErr{0.0};
  double dynBiasStddev{0.0001}, corrTime{300.0};
  double precision{0};
  double bottomZ{-10.0}, minAlt{0.05}, maxAlt{0.0};
  double bias[3]{0, 0, 0};

  std::chrono::steady_clock::duration period{};
  std::chrono::steady_clock::duration lastPub{};
  //std::default_random_engine gen{std::random_device{}()};
  std::mt19937 gen{std::random_device{}()};
  std::normal_distribution<double> normal{0.0, 1.0};
  bool ready{false};
  bool firstUpdate{true};
};
}  // namespace bluerov2

IGNITION_ADD_PLUGIN(bluerov2::DvlSensor,
                    ignition::gazebo::System,
                    bluerov2::DvlSensor::ISystemConfigure,
                    bluerov2::DvlSensor::ISystemPostUpdate)
IGNITION_ADD_PLUGIN_ALIAS(bluerov2::DvlSensor, "bluerov2::DvlSensor")