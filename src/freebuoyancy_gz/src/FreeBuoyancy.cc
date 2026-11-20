#include "freebuoyancy_gz/FreeBuoyancy.hh"

#include <sstream>
#include <iostream>

#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Inertial.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/components/Gravity.hh>
#include <ignition/gazebo/Util.hh>

using namespace freebuoyancy;
using namespace ignition;
using namespace ignition::gazebo;

//////////////////////////////////////////////////
void FreeBuoyancy::ReadVector3(const std::string &_str, math::Vector3d &_vec)
{
  std::stringstream ss(_str);
  double xyz[3] = {0, 0, 0};
  for (unsigned int i = 0; i < 3; ++i)
    ss >> xyz[i];
  _vec.Set(xyz[0], xyz[1], xyz[2]);
}

//////////////////////////////////////////////////
void FreeBuoyancy::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager & /*_eventMgr*/)
{
  this->model_ = Model(_entity);
  if (!this->model_.Valid(_ecm))
  {
    std::cerr << "[FreeBuoyancy] Must be attached to a <model>, skipping."
               << std::endl;
    return;
  }

  if (_sdf->HasElement("surface"))
  {
    this->hasSurface_ = true;
    math::Vector3d surfacePoint;
    ReadVector3(_sdf->Get<std::string>("surface"), surfacePoint);

    // Gravity is a world-level component; the world entity is this
    // model's parent-of-parent in the ECS tree, so just query it by type.
    Entity worldEnt = worldEntity(_ecm);
    math::Vector3d gravity{0, 0, -9.81};
    if (worldEnt != kNullEntity)
    {
      auto gravComp = _ecm.Component<components::Gravity>(worldEnt);
      if (gravComp)
        gravity = gravComp->Data();
    }
    math::Vector3d gravityDir = gravity.Normalized();
    this->surfacePlane_.Set(gravityDir.X(), gravityDir.Y(), gravityDir.Z(),
        gravityDir.Dot(surfacePoint));
  }

  // Per-link buoyancy tags live directly under this plugin's own SDF,
  // e.g.:
  //   <plugin filename="libfreebuoyancy-system.so"
  //           name="freebuoyancy::FreeBuoyancy">
  //     <surface>0 0 0</surface>
  //     <link name="buoyancy_link">
  //       <origin xyz="0 0 0.05"/>
  //       <compensation>1.05</compensation>
  //       <limit radius="0.1"/>
  //       <damping xyz="5 5 5" rpy="1 1 1"/>
  //     </link>
  //   </plugin>
  // This replaces the classic version's TinyXML re-parse of the whole
  // URDF -- gz-sim hands the plugin's own SDF block straight to us.
  if (_sdf->HasElement("link"))
  {
    for (auto linkElem = _sdf->FindElement("link"); linkElem;
         linkElem = linkElem->GetNextElement("link"))
    {
      this->ParseLinkElement(linkElem, _ecm);
    }
  }

  if (this->buoyantLinks_.empty())
  {
    std::cerr << "[FreeBuoyancy] No <link> buoyancy tags found for model "
               << this->model_.Name(_ecm) << std::endl;
  }
}

//////////////////////////////////////////////////
void FreeBuoyancy::ParseLinkElement(
    const std::shared_ptr<const sdf::Element> &_linkElem,
    EntityComponentManager &_ecm)
{
  if (!_linkElem->HasAttribute("name"))
    return;

  BuoyantLink buoy;
  buoy.linkName = _linkElem->Get<std::string>("name");
  buoy.linkEntity = this->model_.LinkByName(_ecm, buoy.linkName);

  if (buoy.linkEntity == kNullEntity)
  {
    std::cerr << "[FreeBuoyancy] Link '" << buoy.linkName
               << "' not found on model " << this->model_.Name(_ecm)
               << ", skipping." << std::endl;
    return;
  }

  Link link(buoy.linkEntity);
  // Velocity components are not populated by default -- must opt in,
  // or WorldLinearVelocity()/WorldAngularVelocity() will return nullopt
  // every tick. This is the single most common gotcha porting from
  // classic, where link->WorldLinearVel() always "just worked".
  link.EnableVelocityChecks(_ecm, true);

  double mass = 1.0;
  if (auto inertial = _ecm.Component<components::Inertial>(buoy.linkEntity))
    mass = inertial->Data().MassMatrix().Mass();

  buoy.buoyancyCenter = math::Vector3d::Zero;
  buoy.linearDamping = buoy.angularDamping =
      5.0 * math::Vector3d::One * mass;
  buoy.limit = 0.1;

  double compensation = 0.0;

  if (_linkElem->HasElement("origin"))
  {
    auto originElem = _linkElem->FindElement("origin");
    if (originElem->HasAttribute("xyz"))
      ReadVector3(originElem->Get<std::string>("xyz"), buoy.buoyancyCenter);
  }
  if (_linkElem->HasElement("compensation"))
    compensation = _linkElem->Get<double>("compensation");
  if (_linkElem->HasElement("limit"))
  {
    auto limitElem = _linkElem->FindElement("limit");
    if (limitElem->HasAttribute("radius"))
      buoy.limit = limitElem->Get<double>("radius");
  }
  if (_linkElem->HasElement("damping"))
  {
    auto dampingElem = _linkElem->FindElement("damping");
    if (dampingElem->HasAttribute("xyz"))
      ReadVector3(dampingElem->Get<std::string>("xyz"), buoy.linearDamping);
    if (dampingElem->HasAttribute("rpy"))
      ReadVector3(dampingElem->Get<std::string>("rpy"), buoy.angularDamping);
  }

  // gravity magnitude only -- direction is baked into the sign here to
  // match the classic plugin's convention (force points opposite gravity)
  buoy.buoyantForce = math::Vector3d(0, 0, compensation * mass * 9.81);

  this->buoyantLinks_.push_back(buoy);
}

//////////////////////////////////////////////////
void FreeBuoyancy::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  if (_info.paused || this->buoyantLinks_.empty())
    return;

  for (const auto &buoy : this->buoyantLinks_)
  {
    Link link(buoy.linkEntity);
    auto worldPoseOpt = link.WorldPose(_ecm);
    auto linVelOpt = link.WorldLinearVelocity(_ecm);
    auto angVelOpt = link.WorldAngularVelocity(_ecm);
    if (!worldPoseOpt || !linVelOpt || !angVelOpt)
      continue;

    const math::Pose3d &worldPose = *worldPoseOpt;
    math::Vector3d cobPosition =
        worldPose.Pos() + worldPose.Rot().RotateVector(buoy.buoyancyCenter);

    math::Vector3d actualForce = buoy.buoyantForce;

    if (this->hasSurface_)
    {
      double signedDistance = this->surfacePlane_.W()
          - this->surfacePlane_.X() * cobPosition.X()
          - this->surfacePlane_.Y() * cobPosition.Y()
          - this->surfacePlane_.Z() * cobPosition.Z();

      if (signedDistance > buoy.limit)
      {
        // fully out of the water -- no buoyant force at all.
        // (the classic version used `return` here by mistake, which
        // silently skipped every remaining link once one broke surface;
        // `continue` for this link only is the intended fix.)
        continue;
      }
      else if (signedDistance > -buoy.limit)
      {
        actualForce *= cos(M_PI / 4.0 * (signedDistance / buoy.limit + 1));
      }
    }

    // Linear damping, computed in the link frame then applied at the
    // center of buoyancy. AddWorldForce's position argument is in the
    // link's local frame, so no manual world-frame offset math needed.
    math::Vector3d velDiff =
        worldPose.Rot().RotateVectorReverse(*linVelOpt - this->fluidVelocity_);
    velDiff.X(velDiff.X() * std::fabs(velDiff.X()));
    velDiff.Y(velDiff.Y() * std::fabs(velDiff.Y()));
    velDiff.Z(velDiff.Z() * std::fabs(velDiff.Z()));
    actualForce -= worldPose.Rot().RotateVector(buoy.linearDamping * velDiff);

    link.AddWorldForce(_ecm, actualForce, buoy.buoyancyCenter);

    // Angular damping: compute in the link (relative) frame, then
    // rotate into world frame since AddWorldWrench only takes wrenches
    // expressed in the world frame.
    math::Vector3d angVelLocal =
        worldPose.Rot().RotateVectorReverse(*angVelOpt);
    angVelLocal.X(angVelLocal.X() * std::fabs(angVelLocal.X()));
    angVelLocal.Y(angVelLocal.Y() * std::fabs(angVelLocal.Y()));
    angVelLocal.Z(angVelLocal.Z() * std::fabs(angVelLocal.Z()));
    math::Vector3d torqueWorld =
        worldPose.Rot().RotateVector(-buoy.angularDamping * angVelLocal);

    link.AddWorldWrench(_ecm, math::Vector3d::Zero, torqueWorld);
  }
}

IGNITION_ADD_PLUGIN(
    freebuoyancy::FreeBuoyancy,
    ignition::gazebo::System,
    freebuoyancy::FreeBuoyancy::ISystemConfigure,
    freebuoyancy::FreeBuoyancy::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(freebuoyancy::FreeBuoyancy,
    "freebuoyancy::FreeBuoyancy")
