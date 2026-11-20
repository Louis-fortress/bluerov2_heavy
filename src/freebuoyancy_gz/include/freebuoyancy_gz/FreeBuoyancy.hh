#ifndef FREEBUOYANCY_GZ_FREEBUOYANCY_HH_
#define FREEBUOYANCY_GZ_FREEBUOYANCY_HH_

#include <string>
#include <vector>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include <sdf/Element.hh>

namespace freebuoyancy
{
  /// Per-link buoyancy parameters, resolved once in Configure() from the
  /// plugin's own <link> tags (see the example SDF at the bottom of the
  /// .cc file for the expected schema).
  struct BuoyantLink
  {
    /// Entity of the link this force is applied to.
    ignition::gazebo::Entity linkEntity{ignition::gazebo::kNullEntity};

    /// Name of the link, kept for logging.
    std::string linkName;

    /// Center of buoyancy, expressed in the link's own frame.
    ignition::math::Vector3d buoyancyCenter{0, 0, 0};

    /// Theoretical buoyant force (world frame, N) at full submersion,
    /// computed once from the compensation ratio and link mass.
    ignition::math::Vector3d buoyantForce{0, 0, 0};

    /// Linear damping coefficients, link frame.
    ignition::math::Vector3d linearDamping{0, 0, 0};

    /// Angular damping coefficients, link frame.
    ignition::math::Vector3d angularDamping{0, 0, 0};

    /// Half-thickness (m) of the surface transition band used to taper
    /// the buoyant force as the link crosses the free surface.
    double limit{0.1};
  };

  /// Gazebo Fortress (gz-sim / ignition-gazebo6) port of the classic
  /// freebuoyancy_gazebo model plugin. Attach this directly to the
  /// vehicle model's SDF/URDF <model> tag -- there is no more world-level
  /// scanning for new/removed models, so each vehicle owns one instance.
  class FreeBuoyancy:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate
  {
    public: FreeBuoyancy() = default;

    public: void Configure(
        const ignition::gazebo::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        ignition::gazebo::EntityComponentManager &_ecm,
        ignition::gazebo::EventManager &_eventMgr) override;

    public: void PreUpdate(
        const ignition::gazebo::UpdateInfo &_info,
        ignition::gazebo::EntityComponentManager &_ecm) override;

    /// Parses one <link> block of this plugin's own SDF into a BuoyantLink.
    private: void ParseLinkElement(
        const std::shared_ptr<const sdf::Element> &_linkElem,
        ignition::gazebo::EntityComponentManager &_ecm);

    private: static void ReadVector3(
        const std::string &_str, ignition::math::Vector3d &_vec);

    /// The vehicle model this plugin is attached to.
    private: ignition::gazebo::Model model_{ignition::gazebo::kNullEntity};

    /// True once Configure() has resolved link entities and enabled
    /// velocity checks -- link entities are not guaranteed valid until
    /// after the first PreUpdate, so resolution happens lazily.
    private: bool linksResolved_{false};

    /// Optional world-current velocity; left at zero unless you wire up
    /// a topic subscription (see notes in the .cc file).
    private: ignition::math::Vector3d fluidVelocity_{0, 0, 0};

    private: bool hasSurface_{false};

    /// Surface plane in world frame: plane.X()*x + plane.Y()*y +
    /// plane.Z()*z == plane.W(). Defaults to the world XY-plane at Z=0.
    private: ignition::math::Vector4d surfacePlane_{0, 0, 1, 0};

    private: std::vector<BuoyantLink> buoyantLinks_;
  };
}  // namespace freebuoyancy

#endif  // FREEBUOYANCY_GZ_FREEBUOYANCY_HH_
