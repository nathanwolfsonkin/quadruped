#ifndef QUADRUPED_GAZEBO__PLANAR_CONSTRAINT_HH_
#define QUADRUPED_GAZEBO__PLANAR_CONSTRAINT_HH_

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Name.hh>

namespace quadruped_gazebo
{
  class PlanarConstraint:
    public gz::sim::System,
    public gz::sim::ISystemPostUpdate
  {
    public: 
      PlanarConstraint();

      void PostUpdate(const gz::sim::UpdateInfo &_info,
                      const gz::sim::EntityComponentManager &_ecm) override;

    private:
      void EnforcePlanarConstraint(const gz::sim::Entity &_entity,
                                    gz::sim::EntityComponentManager &_ecm);

      gz::sim::Entity FindEntityByName(const std::string &name,
                                       const gz::sim::EntityComponentManager &_ecm);

      gz::sim::Entity constrainedEntity{gz::sim::kNullEntity};  // Entity to be constrained
  };
}

#endif  // QUADRUPED_GAZEBO__PLANAR_CONSTRAINT_HH_
