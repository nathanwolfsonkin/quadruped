#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include "quadruped_gazebo/PlanarConstraint.hh"

#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>

GZ_ADD_PLUGIN(
    quadruped_gazebo::PlanarConstraint,
    gz::sim::System,
    quadruped_gazebo::PlanarConstraint::ISystemPostUpdate)

namespace quadruped_gazebo
{
    PlanarConstraint::PlanarConstraint()
    {
        gzdbg << "PlanarConstraint Plugin Loaded!" << std::endl;
    }

    void PlanarConstraint::PostUpdate(const gz::sim::UpdateInfo &_info,
                                      const gz::sim::EntityComponentManager &_ecm)
    {
        if (this->constrainedEntity == gz::sim::kNullEntity)
        {
            this->constrainedEntity = FindEntityByName("quadruped", _ecm);
            if (this->constrainedEntity != gz::sim::kNullEntity)
            {
                gzdbg << "Constrained entity set to: quadruped (ID " << this->constrainedEntity << ")" << std::endl;
            }
            else
            {
                gzdbg << "Entity 'quadruped' not found!" << std::endl;
            }
        }

        if (!_info.paused && _info.iterations % 100 == 0)
        {
            EnforcePlanarConstraint(this->constrainedEntity,
                                    const_cast<gz::sim::EntityComponentManager &>(_ecm));
        }
    }

    gz::sim::Entity PlanarConstraint::FindEntityByName(const std::string &name,
                                                       const gz::sim::EntityComponentManager &_ecm)
    {
        gz::sim::Entity foundEntity = gz::sim::kNullEntity;

        _ecm.Each<gz::sim::components::Name>(
            [&foundEntity, &name](const gz::sim::Entity &_entity,
                                  const gz::sim::components::Name *_nameComp) -> bool
            {
                if (_nameComp && _nameComp->Data() == name)
                {
                    foundEntity = _entity;
                    return false; // Stop iteration after finding the entity
                }
                return true; // Continue searching
            });

        return foundEntity;
    }

    void PlanarConstraint::EnforcePlanarConstraint(const gz::sim::Entity &_entity,
                                                   gz::sim::EntityComponentManager &_ecm)
    {
        if (_entity == gz::sim::kNullEntity)
        {
            return;
        }

        gzdbg << "Entity ID: " << _entity << std::endl;

        // Get the linear and angular velocity components
        auto linearVelComp = _ecm.Component<gz::sim::components::LinearVelocity>(_entity);
        auto angularVelComp = _ecm.Component<gz::sim::components::AngularVelocity>(_entity);

        if (linearVelComp)
        {
            auto linearVel = linearVelComp->Data();
            gzdbg << "linearVel: " << linearVel << std::endl;
            linearVel.Y() = 0.0; // Fix the Y component of linear velocity to 0
            _ecm.SetComponentData<gz::sim::components::LinearVelocity>(_entity, linearVel);
        }
        else
        {
            // gzdbg << "linearVelComp is null" << std::endl;
        }

        if (angularVelComp)
        {
            auto angularVel = angularVelComp->Data();
            gzdbg << "angularVel: " << angularVel << std::endl;
            _ecm.SetComponentData<gz::sim::components::AngularVelocity>(_entity, angularVel);
        }
        else
        {
            // gzdbg << "angularVelComp is null" << std::endl;
        }
    }
} // namespace quadruped_gazebo
