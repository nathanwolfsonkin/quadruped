#ifndef QUADRUPED_GAZEBO__JOINT_TORQUE_PUBLISHER_HH_
#define QUADRUPED_GAZEBO__JOINT_TORQUE_PUBLISHER_HH_

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/config.hh>
#include <gz/sim/EventManager.hh>
#include <vector>
#include <unordered_map>

namespace quadruped_gazebo
{
    // This is the main plugin's class. It must inherit from System and at least
    // one other interface.
    // Here we use `ISystemPostUpdate`, which is used to get results after
    // physics runs. The opposite of that, `ISystemPreUpdate`, would be used by
    // plugins that want to send commands.
    class JointTorquePublisher : public gz::sim::System,
                                 public gz::sim::ISystemConfigure,
                                 public gz::sim::ISystemPostUpdate
    {
        // Plugins inheriting ISystemPostUpdate must implement the PostUpdate
        // callback. This is called at every simulation iteration after the physics
        // updates the world. The _info variable provides information such as time,
        // while the _ecm provides an interface to all entities and components in
        // simulation.
    public:
        JointTorquePublisher();

        // Called once when the plugin is loaded
        void Configure(const gz::sim::Entity &_entity,
                       const std::shared_ptr<const sdf::Element> &_element,
                       gz::sim::EntityComponentManager &_ecm,
                       gz::sim::EventManager &_eventManager) override;

        // Runs every simulation update step
        void PostUpdate(const gz::sim::UpdateInfo &_info,
                        const gz::sim::EntityComponentManager &_ecm) override;

    private:
        std::vector<gz::sim::Entity> jointEntities;
        std::unordered_map<gz::sim::Entity, double> jointTorques;
    };
}
#endif
