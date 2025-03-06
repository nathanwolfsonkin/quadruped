// Include gz headers
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/transport/Node.hh>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/joint-position-controller-system/Export.hh>

// Don't forget to include the plugin's header.
#include <torque_test/JointTorquePublisher.hh>

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
GZ_ADD_PLUGIN(
    quadruped_gazebo::JointTorquePublisher,
    gz::sim::System,
    quadruped_gazebo::JointTorquePublisher::ISystemPostUpdate,
    quadruped_gazebo::JointTorquePublisher::ISystemConfigure)

namespace quadruped_gazebo
{
    JointTorquePublisher::JointTorquePublisher()
    {
        gzdbg << "JointTorquePublisher Plugin Loaded!" << std::endl;
    }

    void JointTorquePublisher::Configure(const gz::sim::Entity &_entity,
                                         const std::shared_ptr<const sdf::Element> &_sdf,
                                         gz::sim::EntityComponentManager &_ecm,
                                         gz::sim::EventManager &)
    {
        gzdbg << "JointTorquePublisher::Configure called!" << std::endl;

        // Store joint entities once
        _ecm.Each<gz::sim::components::Joint, gz::sim::components::Name>(
            [&](const gz::sim::Entity &_jointEntity,
                const gz::sim::components::Joint *,
                const gz::sim::components::Name *_name) -> bool
            {
                gzdbg << "Found joint: " << _name->Data() << " (Entity ID: " << _jointEntity << ")" << std::endl;
                this->jointEntities.push_back(_jointEntity);
                return true;
            });

        gzdbg << "Total Joints Stored: " << this->jointEntities.size() << std::endl;
    }

    void JointTorquePublisher::PostUpdate(const gz::sim::UpdateInfo &_info,
                                          const gz::sim::EntityComponentManager &_ecm)
    {
        // Iterate over all known joints and retrieve applied torque
        for (auto jointEntity : this->jointEntities)
        {
            double appliedTorque1 = 0.0;
            double appliedTorque2 = 0.0;
            double appliedTorque3 = 0.0;
            double appliedTorque4 = 0.0;
            double appliedTorque5 = 0.0;
            double appliedTorque6 = 0.0;
            double totalTorque = 0.0;

            // Get the JointForceCmd component (represents actuator effort
            auto forceCmdComp = _ecm.Component<gz::sim::components::JointForceCmd>(jointEntity);
            
            
            if (forceCmdComp)
            {
                appliedTorque1 = forceCmdComp->Data()[0]; // First component for single DoF joints
                appliedTorque2 = forceCmdComp->Data()[1];
                appliedTorque3 = forceCmdComp->Data()[2];
                appliedTorque4 = forceCmdComp->Data()[3];
                appliedTorque5 = forceCmdComp->Data()[4];
                appliedTorque6 = forceCmdComp->Data()[5];
                gzdbg << "[JointForceCmd] Joint " << jointEntity << " | Applied Torque (Actuator1): " << appliedTorque1 << std::endl;
                gzdbg << "[JointForceCmd] Joint " << jointEntity << " | Applied Torque (Actuator2): " << appliedTorque2 << std::endl;
                gzdbg << "[JointForceCmd] Joint " << jointEntity << " | Applied Torque (Actuator3): " << appliedTorque3 << std::endl;
                gzdbg << "[JointForceCmd] Joint " << jointEntity << " | Applied Torque (Actuator4): " << appliedTorque4 << std::endl;
                gzdbg << "[JointForceCmd] Joint " << jointEntity << " | Applied Torque (Actuator5): " << appliedTorque5 << std::endl;
                gzdbg << "[JointForceCmd] Joint " << jointEntity << " | Applied Torque (Actuator6): " << appliedTorque6 << std::endl;
            }
            else
            {
                gzdbg << "[JointForceCmd] No applied actuator force found for joint " << jointEntity << std::endl;
            }
        }
    }
}
