#pragma once
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/int32.pb.h>
#include <mutex>
#include <regex>
#include <map>
#include <vector>

namespace gz { namespace sim { inline namespace GZ_SIM_VERSION_NAMESPACE { namespace systems {
class MotorFailureSystem : public gz::sim::System,
                           public gz::sim::ISystemConfigure,
                           public gz::sim::ISystemPreUpdate
{
public:
    MotorFailureSystem() = default;
    ~MotorFailureSystem() override = default;
    void Configure(const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
                   EntityComponentManager &_ecm, EventManager &_eventMgr) override;
    void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override;

private:
    void MotorFailureNumberCallback(const gz::msgs::Int32 &_msg);
    void FindMotorJoints(EntityComponentManager &_ecm);
    void ApplyMotorFailure(EntityComponentManager &_ecm);

    gz::sim::Model _model{kNullEntity};
    Entity _model_entity{kNullEntity};
    gz::transport::Node _node;
    std::string _gz_topic{"/motor_failure/motor_number"};
    std::vector<Entity> _motor_joints;
    bool _joints_found{false};
    int32_t _motor_failure_number{0};
    int32_t _prev_motor_failure_number{0};
    std::mutex _motor_failure_mutex;
};
}}}}
