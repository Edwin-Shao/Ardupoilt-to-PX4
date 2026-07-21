#include "MotorFailureSystem.hpp"
#include <gz/plugin/Register.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/Util.hh>
#include <iostream>

using namespace gz;
using namespace sim;
using namespace systems;

void MotorFailureSystem::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm, EventManager &)
{
    this->_model = Model(_entity);
    this->_model_entity = _entity;
    if (!this->_model.Valid(_ecm)) {
        gzerr << "[MotorFailure] plugin must be attached to a model entity" << std::endl;
        return;
    }
    std::string model_name = this->_model.Name(_ecm);
    if (_sdf->HasElement("MotorFailureTopic")) {
        this->_gz_topic = _sdf->Get<std::string>("MotorFailureTopic");
    } else {
        this->_gz_topic = "/model/" + model_name + "/motor_failure/motor_number";
    }
    if (!this->_node.Subscribe(this->_gz_topic, &MotorFailureSystem::MotorFailureNumberCallback, this)) {
        gzerr << "[MotorFailure] failed to subscribe to " << this->_gz_topic << std::endl;
        return;
    }
    gzmsg << "[MotorFailure] model=" << model_name << " topic=" << this->_gz_topic << std::endl;
}

void MotorFailureSystem::FindMotorJoints(EntityComponentManager &_ecm)
{
    if (this->_joints_found) return;
    this->_motor_joints.clear();
    auto joints = this->_model.Joints(_ecm);
    std::regex motorPattern("rotor_(\\d+)_joint");
    std::smatch match;
    std::map<int, Entity> motorMap;
    for (const auto &joint : joints) {
        auto nameComp = _ecm.Component<components::Name>(joint);
        if (nameComp && std::regex_match(nameComp->Data(), match, motorPattern)) {
            try {
                int motorNumber = std::stoi(match[1].str());
                motorMap[motorNumber] = joint;
            } catch (...) {}
        }
    }
    for (const auto &pair : motorMap) {
        if (pair.first >= static_cast<int>(this->_motor_joints.size()))
            this->_motor_joints.resize(pair.first + 1, kNullEntity);
        this->_motor_joints[pair.first] = pair.second;
    }
    if (!this->_motor_joints.empty()) {
        gzmsg << "[MotorFailure] found " << this->_motor_joints.size() << " motors" << std::endl;
        this->_joints_found = true;
    }
}

void MotorFailureSystem::ApplyMotorFailure(EntityComponentManager &_ecm)
{
    int32_t current_failure;
    {
        std::lock_guard<std::mutex> lock(this->_motor_failure_mutex);
        current_failure = this->_motor_failure_number;
    }
    if (current_failure > 0 && current_failure <= static_cast<int32_t>(this->_motor_joints.size())) {
        int motorIdx = current_failure - 1;
        Entity jointEntity = this->_motor_joints[motorIdx];
        if (jointEntity != kNullEntity) {
            auto jointVelCmd = _ecm.Component<components::JointVelocityCmd>(jointEntity);
            if (jointVelCmd) {
                *jointVelCmd = components::JointVelocityCmd({0.0});
            }
        }
    }
}

void MotorFailureSystem::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
    if (_info.paused) return;
    if (!this->_joints_found) { this->FindMotorJoints(_ecm); }
    else { this->ApplyMotorFailure(_ecm); }
}

void MotorFailureSystem::MotorFailureNumberCallback(const gz::msgs::Int32 &_msg)
{
    std::lock_guard<std::mutex> lock(this->_motor_failure_mutex);
    this->_motor_failure_number = _msg.data();
    gzdbg << "[MotorFailure] motor=" << this->_motor_failure_number << std::endl;
}

GZ_ADD_PLUGIN(MotorFailureSystem, gz::sim::System,
    MotorFailureSystem::ISystemConfigure, MotorFailureSystem::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(MotorFailureSystem, "gz::sim::systems::MotorFailureSystem")
