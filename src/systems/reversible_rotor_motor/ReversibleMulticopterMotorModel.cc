#include "ReversibleMulticopterMotorModel.hh"

#include <mutex>
#include <string>

#include <gz/msgs/actuators.pb.h>

#include <gz/common/Profiler.hh>

#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/Utility.hh>

#include <sdf/sdf.hh>

#include "gz/sim/components/Actuators.hh"
#include "gz/sim/components/ExternalWorldWrenchCmd.hh"
#include "gz/sim/components/JointAxis.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/ParentLinkName.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Wind.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::ReversibleMulticopterMotorModelPrivate
{
  public: void OnActuatorMsg(const msgs::Actuators &_msg);
  public: void UpdateForcesAndMoments(EntityComponentManager &_ecm);

  public: Entity jointEntity;
  public: std::string jointName;

  public: Entity linkEntity;
  public: std::string linkName;

  public: Entity parentLinkEntity; 
  public: std::string parentLinkName = "";

  public: Model model{kNullEntity};

  public: std::string robotNameSpace = "";
  public: std::string commandSubTopic = "";

  public: int motorNumber = 0;
  public: double maxRotVelocity = 2000.0;
  public: double motorInputVel = 0.0;
  public: double simVelSlowDown = 10.0;

  public: std::vector<double> TorquePolynomial = {0.0, 0.0, 0.0, 0.0};
  public: std::vector<double> ThrustPolynomial = {0.0, 0.0, 0.0, 0.0};

  public: std::optional<msgs::Actuators> recvdActuatorsMsg;
  public: std::mutex recvdActuatorsMsgMutex;
  public: transport::Node node;
};

ReversibleMulticopterMotorModel::ReversibleMulticopterMotorModel()
  : dataPtr(std::make_unique<ReversibleMulticopterMotorModelPrivate>())
{
}

void ReversibleMulticopterMotorModel::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  using string = std::string;
  dataPtr->model = Model(_entity);

  if (!dataPtr->model.Valid(_ecm))
  {
    gzerr << "ReversibleMulticopterMotorModel plugin should be attached to a model "
           << "entity. Failed to initialize." << std::endl;
    return;
  }

  auto sdfClone = _sdf->Clone();

  // Robot Name Space
  if(sdfClone->HasElement("robotName")){
    dataPtr->robotNameSpace = sdfClone->Get<string>("robotName");
  } else {
    gzwarn << "Robot Name Not Passed, using entity name\n";
    dataPtr->robotNameSpace = dataPtr->model.Name(_ecm);
  }

  // Command to Sub for motor control
  if (sdfClone->HasElement("commandSubTopic")){
    dataPtr->commandSubTopic = sdfClone->Get<string>("commandSubTopic");
  } else {
    gzerr << "ReversibleMultiCopterMotorModel found an empty commandSubTopic parameter. "
          << "Failed to initialize.";
    return;
  }

  // Joint Name
  if (sdfClone->HasElement("jointName")) {
    dataPtr->jointName = sdfClone->Get<string>("jointName");
  } 

  if (dataPtr->jointName.empty()) {
    gzerr << "ReversibleMulticopterMotorModel found an empty jointName parameter. "
           << "Failed to initialize.";
    return;
  }

  // Link Name
  if (sdfClone->HasElement("linkName")) {
    dataPtr->linkName = sdfClone->Get<string>("linkName");
  }

  if (dataPtr->linkName.empty()) {
    gzerr << "ReversibleMulticopterMotorModel found an empty linkName parameter. "
           << "Failed to initialize.";
    return;
  }

  if (sdfClone->HasElement("motorNumber")) {
    dataPtr->motorNumber = sdfClone->Get<int>("motorNumber");
  }

  auto a0Thrust = sdfClone->Get<double>("a0ThrustConstant");
  auto a1Thrust = sdfClone->Get<double>("a1ThrustConstant");
  auto a2Thrust = sdfClone->Get<double>("a2ThrustConstant");
  auto a3Thrust = sdfClone->Get<double>("a3ThrustConstant");

  this->dataPtr->ThrustPolynomial = {a0Thrust, a1Thrust, a2Thrust, a3Thrust};

  auto a0Torque = sdfClone->Get<double>("a0TorqueConstant");
  auto a1Torque = sdfClone->Get<double>("a1TorqueConstant");
  auto a2Torque = sdfClone->Get<double>("a2TorqueConstant");
  auto a3Torque = sdfClone->Get<double>("a3TorqueConstant");

  this->dataPtr->TorquePolynomial = {a0Torque, a1Torque, a2Torque, a3Torque};


  string topic = transport::TopicUtils::AsValidTopic( 
    "/" +  dataPtr->robotNameSpace + "/" + dataPtr->commandSubTopic
  );

  if (topic.empty())
  {
    gzerr << "Failed to create topic for command subscription." << std::endl;
    return;
  }
  else
  {
    gzerr << "Listening to topic: " << topic << std::endl;
  }
  this->dataPtr->node.Subscribe(topic,
      &ReversibleMulticopterMotorModelPrivate::OnActuatorMsg, this->dataPtr.get());
  gzerr << "Subscribed to topic: " << topic << std::endl;
}

void ReversibleMulticopterMotorModel::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("ReversibleMulticopterMotorModel::PreUpdate");

  if (_info.paused) {
    return;
  }

  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detect Jump back in time [" <<
      std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count() << 
      "s]. System may not work properl" << std::endl; 
  }

  if (dataPtr->jointEntity == kNullEntity) {
    dataPtr->jointEntity = dataPtr->model.JointByName(_ecm, dataPtr->jointName);

    const auto parentLinkName = _ecm.Component<components::ParentLinkName>(
        dataPtr->jointEntity);
    dataPtr->parentLinkName = parentLinkName->Data();
  }

  if (dataPtr->linkEntity == kNullEntity) {
    dataPtr->linkEntity = dataPtr->model.LinkByName(_ecm, dataPtr->linkName);
  }

  if (dataPtr->parentLinkEntity == kNullEntity) {
    dataPtr->parentLinkEntity = dataPtr->model.LinkByName(_ecm, dataPtr->parentLinkName);
  }

  if (dataPtr->jointEntity == kNullEntity || 
      dataPtr->linkEntity == kNullEntity || 
      dataPtr->parentLinkEntity == kNullEntity) {
    return;
  }


  bool doUpdateForcesAndMoments = true;
  const auto jointVelocity = _ecm.Component<components::JointVelocity> (dataPtr->jointEntity);

  if (!jointVelocity) {
    _ecm.CreateComponent(dataPtr->jointEntity, components::JointVelocity());
  } else if (jointVelocity->Data().empty()) {
    doUpdateForcesAndMoments = false;
  }

  if (!_ecm.Component<components::JointVelocityCmd>(dataPtr->jointEntity)) {
    _ecm.CreateComponent(dataPtr->jointEntity, components::JointVelocityCmd({0}));
    doUpdateForcesAndMoments = false;
  }

  if (!_ecm.Component<components::WorldPose>(dataPtr->linkEntity)){
    _ecm.CreateComponent(dataPtr->linkEntity, components::WorldPose());
    doUpdateForcesAndMoments = false;
  }
  if (!_ecm.Component<components::WorldLinearVelocity>(dataPtr->linkEntity)) {
    _ecm.CreateComponent(dataPtr->linkEntity,
        components::WorldLinearVelocity());
    doUpdateForcesAndMoments = false;
  }

  if (!_ecm.Component<components::WorldPose>(this->dataPtr->parentLinkEntity)) {
    _ecm.CreateComponent(this->dataPtr->parentLinkEntity, components::WorldPose());
    doUpdateForcesAndMoments = false;
  }

  if(doUpdateForcesAndMoments) {
    this->dataPtr->UpdateForcesAndMoments(_ecm);
  }
}

void ReversibleMulticopterMotorModelPrivate::OnActuatorMsg(
    const msgs::Actuators &_msg)
{
  std::lock_guard<std::mutex> lock(this->recvdActuatorsMsgMutex);
  this->recvdActuatorsMsg = _msg;
}

void ReversibleMulticopterMotorModelPrivate::UpdateForcesAndMoments(
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("ReversibleMulticopterMotorModelPrivate::UpdateForcesAndMoments");
  
  auto actuatorMsgComp = _ecm.Component<components::Actuators>(this->model.Entity());
  std::optional<msgs::Actuators> msg;

  if (actuatorMsgComp)
  {
    msg = actuatorMsgComp->Data();
  } else {
    std::lock_guard<std::mutex> lock(this->recvdActuatorsMsgMutex);

    if (this->recvdActuatorsMsg.has_value())
    {
      msg = *this->recvdActuatorsMsg;
      this->recvdActuatorsMsg.reset();
    }
  }


  if (msg.has_value()) {
    if (msg->velocity_size() > this->motorNumber) {
      this->motorInputVel = std::clamp(
        msg->velocity(this->motorNumber),
        -this->maxRotVelocity, 
        this->maxRotVelocity
      );
    } else if (msg->normalized_size() > this->motorNumber) {
      this->motorInputVel = std::clamp(msg->normalized(this->motorNumber), -1.0, 1.0) * this->maxRotVelocity;  
    }
  }


  sim::Link link(this->linkEntity);
  const auto worldPose = link.WorldPose(_ecm);
  using Vector3 = math::Vector3d;

  if  (!worldPose.has_value())
    gzerr << "worldPose is null.\n";

  // Compute thrust according to the polynomial we have defined
  double thrust = 0.0;
  for (unsigned int i = 0; i < this->ThrustPolynomial.size(); i++) {
    thrust += this->ThrustPolynomial[i] * std::pow(this->motorInputVel, i);
  }

  link.AddWorldForce(_ecm, worldPose->Rot().RotateVector(Vector3(0, 0, thrust)));
  
  double torque = 0.0;

  for (unsigned int i = 0; i < this->TorquePolynomial.size(); i++) {
    torque += this->TorquePolynomial[i] * std::pow(this->motorInputVel, i);
  }

  link.AddWorldForce(_ecm, worldPose->Rot().RotateVector(Vector3(0, 0, torque)));

  const auto jointVelCmd = _ecm.Component<components::JointVelocityCmd>(
      this->jointEntity); 
}

GZ_ADD_PLUGIN(ReversibleMulticopterMotorModel,
                    System,
                    ReversibleMulticopterMotorModel::ISystemConfigure,
                    ReversibleMulticopterMotorModel::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(ReversibleMulticopterMotorModel,
                          "gz::sim::systems::ReversibleMulticopterMotorModel")
