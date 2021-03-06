/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <functional>

#include <ignition/common/Profiler.hh>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>

#include "ShutterPluginPrivate.hh"
#include "ShutterPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ShutterPlugin)

/////////////////////////////////////////////////
ShutterPlugin::ShutterPlugin()
  : dataPtr(new ShutterPluginPrivate)
{
  this->dataPtr->liftController = NULL;
  this->dataPtr->doorWaitTime = common::Time(1, 100);
  // std::cout << "Constructed 1 sec: " << this->dataPtr->doorWaitTime << " with nano sec = " << this->dataPtr->doorWaitTime.nsec << std::endl << std::endl;
}

/////////////////////////////////////////////////
ShutterPlugin::~ShutterPlugin()
{
  this->dataPtr->updateConnection.reset();

  delete this->dataPtr->liftController;
  this->dataPtr->liftController = NULL;
}

/////////////////////////////////////////////////
void ShutterPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "ShutterPlugin model pointer is NULL");
  GZ_ASSERT(_sdf, "ShutterPlugin sdf pointer is NULL");
  this->dataPtr->model = _model;
  this->dataPtr->sdf = _sdf;

  // Get the shutter topic.
  std::string shutterTopic = "~/shutter";
  if (this->dataPtr->sdf->HasElement("topic"))
    shutterTopic = this->dataPtr->sdf->Get<std::string>("topic");

  float floorHeight = 6.0;
  if (this->dataPtr->sdf->HasElement("floor_height"))
    floorHeight = this->dataPtr->sdf->Get<float>("floor_height");
  else
  {
    gzwarn << "No <floor_height> specified for elevator plugin. "
           << "Using a height of 3 meters. This value may cause "
           << "the elevator to move incorrectly.\n";
  }

  // Get the lift joint
  std::string liftJointName =
    this->dataPtr->sdf->Get<std::string>("lift_joint");

  this->dataPtr->liftJoint = this->dataPtr->model->GetJoint(liftJointName);
  if (!this->dataPtr->liftJoint)
  {
    gzerr << "Unable to find lift joint[" << liftJointName << "].\n";
    gzerr << "The shutter plugin is disabled.\n";
    return;
  }


  // Create the door and lift controllers.
  this->dataPtr->liftController = new ShutterPluginPrivate::LiftController(
      this->dataPtr->liftJoint, floorHeight);

  // Connect to the update event.
  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&ShutterPlugin::Update, this, std::placeholders::_1));

  // Create the node for communication
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init(this->dataPtr->model->GetWorld()->Name());

  // Subscribe to the elevator topic.
  this->dataPtr->elevatorSub = this->dataPtr->node->Subscribe(shutterTopic,
      &ShutterPlugin::OnShutter, this);
}

/////////////////////////////////////////////////
void ShutterPlugin::OnShutter(ConstGzStringPtr &_msg)
{
  // Currently we only expect the message to contain a floor to move to.
  try
  {
    std::cout << "Shutter moving to " << _msg->data() << std::endl;
    this->MoveToFloor(std::stoi(_msg->data()));
  }
  catch(...)
  {
    gzerr << "Unable to process elevator message["
          << _msg->data() << "]\n";
  }
}

/////////////////////////////////////////////////
void ShutterPlugin::MoveToFloor(const int _floor)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->stateMutex);

  // Ignore messages when the elevator is currently busy.
  if (!this->dataPtr->states.empty()) {
    std::cout << "shutter is busy ..." << std::endl;
    return;
  }

  // Move to the correct floor.
  this->dataPtr->states.push_back(new ShutterPluginPrivate::MoveState(
        _floor, this->dataPtr->liftController));

  // Step 4: Wait
  this->dataPtr->states.push_back(new ShutterPluginPrivate::WaitState(
        this->dataPtr->doorWaitTime));
}

/////////////////////////////////////////////////
void ShutterPlugin::Update(const common::UpdateInfo &_info)
{
  IGN_PROFILE("ShutterPlugin::Update");
  IGN_PROFILE_BEGIN("Update");
  std::lock_guard<std::mutex> lock(this->dataPtr->stateMutex);

  // Process the states
  if (!this->dataPtr->states.empty())
  {
    // Update the front state, and remove it if the state is done
    if (this->dataPtr->states.front()->Update())
    {
      delete this->dataPtr->states.front();
      this->dataPtr->states.pop_front();
    }
  }

  // Update the controllers
  this->dataPtr->liftController->Update(_info);
  IGN_PROFILE_END();
}

////////////////////////////////////////////////
void ShutterPlugin::Reset()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->stateMutex);
  for (auto s: this->dataPtr->states)
    delete s;
  this->dataPtr->states.clear();
  this->dataPtr->liftController->Reset();
}

////////////////////////////////////////////////
// ShutterPluginPrivate Class

/////////////////////////////////////////////////
ShutterPluginPrivate::~ShutterPluginPrivate()
{

  delete this->liftController;
  this->liftController = NULL;

  for (auto s: this->states)
    delete s;
  this->states.clear();
}


////////////////////////////////////////////////
// MoveState Class

/////////////////////////////////////////////////
ShutterPluginPrivate::MoveState::MoveState(int _floor, LiftController *_ctrl)
  : State(), floor(_floor), ctrl(_ctrl)
{
  // std::cout << "MoveState created." << std::endl;
}

/////////////////////////////////////////////////
void ShutterPluginPrivate::MoveState::Start()
{
  this->ctrl->SetFloor(this->floor);
  this->started = true;
}

/////////////////////////////////////////////////
bool ShutterPluginPrivate::MoveState::Update()
{
  IGN_PROFILE("ShutterPlugin::MoveState");
  IGN_PROFILE_BEGIN("Update");

  if (!this->started)
  {
    // std::cout << "Shutter started." << std::endl;
    this->Start();
    return false;
  }
  else
  {
    return this->ctrl->GetState() ==
      ShutterPluginPrivate::LiftController::STATIONARY;
  }
  IGN_PROFILE_END();
}


////////////////////////////////////////////////
// WaitState Class

/////////////////////////////////////////////////
ShutterPluginPrivate::WaitState::WaitState(const common::Time &_waitTime)
  : State(), waitTimer(_waitTime, true)
{
}

/////////////////////////////////////////////////
void ShutterPluginPrivate::WaitState::Start()
{
  this->waitTimer.Reset();
  this->waitTimer.Start();
  this->started = true;
}

/////////////////////////////////////////////////
bool ShutterPluginPrivate::WaitState::Update()
{
  IGN_PROFILE("ElevatorPlugin::WaitState");
  IGN_PROFILE_BEGIN("Update");

  if (!this->started)
  {
    this->Start();
    return false;
  }
  else
  {
    if (this->waitTimer.GetElapsed() == common::Time::Zero)
      return true;
    else
      return false;
  }
  IGN_PROFILE_END();
}


////////////////////////////////////////////////
// LiftController Class

/////////////////////////////////////////////////
ShutterPluginPrivate::LiftController::LiftController(
    physics::JointPtr _liftJoint, float _floorHeight)
  : state(STATIONARY), floor(0), floorHeight(_floorHeight),
    liftJoint(_liftJoint)
{
  this->liftPID.Init(100000, 0, 100000.0);
}

/////////////////////////////////////////////////
void ShutterPluginPrivate::LiftController::Reset()
{
  this->prevSimTime = common::Time::Zero;
}

/////////////////////////////////////////////////
bool ShutterPluginPrivate::LiftController::Update(
    const common::UpdateInfo &_info)
{
  IGN_PROFILE("ShutterPlugin::LiftController");
  IGN_PROFILE_BEGIN("Update");

  // Bootstrap the time.
  if (this->prevSimTime == common::Time::Zero)
  {
    this->prevSimTime = _info.simTime;
    return false;
  }

  double error = this->liftJoint->Position() -
    (this->floor * this->floorHeight);
  // std::cout << std::endl;
  // std::cout << "Life Joint: " << this->liftJoint->GetName() << std::endl;
  // std::cout << "Life Pose: " << this->liftJoint->WorldPose() << std::endl;
  // std::cout << "Lift Position: " << this->liftJoint->Position() << std::endl;
  // std::cout << "this->floor: " << this->floor << std::endl;
  // std::cout << "error: " << error << std::endl;

  // double force = this->liftPID.Update(error, _info.simTime - this->prevSimTime) / 1000000000.0 * 1000000.0;
  double force = this->liftPID.Update(error, _info.simTime - this->prevSimTime);
  // std::cout << "_info.simTime" << _info.simTime << std::endl;
  // std::cout << "this->prevSimTime" << this->prevSimTime << std::endl;
  // std::cout << "Time delta: " << _info.simTime - this->prevSimTime << std::endl;
  this->prevSimTime = _info.simTime;

  // std::cout << "Lift set force: " << force <<std::endl;
  this->liftJoint->SetForce(0, force);

  if (std::abs(error) < 0.15)
  {
    // std::cout << "Lift Error <= 0.15 -> state to STATIONARY" << std::endl;
    this->state = ShutterPluginPrivate::LiftController::STATIONARY;
    return true;
  }
  else
  {
    // std::cout << "Lift Error > 0.15 -> state to MOVING" << std::endl;
    this->state = ShutterPluginPrivate::LiftController::MOVING;
    return false;
  }
  IGN_PROFILE_END();
}

/////////////////////////////////////////////////
void ShutterPluginPrivate::LiftController::SetFloor(int _floor)
{
  this->floor = _floor;
  std::cout << "Set shutter to floor: " << _floor << std::endl;
}

/////////////////////////////////////////////////
int ShutterPluginPrivate::LiftController::GetFloor() const
{
  return this->floor;
}

/////////////////////////////////////////////////
ShutterPluginPrivate::LiftController::State
ShutterPluginPrivate::LiftController::GetState() const
{
  return this->state;
}
