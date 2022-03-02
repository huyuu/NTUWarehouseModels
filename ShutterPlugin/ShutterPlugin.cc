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
  this->dataPtr->doorController = NULL;
  this->dataPtr->liftController = NULL;
}

/////////////////////////////////////////////////
ShutterPlugin::~ShutterPlugin()
{
  this->dataPtr->updateConnection.reset();

  delete this->dataPtr->doorController;
  this->dataPtr->doorController = NULL;

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

  float floorHeight = 3.0;
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
    gzerr << "The elevator plugin is disabled.\n";
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
      &ShutterPlugin::OnElevator, this);
}

/////////////////////////////////////////////////
void ShutterPlugin::OnElevator(ConstGzStringPtr &_msg)
{
  // Currently we only expect the message to contain a floor to move to.
  try
  {
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
  if (!this->dataPtr->states.empty())
    return;

  // Move to the correct floor.
  this->dataPtr->states.push_back(new ShutterPluginPrivate::MoveState(
        _floor, this->dataPtr->liftController));
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

  double force = this->liftPID.Update(error, _info.simTime - this->prevSimTime);
  this->prevSimTime = _info.simTime;

  this->liftJoint->SetForce(0, force);

  if (std::abs(error) < 0.15)
  {
    this->state = ShutterPluginPrivate::LiftController::STATIONARY;
    return true;
  }
  else
  {
    this->state = ShutterPluginPrivate::LiftController::MOVING;
    return false;
  }
  IGN_PROFILE_END();
}

/////////////////////////////////////////////////
void ShutterPluginPrivate::LiftController::SetFloor(int _floor)
{
  this->floor = _floor;
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
