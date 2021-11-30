#include <functional>

#include <ignition/math.hh>
#include "gazebo/physics/physics.hh"
#include "ActorAvoidingObstaclesPlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(ActorAvoidingObstaclesPlugin)

#define WALKING_ANIMATION "walking"

// reference: https://gazebosim.org/tutorials?tut=actor&cat=build_robot


// MARK: - Definitions

// Constructor
ActorAvoidingObstaclesPlugin::ActorAvoidingObstaclesPlugin(): ModelPlugin() {
  std::cout << "Actor Avoiding Obstacls Plugin Constructed." << std::endl;
}


// Override Function: Load
void ActorAvoidingObstaclesPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  this->sdf = _sdf;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->actorWidth = 1.5;
  this->world = this->actor->GetWorld();

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(std::bind(&ActorAvoidingObstaclesPlugin::OnUpdate, this, std::placeholders::_1)));

  this->Reset();

  // Read in the target weight
  if (_sdf->HasElement("target_weight"))
    this->targetWeight = _sdf->Get<double>("target_weight");
  else
    this->targetWeight = 1.15;

  // Read in the obstacle weight
  if (_sdf->HasElement("obstacle_weight"))
    this->obstacleWeight = _sdf->Get<double>("obstacle_weight");
  else
    this->obstacleWeight = 1.5;

  // Read in the animation factor (applied in the OnUpdate function).
  if (_sdf->HasElement("animation_factor"))
    this->animationFactor = _sdf->Get<double>("animation_factor");
  else
    this->animationFactor = 4.5;

  // Add our own name to models we should ignore when avoiding obstacles.
  this->ignoreModels.push_back(this->actor->GetName());

  // Read in the other obstacles to ignore
  if (_sdf->HasElement("ignore_obstacles"))
  {
    sdf::ElementPtr modelElem = _sdf->GetElement("ignore_obstacles")->GetElement("model");
    while (modelElem) {
      this->ignoreModels.push_back(modelElem->Get<std::string>());
      modelElem = modelElem->GetNextElement("model");
    }
  }

  // handle pathPlanner related stuffs
  this->pathPlanner = AStarPathPlanner(this->actor->WorldPose().Pos(), this->target, this->actor->BoundingBox(), this->world, this->ignoreModels, this->actorWidth, true);
  this->pathPlanner.generatePathInCheatMode();
  // std::cout << "here!" << std::endl;
  // this->pathPlanner.lazyConstructor(this->actor->WorldPose().Pos(), this->target);
  // this->pathPlanner.updateModels(this->actor->BoundingBox(), this->world, this->ignoreModels);

  const unsigned int modelCount {world->ModelCount()};
  for (unsigned int i = 0; i < modelCount; ++i) {
    const physics::ModelPtr model = this->world->ModelByIndex(i);
    if (model->GetName() == "walls") {
      this->outerMostBoundaryBox = model->BoundingBox();
      break;
    }
  }
}


// Override Function: Reset
void ActorAvoidingObstaclesPlugin::Reset() {
  this->velocity = 4.0;
  this->lastUpdate = 0;

  if (this->sdf && this->sdf->HasElement("target"))
    this->target = this->sdf->Get<ignition::math::Vector3d>("target");
  else
    this->target = ignition::math::Vector3d(0, -5, 1.2138);

  auto skelAnims = this->actor->SkeletonAnimations();
  if (skelAnims.find(WALKING_ANIMATION) == skelAnims.end())
  {
    gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
  }
  else
  {
    // Create custom trajectory
    this->trajectoryInfo.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo->type = WALKING_ANIMATION;
    this->trajectoryInfo->duration = 1.0;

    this->actor->SetCustomTrajectory(this->trajectoryInfo);
  }
}


// Private Function: Choose New Target
void ActorAvoidingObstaclesPlugin::ChooseNewTarget() {
  ignition::math::Vector3d newTarget(this->target);
  while ((newTarget - this->target).Length() < 2.0)
  {
    newTarget.X(ignition::math::Rand::DblUniform(20, 10)); // (mean, sigma) for normal distribution
    newTarget.Y(ignition::math::Rand::DblUniform(20, 10)); // (mean, sigma) for normal distribution

    for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
    {
      double dist = (this->world->ModelByIndex(i)->WorldPose().Pos()
          - newTarget).Length();
      if (dist < 2.0)
      {
        newTarget = this->target;
        break;
      }
    }
  }
  this->target = newTarget;
  this->goingVector = newTarget - this->actor->WorldPose().Pos();
  this->goingVector.Normalize();
  this->pathPlanner = AStarPathPlanner(this->actor->WorldPose().Pos(), this->target, this->actor->BoundingBox(), this->world, this->ignoreModels, this->actorWidth, true);
  this->pathPlanner.generatePathInCheatMode();
}


// Private Function: Handle Obstacles
// void ActorAvoidingObstaclesPlugin::HandleObstacles() {
//   for (unsigned int i = 0; i < this->world->ModelCount(); ++i) {
//     const physics::ModelPtr model = this->world->ModelByIndex(i);
//     // if the model is in ignoringModels, meaning that the actor should NOT avoid it, we end the handleObstacle process.
//     if (std::find(this->ignoreModels.begin(), this->ignoreModels.end(), model->GetName()) != this->ignoreModels.end()) {
//       return ;
//     }
//     ignition::math::Vector3d vectorFromActorToObstacle = model->WorldPose().Pos() - this->actor->WorldPose().Pos();
//     const double&& distanceFromActorToObstacle {vectorFromActorToObstacle.Length()};
//     if (distanceFromActorToObstacle < 4.0) {
//       const double&& invModelDist {this->obstacleWeight / distanceFromActorToObstacle};
//       vectorFromActorToObstacle.Normalize();
//       vectorFromActorToObstacle *= invModelDist;
//       vectorFromActorToTarget -= vectorFromActorToObstacle;
//     }
//   }
// }


// Private Function: OnUpdate
void ActorAvoidingObstaclesPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  // Time delta
  const double dt = (_info.simTime - this->lastUpdate).Double();

  ignition::math::Pose3d pose = this->actor->WorldPose();
  ignition::math::Vector3d currentPosition = pose.Pos();
  ignition::math::Vector3d vectorFromActorToTarget = this->target - currentPosition;
  this->goingVector = this->target - currentPosition;
  this->goingVector.Normalize();
  ignition::math::Vector3d rpy = pose.Rot().Euler();

  // Choose a new target position if the actor has reached its current
  // target.
  if (vectorFromActorToTarget.Length() < AStarPathPlanner::distanceAsReached) {
    this->ChooseNewTarget();
  }

  // Adjust the direction vector by avoiding obstacles
  // this->HandleObstacles();
  // this->goingVector = this->pathPlanner.generateGradientNearPosition(currentPosition, this->target);
  this->goingVector = this->pathPlanner.generateGradientNearPosition_cheatMode(currentPosition);
  // this->goingVector = ignition::math::Vector3d {0.0, 0.0, 0.0};

  // Compute the yaw orientation
  ignition::math::Angle yaw = atan2(currentPosition.Y(), currentPosition.X()) + 1.5707 - rpy.Z();
  yaw.Normalize();

  // Rotate in place, instead of jumping.
  if (std::abs(yaw.Radian()) > IGN_DTOR(10))
  {
    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+
        yaw.Radian()*0.001);
  }
  else
  {
    // pose.Pos() += this->goingVector * this->velocity * dt;
    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+yaw.Radian());
  }
  // std::cout << "old position: " << pose.Pos() << ", ";
  pose.Pos() += this->goingVector * this->velocity * dt;
  // std::cout << "goingVector: " << this->goingVector * this->velocity * dt << std::endl;

  // Make sure the actor stays within bounds
  pose.Pos().X(std::max(this->outerMostBoundaryBox.Min().X(), std::min(this->outerMostBoundaryBox.Max().X(), pose.Pos().X())));
  pose.Pos().Y(std::max(this->outerMostBoundaryBox.Min().Y(), std::min(this->outerMostBoundaryBox.Max().Y(), pose.Pos().Y())));
  pose.Pos().Z(1.2138);

  // Distance traveled is used to coordinate motion with the walking
  // animation
  double distanceTraveled = (pose.Pos() -
      this->actor->WorldPose().Pos()).Length();

  this->actor->SetWorldPose(pose, false, false);
  this->actor->SetScriptTime(this->actor->ScriptTime() +
    (distanceTraveled * this->animationFactor));
  this->lastUpdate = _info.simTime;
}
