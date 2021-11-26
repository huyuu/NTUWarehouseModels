#ifndef MY_PLUGINS_ACTORAVOIDINGOBSTACLESPLUGIN_HH_
#define MY_PLUGINS_ACTORAVOIDINGOBSTACLESPLUGIN_HH_

#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"
#include "ExpandingPotentialFieldPathPlanner.hh"
#include "AStarPathPlanner.hh"


namespace gazebo {
  class GZ_PLUGIN_VISIBLE ActorAvoidingObstaclesPlugin : public ModelPlugin {
  public: // Public Functions
    // Constructor
    ActorAvoidingObstaclesPlugin();

    /// \brief Load the actor plugin.
    /// \param[in] _model Pointer to the parent model.
    /// \param[in] _sdf Pointer to the plugin's SDF elements.
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    // Documentation Inherited.
    void Reset() override;


  private: // Private Functions
    /// \brief Function that is called every update cycle.
    /// \param[in] _info Timing information
    virtual void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Helper function to choose a new target location
    virtual void ChooseNewTarget();

    /// \brief Helper function to avoid obstacles. This implements a very
    /// simple vector-field algorithm.
    /// \param[in] _pos Direction vector that should be adjusted according
    /// to nearby obstacles.
    // virtual void HandleObstacles();


    // Private Properties

    /// \brief Pointer to the parent actor.
    physics::ActorPtr actor;

    /// \brief Pointer to the world, for convenience.
    physics::WorldPtr world;

    /// \brief Pointer to the sdf element.
    sdf::ElementPtr sdf;

    /// \brief Velocity of the actor
    ignition::math::Vector3d velocity;

    /// \brief List of connections
    std::vector<event::ConnectionPtr> connections;

    /// \brief Current target location
    ignition::math::Vector3d target;

    // vector which the actor is going
    ignition::math::Vector3d goingVector;

    /// \brief Target location weight (used for vector field)
    double targetWeight {1.0};

    /// \brief Obstacle weight (used for vector field)
    double obstacleWeight {1.0};

    /// \brief Time scaling factor. Used to coordinate translational motion
    /// with the actor's walking animation.
    double animationFactor {1.0};

    /// \brief Time of the last update.
    common::Time lastUpdate;

    /// \brief List of models to ignore. Used for vector field
    std::vector<std::string> ignoreModels;

    /// \brief Custom trajectory info.
    physics::TrajectoryInfoPtr trajectoryInfo;

    // the custom path planner
    AStarPathPlanner pathPlanner {};

    ignition::math::AxisAlignedBox outerMostBoundaryBox {};
  };
}


#endif
