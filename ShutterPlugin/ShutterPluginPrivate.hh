
#ifndef GAZEBO_PLUGINS_SHUTTERPLUGINPRIVATE_HH_
#define GAZEBO_PLUGINS_SHUTTERPLUGINPRIVATE_HH_

#include <list>
#include <mutex>
#include <string>

#include <sdf/sdf.hh>
#include <ignition/transport/Node.hh>

#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Subscriber.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/common/Timer.hh>

namespace gazebo
{
  /// \internal
  /// \brief Private data for the ShutterPlugin
  class ShutterPluginPrivate
  {
    /// \brief Constructor
    public: ShutterPluginPrivate() = default;

    /// \brief Destructor
    public: virtual ~ShutterPluginPrivate();


    /// \brief Controller for raising and lowering the elevator.
    public: class LiftController
    {
      /// \enum State
      /// \brief Lift state
      public: enum State {
                /// \brief The lift is moving
                MOVING,

                /// \brief The lift is stationary
                STATIONARY
              };

      /// \brief Constructor
      /// \param[in] _liftJoint Pointer to the joint that should be
      /// controlled.
      /// \param[in] _floorHeight Height of each floor.
      public: LiftController(physics::JointPtr _liftJoint,
                             float _floorHeight);

      /// \brief Destructor
      public: virtual ~LiftController() = default;

      /// \brief Set the current floor to move to.
      /// \param[in] _floor Floor number.
      public: void SetFloor(int _floor);

      /// \brief Get the current floor.
      /// \return Floor number
      public: int GetFloor() const;

      /// \brief Get the current state.
      /// \return Current lift state.
      public: ShutterPluginPrivate::LiftController::State GetState() const;

      /// \brief Reset the controller
      public: void Reset();

      /// \brief Update the controller.
      /// \param[in] _info Update information provided by the server.
      /// \return True if the target has been reached.
      public: virtual bool Update(const common::UpdateInfo &_info);

      /// \brief State of the controller.
      public: State state;

      /// \brief Floor the elevator is on or moving to.
      public: int floor;

      /// \brief Height of each floor.
      public: float floorHeight;

      /// \brief Joint to control
      public: physics::JointPtr liftJoint;

      /// \brief PID controller.
      public: common::PID liftPID;

      /// \brief Previous simulation time.
      public: common::Time prevSimTime;
    };


    /// \brief State base class
    public: class State
    {
      /// \brief Constructor
      public: State() : started(false) {}

      /// \brief Destructor
      public: virtual ~State() = default;

      /// \brief State name
      public: std::string name;

      /// \brief Used to start a state.
      public: virtual void Start() {}

      /// \brief Used to update a state.
      public: virtual bool Update() {return true;}

      /// \brief True when started.
      protected: bool started;
    };


    /// \brief State used to move the elevator to a floor.
    public: class MoveState : public State
    {
      /// \brief Constructor
      /// \param[in] _floor Floor index to move to.
      /// \param[in] _ctrl Lift controller pointer.
      public: MoveState(int _floor, LiftController *_ctrl);

      // Documentation inherited
      public: virtual void Start();

      // Documentation inherited
      public: virtual bool Update();

      /// \brief Target floor number.
      public: int floor;

      /// \brief Lift controller.
      public: LiftController *ctrl;
    };



    /// \brief Pointer to the elevator model.
    public: physics::ModelPtr model;

    /// \brief Pointer to the joint that lifts the elevator
    public: physics::JointPtr liftJoint;

    /// \brief Pointer to the joint that opens the door
    public: physics::JointPtr doorJoint;

    /// \brief SDF pointer.
    public: sdf::ElementPtr sdf;

    /// \brief Pointer to the update event connection
    public: event::ConnectionPtr updateConnection;

    /// \brief Node for communication
    public: transport::NodePtr node;

    /// \brief Used to subscribe to command message. This will call the
    /// OnElevator function when a message arrives.
    public: transport::SubscriberPtr elevatorSub;

    /// \brief Lift controller.
    public: LiftController *liftController;

    /// \brief List of states that should be executed.
    public: std::list<State *> states;

    /// \brief Mutex to protect states.
    public: std::mutex stateMutex;

    // Place ignition::transport objects at the end of this file to
    // guarantee they are destructed first.

    /// \brief Ignition node for communication
    public: ignition::transport::Node nodeIgn;
  };
}
#endif
