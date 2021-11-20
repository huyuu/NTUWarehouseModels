#include <gazebo/gazebo.hh>

namespace gazebo {
  class WorldPluginTutorial: public WorldPlugin {
  private:
    /* data */

  public:
    using WorldPlugin::WorldPlugin;
    virtual ~WorldPluginTutorial () = default;
    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override;
  };
  GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial);
}
