#include <gazebo/gazebo.hh>

namespace gazebo {
  class WorldPluginTutorial: public WorldPlugin {
  private:
    /* data */

  public:
    WorldPluginTutorial(): WorldPlugin();
    virtual ~WorldPluginTutorial () = default;
    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override;
  };
  GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial);


  WorldPluginTutorial::WorldPluginTutorial(): WorldPlugin() {
    printf("Hello World!\n");
  }
}
