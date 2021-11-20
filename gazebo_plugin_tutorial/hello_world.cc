#include <gazebo/gazebo.hh>

namespace gazebo {
  class WorldPluginTutorial: public WorldPlugin {
  public:
    WorldPluginTutorial();
    virtual ~WorldPluginTutorial () = default;
    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override;
  };
  GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial);


  WorldPluginTutorial::WorldPluginTutorial(): WorldPlugin() {
    printf("Hello World!\n");
  }


  void WorldPluginTutorial::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
    std::cout << "loaded" << std::endl;
  }
}
