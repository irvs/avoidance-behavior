#include <velodyne_bar.h>

#include <cnoid/Plugin>
#include <cnoid/ItemManager>
#include <cnoid/BodyMotionItem>

#include <iostream>
#include <string>
#include <stdlib.h>

using namespace cnoid;
using namespace grasp;

namespace {
  class VelodynePlugin : public Plugin {
    public:
      VelodynePlugin() : Plugin("Velodyne") {
        depend("Trajectory");
      }

      bool initialize() {
        addToolBar(grasp::VelodyneBar::instance());
        return true;
      }
  };
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(VelodynePlugin);
