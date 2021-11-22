#ifndef ALGORITHM_EXPANDINGPOTENTIALFIELDPATHPLANNER_HH_
#define ALGORITHM_EXPANDINGPOTENTIALFIELDPATHPLANNER_HH_

#include <iostream>
#include <vector>
#include <functional>
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"
#include <ignition/math.hh>

namespace gazebo {
  class GZ_PLUGIN_VISIBLE ExpandingPotentialFieldPathPlanner {
  public:
    /// constructor
    ExpandingPotentialFieldPathPlanner(const ignition::math::Box, const physics::World&);
    ~ExpandingPotentialFieldPathPlanner();
    /// calculate vector for next step
    virtual ignition::math::Vector2d generateGradientNearPosition(const ignition::math::Vector3d&) const;


  private:
    // Member Functions
    /// update models when there is any change
    void __updateModels(const ignition::math::Box, const physics::World&);
    /// calculate potential by Gauss Integral using the specific formula
    double __calculatePotentialUsingFormula(const double& x, const double& y, const double& X_down, const double& X_up, const double& Y_down, const double& Y_up) const;
    /// generate potential at point
    double __generatePotentialAtPoint(const ignition::math::Vector3d&) const;

    // Properties
    /// actor bounding box
    ignition::math::Box actorBoundingBox;
    /// obstacles bounding boxes
    vector<ignition::math::Box> obstacleBoundingBoxes;
    /// target position
    ignition::math::Vector3d targetPosition {};
    /// sample amount
    int sampleAmount {100};
    /// potential map
    vector<vector<double>> potentialMap(sampleAmount, vector<double>(sampleAmount, 0.0));
    /// gauss points and weights
    // reference: https://pomax.github.io/bezierinfo/legendre-gauss.html
    static const int gaussSampleAmount = 10;
    static const vector<double> gaussPoints {
      -0.1488743389816312,
      0.1488743389816312,
      -0.4333953941292472,
      0.4333953941292472,
      -0.6794095682990244,
      0.6794095682990244,
      -0.8650633666889845,
      0.8650633666889845,
      -0.9739065285171717,
      0.9739065285171717
    };
    static const vector<double> gaussWeights {
      0.2955242247147529,
      0.2955242247147529,
      0.2692667193099963,
      0.2692667193099963,
      0.2190863625159820,
      0.2190863625159820,
      0.1494513491505806,
      0.1494513491505806,
      0.0666713443086881,
      0.0666713443086881
    };
    /// small delta h for numerous differential
    static const double h {0.01}; // 1[cm]
  };
}

#endif
