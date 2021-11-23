#ifndef ALGORITHM_EXPANDINGPOTENTIALFIELDPATHPLANNER_HH_
#define ALGORITHM_EXPANDINGPOTENTIALFIELDPATHPLANNER_HH_

#include <iostream>
#include <vector>
#include <functional>
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"
#include <ignition/math.hh>

using std::vector;


namespace gazebo {
  class GZ_PLUGIN_VISIBLE ExpandingPotentialFieldPathPlanner {
  public:
    /// constructor
    ExpandingPotentialFieldPathPlanner();
    ~ExpandingPotentialFieldPathPlanner();
    /// update models when there is any change
    void updateModels(const ignition::math::AxisAlignedBox, const physics::World&, const std::vector<std::string>&);
    /// calculate vector for next step
    virtual ignition::math::Vector2d generateGradientNearPosition(const ignition::math::Vector3d&) const;


  private:
    // Member Functions
    /// calculate potential by Gauss Integral using the specific formula
    double __calculatePotentialUsingFormula(const double& x, const double& y, const double& X_down, const double& X_up, const double& Y_down, const double& Y_up) const;
    /// generate potential at point
    double __generatePotentialAtPoint(const ignition::math::Vector3d&) const;

    // Properties
    /// actor bounding box
    ignition::math::AxisAlignedBox actorBoundingBox;
    /// obstacles bounding boxes
    vector<ignition::math::AxisAlignedBox> obstacleBoundingBoxes;
    /// target position
    ignition::math::Vector3d targetPosition {};
    /// sample amount
    const int sampleAmount {100};
    /// potential map
    vector<vector<double>> potentialMap;
    /// gauss points and weights
    // reference: https://pomax.github.io/bezierinfo/legendre-gauss.html
    static const int gaussSampleAmount = 10;
    static const vector<double> gaussPoints;
    static const vector<double> gaussWeights;
    /// small delta h for numerous differential
    static constexpr double h {0.01}; // 1[cm]
  };
}

#endif
