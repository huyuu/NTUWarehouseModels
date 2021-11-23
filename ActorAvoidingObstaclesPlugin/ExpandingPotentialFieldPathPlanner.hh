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
    void updateModels(const ignition::math::AxisAlignedBox, const physics::WorldPtr, const std::vector<std::string>&);
    /// calculate vector for next step
    virtual ignition::math::Vector3d generateGradientNearPosition(const ignition::math::Vector3d&, const ignition::math::Vector3d&) const;
    vector<double> gaussPoints;
    vector<double> gaussWeights;
    void storePotentialsOnSamplePoints(ignition::math::AxisAlignedBox) const;


  private:
    // Member Functions
    /// calculate potential by Gauss Integral using the specific formula
    double __calculatePotentialUsingFormula(const double& x, const double& y, const double& X_down, const double& X_up, const double& Y_down, const double& Y_up, const double) const;
    double __calculatePotentialUsingFormulaForEmittingPoint(const ignition::math::Vector2d&, const ignition::math::Vector2d&, const double) const;
    /// generate potential at point
    double __generatePotentialAtPoint(const ignition::math::Vector2d&, const ignition::math::Vector2d&) const;

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
    /// small delta h for numerous differential
    static constexpr double h {0.01}; // 1[cm]
  };

  // ExpandingPotentialFieldPathPlanner::gaussPoints = vector<double>{
  //   -0.1488743389816312,
  //   0.1488743389816312,
  //   -0.4333953941292472,
  //   0.4333953941292472,
  //   -0.6794095682990244,
  //   0.6794095682990244,
  //   -0.8650633666889845,
  //   0.8650633666889845,
  //   -0.9739065285171717,
  //   0.9739065285171717
  // };
  // ExpandingPotentialFieldPathPlanner::gaussWeights = vector<double>{
  //   0.2955242247147529,
  //   0.2955242247147529,
  //   0.2692667193099963,
  //   0.2692667193099963,
  //   0.2190863625159820,
  //   0.2190863625159820,
  //   0.1494513491505806,
  //   0.1494513491505806,
  //   0.0666713443086881,
  //   0.0666713443086881
  // };
}

#endif
