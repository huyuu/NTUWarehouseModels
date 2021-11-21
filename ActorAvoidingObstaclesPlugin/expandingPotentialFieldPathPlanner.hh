#ifndef ALGORITHM_EXPANDINGPOTENTIALFIELDPATHPLANNER_HH_
#define ALGORITHM_EXPANDINGPOTENTIALFIELDPATHPLANNER_HH_

#include <iostream>
#include <ignition/math.hh>
#include <vector>

namespace gazebo {
  class GZ_PLUGIN_VISIBLE ExpandingPotentialFieldPathPlanner {
  public:
    /// constructor
    ExpandingPotentialFieldPathPlanner(ignition::math::Box&, vector<ignition::math::Box>&);
    ~ExpandingPotentialFieldPathPlanner();
    /// calculate potential map
    virtual void updatePotentialMap();
    /// calculate vector for next step
    virtual ignition::math::Vector3d calculateNextStepVector(const ignition::math::Vector3d& currenctPosition, const ignition::math::Vector3d& goingVector) const;


  private:
    vector<ignition::math::Vector3d> translateBoxToSurfacePoints(ignition::math::Box&) const;
    // Properties
    // actor bounding box
    ignition::math::Box actorBoundingBox;
    // obstacles bounding boxes
    vector<ignition::math::Box> obstacleBoundingBoxes;
    // target position
    ignition::math::Vector3d targetPosition {};
    // sample amount
    int sampleAmount {100};
    // potential map
    vector<vector<double>> potentialMap(sampleAmount, vector<double>(sampleAmount, 0.0));
    // gauss points and weights
    // reference: https://pomax.github.io/bezierinfo/legendre-gauss.html
    static const int gaussSampleAmount = 10;
    static const vector<double> gaussPoints = {
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
    static const vector<double> gaussWeights = {
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
  };
}

#endif
