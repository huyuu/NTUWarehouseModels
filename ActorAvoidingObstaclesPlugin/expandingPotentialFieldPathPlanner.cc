#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"
#include "expandingPotentialFieldPathPlanner.hh"
using namespace gazebo


// MARK: - Definitions

// constructor
// ExpandingPotentialFieldPathPlanner::ExpandingPotentialFieldPathPlanner(ignition::math::Box& actorBoundingBox, vector<ignition::math::Box>& obstacleBoundingBoxes) {
ExpandingPotentialFieldPathPlanner::ExpandingPotentialFieldPathPlanner(const ignition::math::Box actorBoundingBox, const physics::World& world) {
  ExpandingPotentialFieldPathPlanner::updateBoundingBoxes(const ignition::math::Box actorBoundingBox, const physics::World& world);
}


// update boundingBoxes
void ExpandingPotentialFieldPathPlanner::updateModels(const ignition::math::Box actorBoundingBox, const physics::World& world) {
  this->actorBoundingBox = actorBoundingBox;
  const int modelCount {world.ModelCount()};
  for (unsigned int i = 0; i < modelCount; ++i) {
    this->obstacles.push_back(world.ModelByIndex(i));
  }
}


// update potential map
// void ExpandingPotentialFieldPathPlanner::updatePotentialMap() {
//   for (int i = 0; i < this->sampleAmount; ++i) {
//     for (int j = 0; j < this->sampleAmount; ++j) {
//       double sum {0.0};
//       const double& coordinateX {this->coordinateMapX[i][j]};
//       const double& coordinateY {this->coordinateMapY[i][j]};
//       // generate barrier potential for obstacles
//       for (const auto& obstacle: this->obstacles) {
//         const ignition::math::Box boundingBox {obstacle->BoundingBox()};
//         sum += this->__generatePotentialAtPointFromBoundingBox(coordinateX, coordinateY, boundingBox);
//         // if (boundingBox.Min().X() < this->coordinateMapX[i][j] &&\
//         //   boundingBox.Min().Y() < this->coordinateMapY[i][j] &&\
//         //   boundingBox.Max().X() > this->coordinateMapX[i][j] &&\
//         //   boundingBox.Max().Y() > this->coordinateMapY[i][j]) {
//         //     sum += 1.0 / (coordinateMapX[i][j]*coordinateMapX[i][j] + coordinateMapY[i][j]*coordinateMapY[i][j] + coordinateMapZ[i][j]*coordinateMapZ[i][j]);
//         //   }
//       }
//       this->potentialMap[i][j] = std::move(sum);
//     }
//   }
// }


// generate potential from bounding box
double ExpandingPotentialFieldPathPlanner::__calculatePotentialUsingFormula(const double& x, const double& y, const double& X_down, const double& X_up, const double& Y_down, const double& Y_up) const {
  double sum_i = 0.0;
  int i = 0;
  int j = 0;
  for (i = 0; i < ExpandingPotentialFieldPathPlanner::gaussSampleAmount; ++i) {
    const x_ = ExpandingPotentialFieldPathPlanner::gaussPoints[i];
    double sum_j = 0.0;
    for (j = 0; j < ExpandingPotentialFieldPathPlanner::gaussSampleAmount; ++j) {
      const y_ = ExpandingPotentialFieldPathPlanner::gaussPoints[j];
      sum_j += ((X_up-X_down)/2.0 * (Y_up-Y_down)/2.0) / std::sqrt( std::pow(x-((X_up-X_down)/2.0*(x_+1)+X_down), 2) + std::pow(y-((Y_up-Y_down)/2.0*(y_+1)+Y_down), 2) );
    }
    sum_j *= gaussWeights[j];
    sum_i += sum_j;
  }
  sum_i *= gaussWeights[i];
  return sum_i;
}


// calculate next step vector
double ExpandingPotentialFieldPathPlanner::__generatePotentialAtPoint(const ignition::math::Vector3d& point) const {
  const double x {point.X()};
  const double y {point.Y()};
  double potentialAtPoint {0.0};
  // generate barrier potential for obstacles
  for (const auto& obstacle: this->obstacles) {
    const ignition::math::Box boundingBox {obstacle->BoundingBox()};
    const double X_down {boundingBox.Min().X()};
    const double X_up {boundingBox.Max().X()};
    const double Y_down {boundingBox.Min().Y()};
    const double Y_up {boundingBox.Max().Y()};
    potentialAtPoint += this->__calculatePotentialUsingFormula(x, y, X_down, X_up, Y_down, Y_up);
  }
  return potentialAtPoint;
}


// generate gradient near point
ignition::math::Vector3d ExpandingPotentialFieldPathPlanner::__generateGradientNearCurrentPosition(const ignition::math::Vector3d& currentPosition) const {

}
