#include "ExpandingPotentialFieldPathPlanner.hh"
using namespace gazebo
using std::vector

// MARK: - Definitions

// constructor
// ExpandingPotentialFieldPathPlanner::ExpandingPotentialFieldPathPlanner(const ignition::math::Box actorBoundingBox, const physics::World& world) {
//   ExpandingPotentialFieldPathPlanner::__updateModels(const ignition::math::Box actorBoundingBox, const physics::World& world);
// }
ExpandingPotentialFieldPathPlanner::ExpandingPotentialFieldPathPlanner() {}


// update boundingBoxes
void ExpandingPotentialFieldPathPlanner::updateModels(const ignition::math::Box actorBoundingBox, const physics::World& world, const std::vector<std::string>& ignoreModels) {
  this->actorBoundingBox = actorBoundingBox;
  const int modelCount {world.ModelCount()};
  for (unsigned int i = 0; i < modelCount; ++i) {
    const physics::ModelPtr model = this->world->ModelByIndex(i);
    if (std::find(ignoreModels.begin(), ignoreModels.end(), model->GetName()) == ignoreModels.end())
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
ignition::math::Vector2d ExpandingPotentialFieldPathPlanner::generateGradientNearPosition(const ignition::math::Vector3d& currentPosition) const {
  ignition::math::Vector2d currentPosition2d {currentPosition.X(), currentPosition.Y()};
  ignition::math::Vector2d deltaX {ExpandingPotentialFieldPathPlanner::h, 0.0};
  ignition::math::Vector2d deltaY {0.0, ExpandingPotentialFieldPathPlanner::h};

  ignition::math::Vector2d point_x_plus {currentPosition2d + deltaX};
  ignition::math::Vector2d point_x_minus {currentPosition2d - deltaX};
  ignition::math::Vector2d point_y_plus {currentPosition2d + deltaY};
  ignition::math::Vector2d point_y_minus {currentPosition2d - deltaY};

  double gradientX = -(this->__generatePotentialAtPoint(point_x_plus) - this->__generatePotentialAtPoint(point_x_minus)) / (2.0*ExpandingPotentialFieldPathPlanner::h);
  double gradientY = -(this->__generatePotentialAtPoint(point_y_plus) - this->__generatePotentialAtPoint(point_y_minus)) / (2.0*ExpandingPotentialFieldPathPlanner::h);
  ignition::math::Vector2d gradient {gradientX, gradientY};
  return gradient;
}
