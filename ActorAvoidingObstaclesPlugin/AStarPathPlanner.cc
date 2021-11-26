#include "AStarPathPlanner.hh"
using namespace gazebo;
using std::vector;


// MARK: - Constructors

void AStarPathPlanner::lazyConstructor(ignition::math::Vector3d start, ignition::math::Vector3d& target) {
  this->start = start;
  this->target = target;
}


// MARK: - Public Functions

// update boundingBoxes
void AStarPathPlanner::updateModels(const ignition::math::AxisAlignedBox actorBoundingBox, const physics::WorldPtr world, const std::vector<std::string>& ignoreModels) {
  this->actorBoundingBox = actorBoundingBox;
  const unsigned int modelCount {world->ModelCount()};
  for (unsigned int i = 0; i < modelCount; ++i) {
    const physics::ModelPtr model = world->ModelByIndex(i);
    if (std::find(ignoreModels.begin(), ignoreModels.end(), model->GetName()) == ignoreModels.end())
      this->obstacleBoundingBoxes.push_back(world->ModelByIndex(i)->BoundingBox());
  }
}

// generate gradient near point
ignition::math::Vector3d AStarPathPlanner::generateGradientNearPosition(const ignition::math::Vector3d& currentPosition, const ignition::math::Vector3d& target) const {
  // if distance to next node is large, return vector to nextNode.
  if (this->nextNode.getDistanceFrom(currentPosition) >= 0.3)
    return nextNode - currentPosition;
  // robot has reached nextNode
  this->__addNodesNearToOpenList(this->nextNode);
  this->nextNode = this->__getNextNodeToMove();
  return this->nextNode.position - currentPosition;
}


// MARK: - Private Member Functions

// nextNode
void AStarPathPlanner::__addNodesNearToOpenList(const Node& currentNode) {
  for (Node& potentialNode: this->allNodesInMap) {
    // if the potentialNode is visible from currentNode and it's not the parent node
    if ( this->__isNodeVisibleFrom(currentNode, potentialNode) == true && potentialNode != *(this->currentNode.parentNodePtr) ) {
      // if potentialNode is not created in nodesTank yet, calculate the total cost and insert into nodesTank.
      if (potentialNode.id < 0) {
        Node newNode {Node(
          this->nodeCounter,// id
          &currentNode,// parent node
          potentialNode.position,// position
          this->target,// target
        )};
        this->nodes.push_back(newNode);
        this->nodeCounter += 1;
        this->openList.push_back(newNode);
        potential.id = newNode.id;
      }
      else { // if potential node is already created
        Node* nodePtrInOpenList = std::find(this->openList.begin(), this->openList.end(), this->potentialNode);
        // if it is in the open list, compare and update the cost if neccessary
        if (nodePtrInOpenList != this->openList.end()) {
          const bool __nouse = nodePtrInOpenList->compareAndUpdateCostIfNeccessary(currentNode);
        }
        else {// if it is in the close list
          Node* nodePtrInCloseList = std::find(this->closeList.begin(), this->closeList.end(), this->potentialNode);
          const bool didUpdate = nodePtrInCloseList->compareAndUpdateCostIfNeccessary(currentNode);
          if (didUpdate == true) {
            this->openList.push_back(*nodePtrInCloseList);
            this->closeList.erase(nodePtrInCloseList);
          }
        }
      }
    }
  }
}


Node& AStarPathPlanner::__getNextNodeToMove() {
  // https://cpprefjp.github.io/reference/algorithm/sort.html
  std::sort(this->openList.begin(), this->openList.end(), [&](Node& left, Node& right) { return left.totalCost > right.totalCost;});
  // https://cpprefjp.github.io/reference/vector/vector.html
  Node& nextNode {this->openList[this->openList.size()-1]};
  this->openList.pop_back();
  this->closeList.push_back(nextNode);
  return nextNode;
}


bool AStarPathPlanner::__isNodeVisibleFrom(const Node& fromNode, const Node& toNode) const {
  // define judge functions
  auto didIntersect = [&] (ignition::math::Vector3d& group1Point1, ignition::math::Vector3d& group1Point2, ignition::math::Vector3d& group2Point1, ignition::math::Vector3d& group2Point2) -> bool {
    auto getJudgeNumber = [&] (ignition::math::Vector3d& basePoint1, ignition::math::Vector3d& basePoint2, ignition::math::Vector3d& testPoint1) -> double {
      return (basePoint2.Y() - basePoint2.Y())/(basePoint2.X() - basePoint1.Y()) * (testPoint.X() - basePoint1.X()) + basePoint1.Y() - testPoint.Y();
    };
    if (getJudgeNumber(group1Point1, group1Point2, group2Point1) * getJudgeNumber(group1Point1, group1Point2, group2Point2) > 0.0)
      return false;
    else if (getJudgeNumber(group2Point1, group2Point2, group1Point1) * getJudgeNumber(group2Point1, group2Point2, group1Point2) > 0.0)
      return false;
    else
      return true;
  };
  // loop over everu obstacles
  for (const auto& boundingBox: this->obstacleBoundingBoxes) {
    // get box edges
    const ignition::math::Vector3d leftDown {boundingBox.Min().X(), boundingBox.Min().Y(), 0.0};
    const ignition::math::Vector3d leftUp {boundingBox.Min().X(), boundingBox.Max().Y(), 0.0};
    const ignition::math::Vector3d rightDown {boundingBox.Max().X(), boundingBox.Min().Y(), 0.0};
    const ignition::math::Vector3d rightUp {boundingBox.Max().X(), boundingBox.Max().Y(), 0.0};
    if (didIntersect(leftDown, rightUp, fromNode.position, toNode.position) || didIntersect(leftUp, rightDown, fromNode.position, toNode.position))
      return false;
  }
  return true;
}


inline double AStarPathPlanner::__getJudgeNumber(ignition::math::Vector3d& basePoint1, ignition::math::Vector3d& basePoint2, ignition::math::Vector3d& testPoint1) {
  return (basePoint2.Y() - basePoint2.Y())/(basePoint2.X() - basePoint1.Y()) * (testPoint.X() - basePoint1.X()) + basePoint1.Y() - testPoint.Y();
}


inline bool AStarPathPlanner::__didIntersect(ignition::math::Vector3d& group1Point1, ignition::math::Vector3d& group1Point2, ignition::math::Vector3d& group2Point1, ignition::math::Vector3d& group2Point2) {
  if (AStarPathPlanner::__getJudgeNumber(group1Point1, group1Point2, group2Point1) * AStarPathPlanner::__getJudgeNumber(group1Point1, group1Point2, group2Point2) > 0.0)
    return false;
  else if (AStarPathPlanner::__getJudgeNumber(group2Point1, group2Point2, group1Point1) * AStarPathPlanner::__getJudgeNumber(group2Point1, group2Point2, group1Point2) > 0.0)
    return false;
  else
    return true;
}
