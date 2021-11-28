#include "AStarPathPlanner.hh"
using namespace gazebo;
using std::vector;


// MARK: - Constructors

AStarPathPlanner::AStarPathPlanner(ignition::math::Vector3d start, ignition::math::Vector3d& target, const ignition::math::AxisAlignedBox actorBoundingBox, const physics::WorldPtr world, const std::vector<std::string>& ignoreModels, const double actorWidth):
  start{start},
  target{target},
  actorWidth{actorWidth} {
  // set actor boundingBox
  this->actorBoundingBox = actorBoundingBox;
  // set obstacleBoundingBoxes
  this->obstacleBoundingBoxes.reserve(world->Models().size());
  const unsigned int modelCount {world->ModelCount()};
  for (unsigned int i = 0; i < modelCount; ++i) {
    const physics::ModelPtr model = world->ModelByIndex(i);
    if (std::find(ignoreModels.begin(), ignoreModels.end(), model->GetName()) == ignoreModels.end()) {
      ignition::math::AxisAlignedBox boundingBox = world->ModelByIndex(i)->CollisionBoundingBox();
      this->obstacleBoundingBoxes.push_back(boundingBox);
      // std::cout << "model added: " << model->GetName() << std::endl;
    }
  }
  // set allNodesInMap
  this->allNodesInMap.reserve(this->obstacleBoundingBoxes.size()*4 + 10);
  for (const auto& boundingBox: this->obstacleBoundingBoxes) {
    const ignition::math::Vector3d min {boundingBox.Min().X() - 0.01, boundingBox.Min().Y() - 0.01, 0.0};
    const ignition::math::Vector3d max {boundingBox.Max().X() + 0.01, boundingBox.Max().Y() + 0.01, 0.0};
    const ignition::math::Vector3d leftDownPosition {min.X(), min.Y(), 0.0};
    const ignition::math::Vector3d leftUpPosition {min.X(), max.Y(), 0.0};
    const ignition::math::Vector3d rightDownPosition {max.X(), min.Y(), 0.0};
    const ignition::math::Vector3d rightUpPosition {max.X(), max.Y(), 0.0};
    this->allNodesInMap.push_back(Node{-1, leftDownPosition, target});
    this->allNodesInMap.push_back(Node{-1, leftUpPosition, target});
    this->allNodesInMap.push_back(Node{-1, rightDownPosition, target});
    this->allNodesInMap.push_back(Node{-1, rightUpPosition, target});
  }
  this->allNodesInMap.push_back(Node{0, start, target});
  this->nodes.reserve(this->allNodesInMap.size() + 10);
  this->nodes.push_back(Node{0, start, target});
  // set nextNode
  Node* startNodePtr = &this->nodes[0];
  this->openList.reserve(this->allNodesInMap.size() + 10);
  this->openList.push_back(startNodePtr->id);
  this->closeList.reserve(this->allNodesInMap.size() + 10);
  this->nextNode = startNodePtr;

  // print openList
  std::cout << "openList: size=" << this->openList.size() << "; ";
  for (const int& id: this->openList) {
    std::cout << id << ", ";
  }
  std::cout << std::endl;
  // print closeList
  std::cout << "closeList: ";
  for (const int& id: this->closeList) {
    std::cout << id << ", ";
  }
  std::cout << std::endl;
  // print allNodes
  std::cout << "nodes: ";
  for (const Node& node: this->nodes) {
    std::cout << node.id << ", ";
  }
  std::cout << std::endl;
  // print allNodes
  std::cout << "allNodesInMap: ";
  for (const Node& node: this->allNodesInMap) {
    std::cout << node << std::endl;
  }
  std::cout << std::endl;
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
ignition::math::Vector3d AStarPathPlanner::generateGradientNearPosition(const ignition::math::Vector3d& currentPosition, const ignition::math::Vector3d& target) {
  // if distance to next node is large, return vector to nextNode.
  if (this->nextNode->getDistanceFrom(currentPosition) >= 0.1)
    return this->nextNode->position - currentPosition;
  // robot has reached nextNode
  std::cout << "robot has reached nextNode" << std::endl;
  Node& currentNode = *(this->nextNode);
  this->__addNodesNearToOpenList(currentNode);
  std::cout << "added nodes near to openList" << std::endl;
  const int nextNodeId = this->__getNextNodeIdToMove();
  this->nextNode = &(this->nodes[nextNodeId]);
  std::cout << "get next node to move" << std::endl;
  // print nextNode, openList, closeList, nodes
  std::cout << "nextNode: " << *(this->nextNode) << std::endl;
  // print openList
  std::cout << "openList: size=" << this->openList.size() << "; ";
  for (const int& id: this->openList) {
    std::cout << id << ", ";
  }
  std::cout << std::endl;
  // print closeList
  std::cout << "closeList: ";
  for (const int& id: this->closeList) {
    std::cout << id << ", ";
  }
  std::cout << std::endl;
  // print allNodes
  std::cout << "nodes: ";
  for (const Node& node: this->nodes) {
    std::cout << node.id << ", ";
  }
  std::cout << std::endl;

  // store debug file
  std::ofstream allNodesInMap_file;
  allNodesInMap_file.open("allNodesInMap.csv", std::ios::out);
  allNodesInMap_file << "id, x, y" << std::endl;
  for (const Node& node: this->allNodesInMap) {
    allNodesInMap_file << node.id << ", " << node.position.X() << "," << node.position.Y() << std::endl;
  }
  allNodesInMap_file.close();

  std::ofstream nodesMap_file;
  nodesMap_file.open("nodesMap.csv", std::ios::out);
  nodesMap_file << "id, x, y" << std::endl;
  for (const Node& node: this->nodes) {
    nodesMap_file << node.id << ", " << node.position.X() << "," << node.position.Y() << std::endl;
  }
  nodesMap_file.close();

  std::ofstream openListMap_file;
  openListMap_file.open("openListMap.csv", std::ios::out);
  openListMap_file << "id, x, y" << std::endl;
  for (const int& id: this->openList) {
    const Node& node = this->nodes[id];
    openListMap_file << node.id << ", " << node.position.X() << "," << node.position.Y() << std::endl;
  }
  openListMap_file.close();

  std::ofstream closeListMap_file;
  closeListMap_file.open("closeListMap.csv", std::ios::out);
  closeListMap_file << "id, x, y" << std::endl;
  for (const int& id: this->openList) {
    const Node& node = this->nodes[id];
    closeListMap_file << node.id << ", " << node.position.X() << "," << node.position.Y() << std::endl;
  }
  closeListMap_file.close();

  std::ofstream startNode_file;
  startNode_file.open("startNode.csv", std::ios::out);
  startNode_file << "id, x, y" << std::endl;
  startNode_file << "0" << ", " << this->start.X() << "," << this->start.Y() << std::endl;
  startNode_file.close();

  std::ofstream targetNode_file;
  targetNode_file.open("targetNode.csv", std::ios::out);
  targetNode_file << "id, x, y" << std::endl;
  targetNode_file << "-" << ", " << this->target.X() << "," << this->target.Y() << std::endl;
  targetNode_file.close();

  const ignition::math::Vector3d gradient {this->nextNode->position.X() - currentPosition.X(), this->nextNode->position.Y() - currentPosition.Y(), 0.0};
  return gradient;
}


// AStarPathPlanner::AStarPathPlanner& operator=(AStarPathPlanner&& source) {
//   this->allNodesInMap = source.allNodesInMap;
//   this->nodes = source.
// }


// MARK: - Private Member Functions

// nextNode
void AStarPathPlanner::__addNodesNearToOpenList(const Node& currentNode) {
  for (Node& potentialNode: this->allNodesInMap) {
    // std::cout << "check if (" << potentialNode.position.X() << ", " << potentialNode.position.Y() << ") is visible from (" << currentNode.position.X() << ", " << currentNode.position.Y() << ")" <<std::endl;

    if (this->__isNodeVisibleFrom(currentNode, potentialNode) == false) {
      std::cout << "potentialNode " << potentialNode << " is not visible from " << currentNode << std::endl;
      continue;
    }
    if (currentNode.parentNodePtr != nullptr && potentialNode.id == currentNode.parentNodePtr->id ) {
      continue;
    }
    // if potentialNode is not created in nodesTank yet, calculate the total cost and insert into nodesTank.
    if (potentialNode.id < 0) {
      // std::cout << "inserting potential node: " << potentialNode.position.X() << ", " << potentialNode.position.Y() << " Into nodes." << std::endl;
      const int nodeCounter = this->nodes.size();
      const Node* const currentNodePtr = &currentNode;
      this->nodes.push_back(Node{
        nodeCounter,// id
        currentNodePtr,// parent node
        potentialNode.position,// position
        this->target,// target
      });
      // std::cout << "currentNode(1) = " << currentNode << std::endl;
      const Node& newNode = this->nodes.back();
      // std::cout << "currentNode(2) = " << currentNode << "with newNode's parent: " << *(newNode.parentNodePtr) <<  std::endl;
      potentialNode.id = nodeCounter;
      // std::cout << "currentNode(3) = " << currentNode << "with newNode's parent: " << *(newNode.parentNodePtr) <<  std::endl;
      this->openList.push_back(nodeCounter);
      // std::cout << "currentNode(4) = " << currentNode << std::endl;
      std::cout << "inserted node: " << newNode << " Into openList. openList becomes size :" << this->openList.size() << std::endl;
    }
    else { // if potential node is already created
      // std::cout << "judging potential node: " << potentialNode.position.X() << ", " << potentialNode.position.Y() << " is in openList or not." << std::endl;
      auto nodeIdPtrInOpenList = std::find_if(this->openList.begin(), this->openList.end(), [&](int openListNodeId) { return openListNodeId == potentialNode.id; });
      // if it is in the open list, compare and update the cost if neccessary
      if (nodeIdPtrInOpenList != this->openList.end()) {
        // std::cout << "potential node: " << potentialNode.position.X() << ", " << potentialNode.position.Y() << " is in openList." << std::endl;
        const bool __nouse = this->nodes[*nodeIdPtrInOpenList].compareAndUpdateCostIfNeccessary(currentNode);
      } else {// if it is in the close list
        // std::cout << "potential node: " << potentialNode.position.X() << ", " << potentialNode.position.Y() << " is in closeList." << std::endl;
        auto nodeIdPtrInCloseList = std::find_if(this->closeList.begin(), this->closeList.end(), [&](int closeListNodeId) { return closeListNodeId == potentialNode.id; });
        const int nodeId = *nodeIdPtrInCloseList;
        const bool didUpdate = this->nodes[nodeId].compareAndUpdateCostIfNeccessary(currentNode);
        if (didUpdate == true) {
          this->openList.push_back(nodeId);
          this->closeList.erase(nodeIdPtrInCloseList);
        }
      }
    }
    // std::cout << "potential node id=" << potentialNode.id << " : " << potentialNode.position.X() << ", " << potentialNode.position.Y() << " is processed." << std::endl;
  }
}


int AStarPathPlanner::__getNextNodeIdToMove() {
  // https://cpprefjp.github.io/reference/algorithm/sort.html
  std::sort(this->openList.begin(), this->openList.end(), [&](int leftId, int rightId) { return this->nodes[leftId].totalCost > this->nodes[rightId].totalCost;});
  // https://cpprefjp.github.io/reference/vector/vector.html
  const int nextNodeId = this->openList.back();
  this->openList.pop_back();
  this->closeList.push_back(nextNodeId);
  return nextNodeId;
}


bool AStarPathPlanner::__isNodeVisibleFrom(const Node& fromNode, const Node& toNode) const {
  // loop over everu obstacles
  for (const auto& boundingBox: this->obstacleBoundingBoxes) {
    // get box edges
    const ignition::math::Vector3d leftDown {boundingBox.Min().X(), boundingBox.Min().Y(), 0.0};
    const ignition::math::Vector3d leftUp {boundingBox.Min().X(), boundingBox.Max().Y(), 0.0};
    const ignition::math::Vector3d rightDown {boundingBox.Max().X(), boundingBox.Min().Y(), 0.0};
    const ignition::math::Vector3d rightUp {boundingBox.Max().X(), boundingBox.Max().Y(), 0.0};
    // center
    if (AStarPathPlanner::__didIntersect(leftDown, rightUp, fromNode.position, toNode.position) || AStarPathPlanner::__didIntersect(leftUp, rightDown, fromNode.position, toNode.position)) {
      // std::cout << "result -> false" << std::endl;
      return false;
    }
    // // right
    // const ignition::math::Vector3d lineVector {toNode.position - fromNode.position};
    // ignition::math::Vector3d verticalVector {-lineVector.Y(), lineVector.X(), 0.0};
    // verticalVector.Normalize();
    // const ignition::math::Vector3d verticalVector_realDistance = verticalVector.Normalize() * this->actorWidth / 2.0;
    // const ignition::math::Vector3d fromNode_right = fromNode.position + verticalVector_realDistance;
    // const ignition::math::Vector3d toNode_right = toNode.position + verticalVector_realDistance;
    // if (AStarPathPlanner::__didIntersect(leftDown, rightUp, fromNode_right, toNode_right) || AStarPathPlanner::__didIntersect(leftUp, rightDown, fromNode_right, toNode_right)) {
    //   return false;
    // }
    // const ignition::math::Vector3d fromNode_left = fromNode.position - verticalVector_realDistance;
    // const ignition::math::Vector3d toNode_left = toNode.position - verticalVector_realDistance;
    // if (AStarPathPlanner::__didIntersect(leftDown, rightUp, fromNode_left, toNode_left) || AStarPathPlanner::__didIntersect(leftUp, rightDown, fromNode_left, toNode_left)) {
    //   return false;
    // }
  }
  // std::cout << "result -> true" << std::endl;
  return true;
}


// MARK: - Private Static Functions

inline double AStarPathPlanner::__getJudgeNumber(const ignition::math::Vector3d& basePoint1, const ignition::math::Vector3d& basePoint2, const ignition::math::Vector3d& testPoint) {
  return (basePoint2.Y() - basePoint1.Y())/(basePoint2.X() - basePoint1.Y()) * (testPoint.X() - basePoint1.X()) + basePoint1.Y() - testPoint.Y();
}


inline bool AStarPathPlanner::__didIntersect(const ignition::math::Vector3d& group1Point1, const ignition::math::Vector3d& group1Point2, const ignition::math::Vector3d& group2Point1, const ignition::math::Vector3d& group2Point2) {
  if (AStarPathPlanner::__getJudgeNumber(group1Point1, group1Point2, group2Point1) * AStarPathPlanner::__getJudgeNumber(group1Point1, group1Point2, group2Point2) > 0.0)
    return false;
  else if (AStarPathPlanner::__getJudgeNumber(group2Point1, group2Point2, group1Point1) * AStarPathPlanner::__getJudgeNumber(group2Point1, group2Point2, group1Point2) > 0.0)
    return false;
  else
    return true;
}
