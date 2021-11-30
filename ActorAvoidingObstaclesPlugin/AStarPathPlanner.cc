#include "AStarPathPlanner.hh"
using namespace gazebo;
using std::vector;


// MARK: - Constructors

AStarPathPlanner::AStarPathPlanner(ignition::math::Vector3d start, ignition::math::Vector3d& target, const ignition::math::AxisAlignedBox actorBoundingBox, const physics::WorldPtr world, const std::vector<std::string>& ignoreModels, const double actorWidth, const bool isCheating):
  start{start},
  target{target},
  actorWidth{actorWidth},
  // openList{vector<int>{}},
  // closeList{vector<int>{}},
  // midwayNodeIds{vector<int>{}},
  // ancestorIds_nextNode{vector<int>{}},
  // searchedMinPathIds{vector<int>{}},
  openList{},
  closeList{},
  midwayNodeIds{},
  ancestorIds_nextNode{},
  searchedMinPathIds{},
  isCheating{isCheating} {
  std::cout << "Entered AStarPathPlanner(...)" << std::endl;
  // set actor boundingBox
  this->actorBoundingBox = actorBoundingBox;
  this->obstacleBoundingBoxes.reserve(world->ModelCount());
  this->allNodesInMap.reserve((world->ModelCount())*4 + 10);
  this->nodes.reserve((world->ModelCount())*4 + 10);
  this->openList.reserve((world->ModelCount())*4 + 10);
  this->closeList.reserve((world->ModelCount())*4 + 10);
  this->midwayNodeIds.reserve(40);
  this->ancestorIds_nextNode.reserve(30);
  if (isCheating == true) {
    this->searchedMinPathIds.reserve(30);
  }
  std::cout << "Vectors allocated." << std::endl;
  // set obstacleBoundingBoxes
  const unsigned int modelCount {world->ModelCount()};
  for (unsigned int i = 0; i < modelCount; ++i) {
    const physics::ModelPtr model = world->ModelByIndex(i);
    if (std::find(ignoreModels.begin(), ignoreModels.end(), model->GetName()) == ignoreModels.end()) {
      ignition::math::AxisAlignedBox boundingBox = world->ModelByIndex(i)->CollisionBoundingBox();
      std::cout << "model added: " << model->GetName() << ": " << boundingBox.Min() << ", " << boundingBox.Max() << std::endl;
      this->obstacleBoundingBoxes.push_back(boundingBox);
    }
  }
  std::cout << "obstacleBoundingBoxes set." << std::endl;
  // set allNodesInMap
  for (const auto& boundingBox: this->obstacleBoundingBoxes) {
    const ignition::math::Vector3d min {boundingBox.Min().X() - AStarPathPlanner::deltaFromCollision, boundingBox.Min().Y() - AStarPathPlanner::deltaFromCollision, 0.0};
    const ignition::math::Vector3d max {boundingBox.Max().X() + AStarPathPlanner::deltaFromCollision, boundingBox.Max().Y() + AStarPathPlanner::deltaFromCollision, 0.0};
    const ignition::math::Vector3d leftDownPosition {min.X(), min.Y(), 0.0};
    const ignition::math::Vector3d leftUpPosition {min.X(), max.Y(), 0.0};
    const ignition::math::Vector3d rightDownPosition {max.X(), min.Y(), 0.0};
    const ignition::math::Vector3d rightUpPosition {max.X(), max.Y(), 0.0};
    if (AStarPathPlanner::__pointIsReachable(leftDownPosition, this->actorWidth, this->obstacleBoundingBoxes) == true && AStarPathPlanner::__pointIsNotNearOtherNodes(leftDownPosition, this->allNodesInMap) == true) {
      this->allNodesInMap.push_back(Node{-1, leftDownPosition, target});
    }
    if (AStarPathPlanner::__pointIsReachable(leftUpPosition, this->actorWidth, this->obstacleBoundingBoxes) == true && AStarPathPlanner::__pointIsNotNearOtherNodes(leftUpPosition, this->allNodesInMap) == true) {
      this->allNodesInMap.push_back(Node{-1, leftUpPosition, target});
    }
    if (AStarPathPlanner::__pointIsReachable(rightDownPosition, this->actorWidth, this->obstacleBoundingBoxes) == true && AStarPathPlanner::__pointIsNotNearOtherNodes(rightDownPosition, this->allNodesInMap) == true) {
      this->allNodesInMap.push_back(Node{-1, rightDownPosition, target});
    }
    if (AStarPathPlanner::__pointIsReachable(rightUpPosition, this->actorWidth, this->obstacleBoundingBoxes) == true && AStarPathPlanner::__pointIsNotNearOtherNodes(rightUpPosition, this->allNodesInMap) == true) {
      this->allNodesInMap.push_back(Node{-1, rightUpPosition, target});
    }
  }
  this->allNodesInMap.push_back(Node{0, start, target});
  this->allNodesInMap.push_back(Node{-1, target, target});
  std::cout << "allNodesInMap set." << std::endl;
  this->nodes.push_back(Node{0, start, target});
  // set nextNode
  Node* startNodePtr = &this->nodes[0];
  this->openList.push_back(startNodePtr->id);

  this->nextNode = startNodePtr;
  std::cout << "AStar Path Planning Generated.";

  // // print openList
  // std::cout << "openList: size=" << this->openList.size() << "; ";
  // for (const int& id: this->openList) {
  //   std::cout << this->nodes[id] << " heuricost=" << this->nodes[id].heuristicCostToTarget << ";  ";
  // }
  // std::cout << std::endl;
  // // print closeList
  // std::cout << "closeList: ";
  // for (const int& id: this->closeList) {
  //   std::cout << id << ", ";
  // }
  // std::cout << std::endl;
  // // print allNodes
  // std::cout << "nodes: ";
  // for (const Node& node: this->nodes) {
  //   std::cout << node.id << ", ";
  // }
  // std::cout << std::endl;
  // // print allNodes
  // std::cout << "allNodesInMap: ";
  // for (const Node& node: this->allNodesInMap) {
  //   std::cout << node << std::endl;
  // }
  // std::cout << std::endl;
  //
  // std::ofstream trajectoryNodes_file;
  // trajectoryNodes_file.open("trajectoryNodes.csv", std::ios::out);
  // trajectoryNodes_file << "id, x, y" << std::endl;
  // trajectoryNodes_file.close();
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
  if (this->nextNode->getDistanceFrom(currentPosition) >= 0.1) {
    ignition::math::Vector3d&& gradient = this->nextNode->position - currentPosition;
    gradient.Normalize();
    return gradient;
  }

  // if any midway node exists, pop from back.
  if (this->midwayNodeIds.size() > 0) {
    const int nextNodeId = this->midwayNodeIds.front();
    this->nextNode = &(this->nodes[nextNodeId]);
    this->midwayNodeIds.erase(this->midwayNodeIds.begin());
    ignition::math::Vector3d&& gradient = this->nextNode->position - currentPosition;
    gradient.Normalize();
    return gradient;
  }
  // robot has reached nextNode
  // std::cout << "robot has reached nextNode" << std::endl;
  Node& currentNode = *(this->nextNode);
  this->__addNodesNearToOpenList(currentNode);
  // std::cout << "added nodes near to openList" << std::endl;

  const int nextNodeId = this->__getNextNodeIdToMove(currentNode);
  this->nextNode = &(this->nodes[nextNodeId]);
  // std::cout << "get next node to move" << std::endl;

  // // print nextNode, openList, closeList, nodes
  // std::cout << "nextNode: " << *(this->nextNode) << std::endl;
  // // print openList
  // std::cout << "openList: size=" << this->openList.size() << "; ";
  // for (const int& id: this->openList) {
  //   const Node& node = this->nodes[id];
  //   std::cout << node << ", ";
  //   std::cout << "parentNode: " << *(node.parentNodePtr) << ", ";
  //   std::cout << "start->parent: "<< node.parentNodePtr->actualCostFromStart << ", ";
  //   std::cout << "parent->node: "<< (node.position - node.parentNodePtr->position).Length() << ", ";
  //   std::cout << "node->target: "<< node.heuristicCostToTarget << ", ";
  //   std::cout << "total: "<< node.totalCost << std::endl;
  // }
  // std::cout << std::endl;
  // // print closeList
  // std::cout << "closeList: ";
  // for (const int& id: this->closeList) {
  //   std::cout << id << ", ";
  // }
  // std::cout << std::endl;
  // // print allNodes
  // std::cout << "nodes: ";
  // for (const Node& node: this->nodes) {
  //   std::cout << node.id << ", ";
  // }
  // std::cout << std::endl;
  //
  // // store debug file
  // std::ofstream allNodesInMap_file;
  // allNodesInMap_file.open("allNodesInMap.csv", std::ios::out);
  // allNodesInMap_file << "id, x, y" << std::endl;
  // for (const Node& node: this->allNodesInMap) {
  //   allNodesInMap_file << node.id << ", " << node.position.X() << "," << node.position.Y() << std::endl;
  // }
  // allNodesInMap_file.close();
  //
  // std::ofstream nodesMap_file;
  // nodesMap_file.open("nodesMap.csv", std::ios::out);
  // nodesMap_file << "id, x, y" << std::endl;
  // for (const Node& node: this->nodes) {
  //   nodesMap_file << node.id << ", " << node.position.X() << "," << node.position.Y() << std::endl;
  // }
  // nodesMap_file.close();
  //
  // std::ofstream openListMap_file;
  // openListMap_file.open("openListMap.csv", std::ios::out);
  // openListMap_file << "id, x, y" << std::endl;
  // for (const int& id: this->openList) {
  //   const Node& node = this->nodes[id];
  //   openListMap_file << node.id << ", " << node.position.X() << "," << node.position.Y() << std::endl;
  // }
  // openListMap_file.close();
  //
  // std::ofstream closeListMap_file;
  // closeListMap_file.open("closeListMap.csv", std::ios::out);
  // closeListMap_file << "id, x, y" << std::endl;
  // for (const int& id: this->openList) {
  //   const Node& node = this->nodes[id];
  //   closeListMap_file << node.id << ", " << node.position.X() << "," << node.position.Y() << std::endl;
  // }
  // closeListMap_file.close();
  //
  // std::ofstream startNode_file;
  // startNode_file.open("startNode.csv", std::ios::out);
  // startNode_file << "id, x, y" << std::endl;
  // startNode_file << "0" << ", " << this->start.X() << "," << this->start.Y() << std::endl;
  // startNode_file.close();
  //
  // std::ofstream targetNode_file;
  // targetNode_file.open("targetNode.csv", std::ios::out);
  // targetNode_file << "id, x, y" << std::endl;
  // targetNode_file << "-" << ", " << this->target.X() << "," << this->target.Y() << std::endl;
  // targetNode_file.close();
  //
  // std::ofstream currentNode_file;
  // currentNode_file.open("currentNode.csv", std::ios::out);
  // currentNode_file << "id, x, y" << std::endl;
  // currentNode_file << "-" << ", " << currentPosition.X() << "," << currentPosition.Y() << std::endl;
  // currentNode_file.close();
  //
  // std::ofstream trajectoryNodes_file;
  // trajectoryNodes_file.open("trajectoryNodes.csv", std::ios::app);
  // trajectoryNodes_file << this->nextNode->id << ", " << this->nextNode->position.X() << "," << this->nextNode->position.Y() << std::endl;
  // trajectoryNodes_file.close();

  if (this->midwayNodeIds.size() > 0) {
    const int nextNodeId = this->midwayNodeIds.front();
    this->nextNode = &(this->nodes[nextNodeId]);
    ignition::math::Vector3d&& gradient = this->nextNode->position - currentPosition;
    gradient.Normalize();
    return gradient;
  }
  ignition::math::Vector3d gradient {this->nextNode->position.X() - currentPosition.X(), this->nextNode->position.Y() - currentPosition.Y(), 0.0};
  gradient.Normalize();
  return gradient;
}


ignition::math::Vector3d AStarPathPlanner::generateGradientNearPosition_cheatMode(const ignition::math::Vector3d& currentPosition) {
  if (this->nextNode->getDistanceFrom(currentPosition) >= 0.2) {
    ignition::math::Vector3d&& gradient = this->nextNode->position - currentPosition;
    gradient.Normalize();
    return gradient;
  }
  // if any midway node exists, pop from back.
  const int nextNodeId = this->searchedMinPathIds.back();
  this->nextNode = &(this->nodes[nextNodeId]);
  this->searchedMinPathIds.pop_back();
  ignition::math::Vector3d&& gradient = this->nextNode->position - currentPosition;
  gradient.Normalize();
  return gradient;
}


void AStarPathPlanner::generatePathInCheatMode() {
  this->searchedMinPathIds.clear();
  const Node* currentNodePtr = &(this->nodes[0]);
  // get minPathIds
  while (currentNodePtr->getDistanceFrom(this->target) > AStarPathPlanner::distanceAsReached) {
    const Node& currentNode = *currentNodePtr;
    // this->searchedMinPathIds.push_back(currentNode.id);
    this->__addNodesNearToOpenList(currentNode);
    const int nextNodeId = this->__getNextNodeIdToMove(currentNode);
    currentNodePtr = &(this->nodes[nextNodeId]);
  }
  // add min route
  while (currentNodePtr->id != 0) {
    this->searchedMinPathIds.push_back(currentNodePtr->id);
    currentNodePtr = currentNodePtr->parentNodePtr;
  }
  currentNodePtr = nullptr;
  // clear allNodesInMap
  this->allNodesInMap.clear();
  this->allNodesInMap.shrink_to_fit();
  // // draw minPath
  // std::ofstream startNode_file;
  // startNode_file.open("startNode.csv", std::ios::out);
  // startNode_file << "id, x, y" << std::endl;
  // startNode_file << "0" << ", " << this->start.X() << "," << this->start.Y() << std::endl;
  // startNode_file.close();
  //
  // std::ofstream targetNode_file;
  // targetNode_file.open("targetNode.csv", std::ios::out);
  // targetNode_file << "id, x, y" << std::endl;
  // targetNode_file << "-" << ", " << this->target.X() << "," << this->target.Y() << std::endl;
  // targetNode_file.close();
  //
  // std::ofstream searchedMinPath_file;
  // searchedMinPath_file.open("searchedMinPath.csv", std::ios::out);
  // searchedMinPath_file << "id, x, y" << std::endl;
  // for (auto p = this->searchedMinPathIds.end(); p != this->searchedMinPathIds.begin(); --p) {
  //   const Node& node = this->nodes[(*p)];
  //   searchedMinPath_file << node.id << ", " << node.position.X() << "," << node.position.Y() << std::endl;
  // }
  // searchedMinPath_file.close();
  // set nextNode
  const int nextNodeId = this->searchedMinPathIds.back();
  this->nextNode = &(this->nodes[nextNodeId]);
  this->searchedMinPathIds.pop_back();
}


// AStarPathPlanner::AStarPathPlanner& operator=(AStarPathPlanner&& source) {
//   this->allNodesInMap = source.allNodesInMap;
//   this->nodes = source.
// }


// MARK: - Private Member Functions

// nextNode
void AStarPathPlanner::__addNodesNearToOpenList(const Node& currentNode) {
  for (Node& potentialNode: this->allNodesInMap) {
    // std::cout << std::endl << "check if (" << potentialNode.position.X() << ", " << potentialNode.position.Y() << ") is visible from (" << currentNode.position.X() << ", " << currentNode.position.Y() << ")" <<std::endl;

    if (this->__isNodeVisibleFrom(currentNode, potentialNode) == false) {
      // std::cout << "potentialNode " << potentialNode << " is not visible from " << currentNode << std::endl;
      continue;
    }
    if (currentNode.parentNodePtr != nullptr && potentialNode.id == currentNode.parentNodePtr->id ) {
      continue;
    }
    // std::cout << "potentialNode " << potentialNode << " is visible from " << currentNode << std::endl;
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
      // std::cout << "inserted node: " << newNode << " Into openList. openList becomes size :" << this->openList.size() << std::endl;
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


int AStarPathPlanner::__getNextNodeIdToMove(const Node& currentNode) {
  // https://cpprefjp.github.io/reference/algorithm/sort.html
  std::sort(this->openList.begin(), this->openList.end(), [&](int leftId, int rightId) { return this->nodes[leftId].totalCost > this->nodes[rightId].totalCost;});
  // https://cpprefjp.github.io/reference/vector/vector.html
  const int nextNodeId = this->openList.back();
  this->openList.pop_back();
  this->closeList.push_back(nextNodeId);

  if (this->__isNodeVisibleFrom(currentNode, this->nodes[nextNodeId])) {
    return nextNodeId;
  }

  // std::cout << "start calculating midway nodes" << std::endl;
  // if not visible, add midway nodes
  this->ancestorIds_nextNode.clear();
  int parentNodeId = nextNodeId;
  while(this->nodes[parentNodeId].parentNodePtr->id != 0) {
    this->ancestorIds_nextNode.push_back(parentNodeId);
    parentNodeId = this->nodes[parentNodeId].parentNodePtr->id;
  }
  this->ancestorIds_nextNode.push_back(0);

  this->midwayNodeIds.clear();
  parentNodeId = currentNode.parentNodePtr->id;
  // if that parentNodeId is not a common ancestor,
  while(std::find(this->ancestorIds_nextNode.begin(), this->ancestorIds_nextNode.end(), parentNodeId) == this->ancestorIds_nextNode.end()) {
    this->midwayNodeIds.push_back(parentNodeId);
    parentNodeId = this->nodes[parentNodeId].parentNodePtr->id;
  }
  auto positionPtr = std::find(this->ancestorIds_nextNode.begin(), this->ancestorIds_nextNode.end(), parentNodeId);
  for (; positionPtr != this->ancestorIds_nextNode.begin(); --positionPtr) {
    const int _id = *positionPtr;
    this->midwayNodeIds.push_back(_id);
  }
  this->midwayNodeIds.push_back(this->ancestorIds_nextNode[0]);
  // std::cout << "from " << currentNode << " to " << this->nodes[nextNodeId] << " needs midway nodes: [ ";
  // for (const auto nodeId: this->midwayNodeIds) {
  //   std::cout << nodeId << ", ";
  // }
  // std::cout << "]" << std::endl << std::endl;

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
    // if (AStarPathPlanner::__didIntersect(leftDown, rightDown, fromNode.position, toNode.position) || AStarPathPlanner::__didIntersect(rightDown, rightUp, fromNode.position, toNode.position) || AStarPathPlanner::__didIntersect(rightUp, leftUp, fromNode.position, toNode.position) || AStarPathPlanner::__didIntersect(leftUp, leftDown, fromNode.position, toNode.position)) {
    //   return false;
    // }
    if (AStarPathPlanner::__didIntersect(leftDown, rightDown, fromNode.position, toNode.position) == true) {
      // std::cout << "nodes: " << fromNode << " and " << toNode << " intersects with line: (leftDown)" << leftDown << " to (rightDown)" << rightDown << std::endl;
      return false;
    }
    if (AStarPathPlanner::__didIntersect(rightDown, rightUp, fromNode.position, toNode.position) == true) {
      // std::cout << "nodes: " << fromNode << " and " << toNode << " intersects with line: (rightDown)" << rightDown << " to (rightUp)" << rightUp << std::endl;
      return false;
    }
    if (AStarPathPlanner::__didIntersect(rightUp, leftUp, fromNode.position, toNode.position) == true) {
      // std::cout << "nodes: " << fromNode << " and " << toNode << " intersects with line: (rightUp)" << rightUp << " to (leftUp)" << leftUp << std::endl;
      return false;
    }
    if (AStarPathPlanner::__didIntersect(leftUp, leftDown, fromNode.position, toNode.position) == true) {
      // std::cout << "nodes: " << fromNode << " and " << toNode << " intersects with line: (leftUp)" << leftUp << " to (leftDown)" << leftDown << std::endl;
      return false;
    }
    if (AStarPathPlanner::__didIntersect(leftDown, rightUp, fromNode.position, toNode.position) == true) {
      // std::cout << "nodes: " << fromNode << " and " << toNode << " intersects with line: (leftDown)" << leftDown << " to (rightUp)" << rightUp << std::endl;
      return false;
    }
    if (AStarPathPlanner::__didIntersect(rightDown, leftUp, fromNode.position, toNode.position) == true) {
      // std::cout << "nodes: " << fromNode << " and " << toNode << " intersects with line: (rightDown)" << rightDown << " to (leftUp)" << leftUp << std::endl;
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
  // return (basePoint2.Y() - basePoint1.Y())/(basePoint2.X() - basePoint1.X()) * (testPoint.X() - basePoint1.X()) + basePoint1.Y() - testPoint.Y();
  return (basePoint2.Y() - basePoint1.Y())*(testPoint.X() - basePoint1.X()) - (testPoint.Y() - basePoint1.Y())*(basePoint2.X() - basePoint1.X());
}


inline bool AStarPathPlanner::__didIntersect(const ignition::math::Vector3d& group1Point1, const ignition::math::Vector3d& group1Point2, const ignition::math::Vector3d& group2Point1, const ignition::math::Vector3d& group2Point2) {
  if (AStarPathPlanner::__getJudgeNumber(group1Point1, group1Point2, group2Point1) * AStarPathPlanner::__getJudgeNumber(group1Point1, group1Point2, group2Point2) > 0.0)
    return false;
  else if (AStarPathPlanner::__getJudgeNumber(group2Point1, group2Point2, group1Point1) * AStarPathPlanner::__getJudgeNumber(group2Point1, group2Point2, group1Point2) > 0.0)
    return false;
  else
    return true;
}


bool AStarPathPlanner::__pointIsReachable(const ignition::math::Vector3d& point, const double actorWidth, const vector<ignition::math::AxisAlignedBox>& boxes) {
  static constexpr double epsilon {0.000001};
  const double boxHeight_min = boxes[0].Min().Z();
  const double boxHeight_max = boxes[0].Max().Z();

  const ignition::math::Vector3d leftDown_minCorner {point.X() - epsilon - actorWidth, point.Y() - epsilon - actorWidth, boxHeight_min};
  const ignition::math::Vector3d leftDown_maxCorner {point.X() - epsilon, point.Y() - epsilon, boxHeight_max};
  const ignition::math::AxisAlignedBox leftDownBox {leftDown_minCorner, leftDown_maxCorner};
  bool isLeftDownBoxAvailable {true};
  for (const auto& box: boxes) {
    if (box.Intersects(leftDownBox) == true) {
      isLeftDownBoxAvailable = false;
    }
  }
  if (isLeftDownBoxAvailable == true) {
    return true;
  }

  const ignition::math::Vector3d leftUp_minCorner {point.X() - epsilon - actorWidth, point.Y() + epsilon, boxHeight_min};
  const ignition::math::Vector3d leftUp_maxCorner {point.X() - epsilon, point.Y() + epsilon + actorWidth, boxHeight_max};
  const ignition::math::AxisAlignedBox leftUpBox {leftUp_minCorner, leftUp_maxCorner};
  bool isLeftUpBoxAvailable {true};
  for (const auto& box: boxes) {
    if (box.Intersects(leftUpBox) == true) {
      isLeftUpBoxAvailable = false;
    }
  }
  if (isLeftUpBoxAvailable == true) {
    return true;
  }

  const ignition::math::Vector3d rightDown_minCorner {point.X() + epsilon, point.Y() - epsilon - actorWidth, boxHeight_min};
  const ignition::math::Vector3d rightDown_maxCorner {point.X() + epsilon + actorWidth, point.Y() - epsilon, boxHeight_max};
  const ignition::math::AxisAlignedBox rightDownBox {rightDown_minCorner, rightDown_maxCorner};
  bool isRightDownBoxAvailable {true};
  for (const auto& box: boxes) {
    if (box.Intersects(rightDownBox) == true) {
      isRightDownBoxAvailable = false;
    }
  }
  if (isRightDownBoxAvailable == true) {
    return true;
  }

  const ignition::math::Vector3d rightUp_minCorner {point.X() + epsilon, point.Y() + epsilon, boxHeight_min};
  const ignition::math::Vector3d rightUp_maxCorner {point.X() + epsilon + actorWidth, point.Y() + epsilon + actorWidth, boxHeight_max};
  const ignition::math::AxisAlignedBox rightUpBox {rightUp_minCorner, rightUp_maxCorner};
  bool isRightUpBoxAvailable {true};
  for (const auto& box: boxes) {
    if (box.Intersects(rightUpBox) == true) {
      isRightUpBoxAvailable = false;
    }
  }
  if (isRightUpBoxAvailable == true) {
    return true;
  }

  return false;
}


bool AStarPathPlanner::__pointIsNotNearOtherNodes(const ignition::math::Vector3d& point, const vector<Node>& otherNodes) {
  for (const Node& otherNode: otherNodes) {
    if (otherNode.getDistanceFrom(point) < AStarPathPlanner::distanceAsReached) {
      return false;
    }
  }
  return true;
}
