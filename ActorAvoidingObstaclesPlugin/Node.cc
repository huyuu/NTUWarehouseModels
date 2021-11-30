#include "Node.hh"

using namespace gazebo;


// MARK: - Constructor

Node::Node(): isOpen{false}, id{-1}, parentNodePtr{nullptr}, position{ignition::math::Vector3d{}}, heuristicCostToTarget{Node::bigNumber}, totalCost{Node::bigNumber} {}

/// Default all terms constructor
Node::Node(bool isOpen, int id, const Node* parentNodePtr, ignition::math::Vector3d position, double heuristicCostToTarget, double totalCost): isOpen{isOpen}, id{id}, parentNodePtr{parentNodePtr}, position{position}, heuristicCostToTarget{heuristicCostToTarget}, totalCost{totalCost} {
  this->actualCostFromStart = this->totalCost - this->heuristicCostToTarget;
}


/// Constructor for new start point
Node::Node(int id, ignition::math::Vector3d position, ignition::math::Vector3d& target):
  isOpen{true},
  id{id},
  parentNodePtr{nullptr},
  position{position},
  heuristicCostToTarget{Node::bigNumber},
  totalCost{Node::bigNumber} {
    const ignition::math::Vector3d vectorFromStartToTarget {target - position};
    this->heuristicCostToTarget = vectorFromStartToTarget.Length();
    this->totalCost = this->heuristicCostToTarget;
    this->actualCostFromStart = this->totalCost - this->heuristicCostToTarget;
}


/// Constructor for new normal points
Node::Node(int id, const Node* const parentNodePtr, ignition::math::Vector3d EstimatedNewNodePosition, ignition::math::Vector3d& target):
  isOpen{true},
  id{id},
  parentNodePtr{parentNodePtr},
  position{EstimatedNewNodePosition},
  heuristicCostToTarget{Node::bigNumber},
  totalCost{Node::bigNumber} {
    // get actual cost from start to parent node
    const double costFromStartToParent {parentNodePtr->actualCostFromStart};
    const ignition::math::Vector3d vectorFromParentToNewNode {EstimatedNewNodePosition - parentNodePtr->position};
    const double costFromParentToNewNode {vectorFromParentToNewNode.Length()};
    const ignition::math::Vector3d vectorFromNewNodeToTarget {target - EstimatedNewNodePosition};
    const double costFromNewNodeToTarget = Node::getManhattanDistance(EstimatedNewNodePosition, target)*0.9 + vectorFromNewNodeToTarget.Length()*0.1;
    // const double costFromNewNodeToTarget = vectorFromNewNodeToTarget.Length();
    // calculate heuristic cost from new node to target
    this->heuristicCostToTarget = costFromNewNodeToTarget;
    this->totalCost = costFromStartToParent + costFromParentToNewNode + costFromNewNodeToTarget;
    this->actualCostFromStart = this->totalCost - this->heuristicCostToTarget;
}

/// Deconstructor
// ~Node::Node() {
//   this->parentNodePtr = nullptr;
// }


// MARK: - Public Functions

/// get distance from point
double Node::getDistanceFrom(const ignition::math::Vector3d& point) const {
  return std::sqrt( std::pow(this->position.X() - point.X(), 2) + std::pow(this->position.Y() - point.Y(), 2) );
}


bool Node::compareAndUpdateCostIfNeccessary(const Node& anotherParentNode) {
  const ignition::math::Vector3d vectorFromParentToNewNode {this->position - anotherParentNode.position};
  const double costFromParentToNewNode {vectorFromParentToNewNode.Length()};
  const double newTotalCost {anotherParentNode.actualCostFromStart + costFromParentToNewNode + this->heuristicCostToTarget};
  if (newTotalCost < this->totalCost) {
    this->parentNodePtr = &anotherParentNode;
    this->totalCost = newTotalCost;
    this->actualCostFromStart = this->totalCost - this->heuristicCostToTarget;
    return true;
  } else {
    return false;
  }
}


bool Node::operator==(const Node& anotherNode) const {
  return anotherNode.id == this->id;
}

// std::ostream& Node::operator<<(std::ostream &out, const Node& data) {
//     out << "id=" << data.id << " (" <<  data.position.X() << ", " << data.position.Y() << ")";
//     return out;
// }


double Node::getManhattanDistance(const ignition::math::Vector3d& from, const ignition::math::Vector3d& to) {
  return std::abs(to.X() - from.X()) + std::abs(to.Y() - from.Y());
}
