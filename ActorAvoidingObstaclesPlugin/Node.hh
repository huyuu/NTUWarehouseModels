#ifndef DATATYPE_NODE_HH_
#define DATATYPE_NODE_HH_

#include <iostream>
#include <ignition/math.hh>

namespace gazebo {

  class Node {
  public:
    // MARK: - Constructor
    /// Default all terms constructor
    Node(bool isOpen, int id, Node* parentNodePtr, ignition::math::Vector3d position, double heuristicCostToTarget, double totalCost): isOpen{isOpen}, id{id}, parentNodePtr{parentNodePtr}, position{position}, heuristicCostToTarget{heuristicCostToTarget}, totalCost{totalCost} {
      this->actualCostFromStart = this->totalCost - this->heuristicCostToTarget;
    }
    /// Constructor for new start point
    Node(int id, ignition::math::Vector3d position, ignition::math::Vector3d& target):
      isOpen{true},
      id{id},
      parentNodePtr{nullptr},
      position{position},
      heuristicCostToTarget{Node::bigNumber},
      totalCost{Node::bigNumber} {
        const double vectorFromStartToTarget {target - position};
        this->heuristicCostToTarget = vectorFromStartToTarget.Length();
        this->totalCost = this->heuristicCostToTarget;
        this->actualCostFromStart = this->totalCost - this->heuristicCostToTarget;
    }
    /// Constructor for new normal points
    Node(int id, Node* parentNodePtr, ignition::math::Vector3d EstimatedNewNodePosition, ignition::math::Vector3d& target):
      isOpen{true},
      id{id},
      parentNodePtr{parentNodePtr},
      position{EstimatedNewNodePosition},
      heuristicCostToTarget{Node::bigNumber},
      totalCost{Node::bigNumber} {
        // get actual cost from start to parent node
        const double costFromStartToParent {parentNode.getActualCostFromStartToCurrentPosition()};
        const double vectorFromParentToNewNode {EstimatedNewNodePosition - parentNodePtr->position};
        const double costFromParentToNewNode {vectorFromParentToNewNode.Length()};
        const double vectorFromNewNodeToTarget {target - EstimatedNewNodePosition};
        const double costFromNewNodeToTarget {vectorFromNewNodeToTarget.Length()};
        // calculate heuristic cost from new node to target
        this->heuristicCostToTarget = costFromNewNodeToTarget;
        this->totalCost = costFromStartToParent + costFromParentToNewNode + costFromNewNodeToTarget;
        this->actualCostFromStart = this->totalCost - this->heuristicCostToTarget;
    }
    /// Deconstructor
    ~Node() {
      this->parentNodePtr = nullptr;
    }


    // MARK: - Public Properties

    /// is in the open list or not
    bool isOpen ;
    /// id
    int id;
    /// parent node ptr
    Node* parentNodePtr;
    /// position in coordinate
    ignition::math::Vector3d position;
    /// heuristic cost from current position to target
    double heuristicCostToTarget;
    /// total cost from start to target
    double totalCost;
    /// actual cost
    double actualCostFromStart;

    static const int bigNumber = 1000000;


    // MARK: - Public Functions

    /// get distance from point
    get getDistanceFrom(ignition::math::Vector3d& point) {
      return std::sqrt( std::pow(this->position.x - point.X(), 2) + std::pow(this->position.y - point.Y(), 2) );
    }


    bool compareAndUpdateCostIfNeccessary(Node& anothorParentNode) {
      const double vectorFromParentToNewNode {this->position - anothorParentNode.position};
      const double costFromParentToNewNode {vectorFromParentToNewNode.Length()};
      newTotalCost = anotherParentNode.actualCostFromStart + costFromParentToNewNode + this->heuristicCostToTarget;
      if (newTotalCost < this->actualCostFromStart) {
        this->parentNodePtr = &anotherParentNode;
        this->totalCost = newTotalCost;
        this->actualCostFromStart = this->totalCost - this->heuristicCostToTarget;
        return true;
      } else {
        return false;
      }
    }
    

    bool operator==(const Node& anotherNode) const {
      return anotherNode.id == this->id;
    }
  }
}

#endif
