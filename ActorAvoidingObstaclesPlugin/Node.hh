#ifndef DATATYPE_NODE_HH_
#define DATATYPE_NODE_HH_

#include <iostream>
#include <ignition/math.hh>

namespace gazebo {

  class Node {
  public:
    // MARK: - Constructor
    Node();
    /// Default all terms constructor
    Node(bool isOpen, int id, const Node* parentNodePtr, ignition::math::Vector3d position, double heuristicCostToTarget, double totalCost);
    /// Constructor for new start point
    Node(int id, ignition::math::Vector3d position, ignition::math::Vector3d& target);
    /// Constructor for new normal points
    Node(int id, const Node* const parentNodePtr, ignition::math::Vector3d EstimatedNewNodePosition, ignition::math::Vector3d& target);
    /// Deconstructor
    ~Node();


    // MARK: - Public Properties

    /// is in the open list or not
    bool isOpen ;
    /// id
    int id;
    /// parent node ptr
    const Node* parentNodePtr;
    /// position in coordinate
    const ignition::math::Vector3d position;
    /// heuristic cost from current position to target
    double heuristicCostToTarget;
    /// total cost from start to target
    double totalCost;
    /// actual cost
    double actualCostFromStart;

    static const int bigNumber = 1000000;


    // MARK: - Public Functions

    /// get distance from point
    double getDistanceFrom(const ignition::math::Vector3d& point) const;
    bool compareAndUpdateCostIfNeccessary(const Node& anotherParentNode);

    bool operator==(const Node& anotherNode) const;
    friend std::ostream& operator<<(std::ostream &out, const Node& data);

    static double getManhattanDistance(const ignition::math::Vector3d& from, const ignition::math::Vector3d& to);
  };
}

#endif
