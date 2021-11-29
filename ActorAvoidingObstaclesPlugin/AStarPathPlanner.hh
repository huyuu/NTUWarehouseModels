#ifndef ALGORITHM_ASTARPATHPLANNER_HH_
#define ALGORITHM_ASTARPATHPLANNER_HH_

#include <iostream>
#include <vector>
#include <functional>
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"
#include <ignition/math.hh>
#include "Node.hh"

using std::vector;

namespace gazebo {
  class GZ_PLUGIN_VISIBLE AStarPathPlanner {
  public:
    /// Constructor
    AStarPathPlanner(): start{ignition::math::Vector3d{}}, nextNode{} {}
    AStarPathPlanner(ignition::math::Vector3d start, ignition::math::Vector3d& target, const ignition::math::AxisAlignedBox actorBoundingBox, const physics::WorldPtr world, const std::vector<std::string>& ignoreModels, const double actorWidth);
    ~AStarPathPlanner() = default;
    AStarPathPlanner& operator=(AStarPathPlanner&&) = default;
    AStarPathPlanner& operator=(AStarPathPlanner&) = default;

    /// update models when there is any change
    void updateModels(const ignition::math::AxisAlignedBox, const physics::WorldPtr, const std::vector<std::string>&);
    /// generate vector to next node
    virtual ignition::math::Vector3d generateGradientNearPosition(const ignition::math::Vector3d&, const ignition::math::Vector3d&);

    // deltaFromCollision
    static constexpr double deltaFromCollision {0.50}; // 50 cm
    static constexpr double distanceAsReached {0.30}; // 15 cm


  private:
    // MARK: - Private Properties

    /// all nodes in map, only in cheat mode.
    vector<Node> allNodesInMap;
    /// all nodes
    vector<Node> nodes;
    /// nextNode
    Node* nextNode;
    /// open list
    vector<int> openList;
    /// close list
    vector<int> closeList;
    /// midway nodes list
    vector<int> midwayNodeIds;
    vector<int> ancestorIds_nextNode;
    /// obstacles bounding boxes
    vector<ignition::math::AxisAlignedBox> obstacleBoundingBoxes;
    /// actor bounding box
    ignition::math::AxisAlignedBox actorBoundingBox;
    /// target
    ignition::math::Vector3d target;
    ignition::math::Vector3d start;
    // actor width
    double actorWidth;


    // MARK: - Private Functions

    void __addNodesNearToOpenList(const Node& currentNode);
    int __getNextNodeIdToMove(const Node& currentNode);
    bool __isNodeVisibleFrom(const Node& fromNode, const Node& toNode) const;
    static double __getJudgeNumber(const ignition::math::Vector3d& basePoint1, const ignition::math::Vector3d& basePoint2, const ignition::math::Vector3d& testPoint);
    static bool __didIntersect(const ignition::math::Vector3d& group1Point1, const ignition::math::Vector3d& group1Point2, const ignition::math::Vector3d& group2Point1, const ignition::math::Vector3d& group2Point2);
    static bool __pointIsReachable(const ignition::math::Vector3d& point, const double actorWidth, const vector<ignition::math::AxisAlignedBox>& boxes);
    static bool __pointIsNotNearOtherNodes(const ignition::math::Vector3d& point, const vector<Node>& otherNodes);
  };
}


#endif
