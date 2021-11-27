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
    AStarPathPlanner(ignition::math::Vector3d start, ignition::math::Vector3d& target, const ignition::math::AxisAlignedBox actorBoundingBox, const physics::WorldPtr world, const std::vector<std::string>& ignoreModels);
    ~AStarPathPlanner() = default;
    AStarPathPlanner& operator=(AStarPathPlanner&&) = default;
    AStarPathPlanner& operator=(AStarPathPlanner&) = default;

    /// update models when there is any change
    void updateModels(const ignition::math::AxisAlignedBox, const physics::WorldPtr, const std::vector<std::string>&);
    /// generate vector to next node
    virtual ignition::math::Vector3d generateGradientNearPosition(const ignition::math::Vector3d&, const ignition::math::Vector3d&);


  private:
    // MARK: - Private Properties

    /// all nodes in map, only in cheat mode.
    vector<Node> allNodesInMap;
    /// all nodes
    vector<Node> nodes;
    /// nextNode
    Node* nextNode;
    /// open list
    vector<Node*> openList;
    /// close list
    vector<Node*> closeList;
    /// obstacles bounding boxes
    vector<ignition::math::AxisAlignedBox> obstacleBoundingBoxes;
    /// actor bounding box
    ignition::math::AxisAlignedBox actorBoundingBox;
    /// target
    ignition::math::Vector3d target;
    ignition::math::Vector3d start;


    // MARK: - Private Functions

    void __addNodesNearToOpenList(const Node& currentNode);
    Node* __getNextNodeToMove();
    bool __isNodeVisibleFrom(const Node& fromNode, const Node& toNode) const;
    static double __getJudgeNumber(const ignition::math::Vector3d& basePoint1, const ignition::math::Vector3d& basePoint2, const ignition::math::Vector3d& testPoint);
    static bool __didIntersect(const ignition::math::Vector3d& group1Point1, const ignition::math::Vector3d& group1Point2, const ignition::math::Vector3d& group2Point1, const ignition::math::Vector3d& group2Point2);
  };
}


#endif
