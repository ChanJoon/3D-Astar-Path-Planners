#ifndef ASTAR_HPP
#define ASTAR_HPP

#include <Planners/AlgorithmBase.hpp>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

namespace Planners{

    class AStar : public AlgorithmBase
    {
        
    public:
        AStar();
        AStar(std::string _name);
        ~AStar();

        // PathData findPath(Eigen::Vector3d &_source, Eigen::Vector3d &_target) override;
        PathData findPath(Eigen::Vector3d _source, Eigen::Vector3d _target, bool dynamic, double time_start) override;
        inline void ComputeCost(NodePtr current, NodePtr neighbor, const Eigen::Vector3d& d_pos, const Eigen::Vector3d& target);
        
    protected:
        void setParam();
        void configAlgorithm();
        virtual void exploreNeighbours(Node* _current, const Eigen::Vector3d &_target,node_by_position &_index_by_pos);
        virtual unsigned int computeG(const Node* _current, Node* _suc, unsigned int _n_i, unsigned int _dirs);
        double getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
        double getDiagonalHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
        inline void UpdateVertex(NodePtr current, NodePtr neighbor, const Eigen::Vector3d& d_pos, const Eigen::Vector3d& target);

        unsigned int line_of_sight_checks_{0};
        std::vector<Node*> closedSet_;
        MagicalMultiSet openSet_;
        
        ros::NodeHandle lnh_{"~"};
    };

}

#endif 
