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

        void setParam() override;
        void init() override;
        virtual inline PathData findPath(Eigen::Vector3d _source, Eigen::Vector3d _target);
        
        protected:
        void configAlgorithm();
        double getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
        double getDiagonalHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
        virtual inline void ComputeCost(NodePtr current, NodePtr neighbor, const Eigen::Vector3d& d_pos, const Eigen::Vector3d& target);
        virtual inline void UpdateVertex(NodePtr current, NodePtr neighbor, const Eigen::Vector3d& d_pos, const Eigen::Vector3d& target);

        unsigned int line_of_sight_checks_{0};
        
        ros::NodeHandle lnh_{"~"};
    };

}

#endif 
