#ifndef ASTAR_HPP
#define ASTAR_HPP

#include <Planners/AlgorithmBase.hpp>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace Planners{

    class AStar : public AlgorithmBase
    {
        
    public:
        AStar(const rclcpp::Node::SharedPtr& node);
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
        rclcpp::Node::SharedPtr node_;
    };

}

#endif 
