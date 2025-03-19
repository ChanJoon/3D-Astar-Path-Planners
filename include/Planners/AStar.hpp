#ifndef ASTAR_HPP
#define ASTAR_HPP
/**
 * @file AStar.hpp
 * @author Rafael Rey (reyarcenegui@gmail.com)
 * @author Jose Antonio Cobano (jacobsua@upo.es)
 * @brief 
 * @version 0.1
 * @date 2021-06-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
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

        PathData findPath(Eigen::Vector3d &_source, Eigen::Vector3d &_target) override;
        PathData findPath(Eigen::Vector3d _source, Eigen::Vector3d _target, bool dynamic, double time_start) override;
        
    protected:
        void setParam();
        void configAlgorithm();
        virtual void exploreNeighbours(Node* _current, const Eigen::Vector3d &_target,node_by_position &_index_by_pos);
        virtual unsigned int computeG(const Node* _current, Node* _suc, unsigned int _n_i, unsigned int _dirs);
        double getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
        double getDiagonalHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);

        unsigned int line_of_sight_checks_{0};
        std::vector<Node*> closedSet_;
        MagicalMultiSet openSet_;
        
        ros::NodeHandle lnh_{"~"};
        ros::Publisher explored_nodes_marker_pub_, occupancy_marker_pub_,
                       openset_marker_pub_, closedset_marker_pub_,
                       best_node_marker_pub_, aux_text_marker_pub_;
        visualization_msgs::Marker explored_node_marker_, openset_markers_,
                                   closed_set_markers_, best_node_marker_, aux_text_marker_;
        ros::Duration duration_pub_{0.001};
        ros::Time last_publish_tamp_;
        pcl::PointCloud<pcl::PointXYZ>  occupancy_marker_;

    };

}

#endif 
