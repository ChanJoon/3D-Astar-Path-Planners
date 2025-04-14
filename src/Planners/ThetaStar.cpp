#include "Planners/ThetaStar.hpp"

namespace Planners
{
    ThetaStar::ThetaStar() : AStar("thetastar") {}

    ThetaStar::ThetaStar(std::string _name = "thetastar") : AStar(_name) {}

    void ThetaStar::init() {
        this->inv_resolution_ = 1.0 / resolution_;
        inv_time_resolution_ = 1.0 / time_resolution_;
      
        grid_map_->getRegion(origin_, map_size_3d_);
        ROS_INFO("origin: %f, %f, %f", origin_(0), origin_(1), origin_(2));
        ROS_INFO("map_size_3d: %f, %f, %f", map_size_3d_(0), map_size_3d_(1), map_size_3d_(2));
      
        path_node_pool_.resize(allocate_num_);
        for (int i = 0; i < allocate_num_; i++) {
          path_node_pool_[i] = new Node0();
        }
        use_node_num_ = 0;
        iter_num_ = 0;
    }

    void ThetaStar::setParam() {
        lnh_.param("thetastar/resolution", resolution_, -1.0);
        lnh_.param("thetastar/time_resolution", time_resolution_, -1.0);
        lnh_.param("thetastar/lambda_heuristic", lambda_heuristic_, -1.0);
        lnh_.param("thetastar/allocate_num", allocate_num_, -1);
        ROS_INFO("thetastar/resolution: %f", resolution_);
        ROS_INFO("thetastar/time_resolution: %f", time_resolution_);
        ROS_INFO("thetastar/lambda_heuristic: %f", lambda_heuristic_);
        ROS_INFO("thetastar/allocate_num: %d", allocate_num_);
        tie_breaker_ = 1.0 + 1.0 / 10000;
    }

    inline void ThetaStar::ComputeCost(NodePtr current, NodePtr neighbor, const Eigen::Vector3d &d_pos, const Eigen::Vector3d &target)
    {
        if (current->parent == nullptr) { // If null, use original A* cost
            double direct_cost = d_pos.norm();
            if ((current->g_score + direct_cost) < neighbor->g_score) {
                neighbor->parent = current;
                neighbor->g_score = current->g_score + direct_cost;
                neighbor->f_score = neighbor->g_score + lambda_heuristic_ * getEuclHeu(neighbor->position, target);
            }
            return;
        }

        double distanceParent = (current->parent->position - neighbor->position).norm();
        line_of_sight_checks_++;
        if (LineOfSight::fastLOS(current->parent, neighbor, grid_map_))
        {
            double tmp_g_score = current->parent->g_score + distanceParent;
            if (tmp_g_score < neighbor->g_score)
            {
                neighbor->parent = current->parent;
                neighbor->g_score = tmp_g_score;
                neighbor->f_score = neighbor->g_score + lambda_heuristic_ * getEuclHeu(neighbor->position, target);
            }
        }
        else
        {
            if ((current->g_score + d_pos.norm()) < neighbor->g_score)
            {
                neighbor->parent = current;
                neighbor->g_score = current->g_score + d_pos.norm();
                neighbor->f_score = neighbor->g_score + lambda_heuristic_ * getEuclHeu(neighbor->position, target);
            }
        }
    }

    inline void ThetaStar::UpdateVertex(NodePtr current, NodePtr neighbor, const Eigen::Vector3d &d_pos, const Eigen::Vector3d &target)
    {
        double old_g_score = neighbor->g_score;

        ComputeCost(current, neighbor, d_pos, target);

        if (neighbor->g_score < old_g_score)
        {
            if (neighbor->node_state == IN_OPEN_SET) {
                open_set_.erase(neighbor);
                open_set_.insert(neighbor);
            }
            else{
                neighbor->node_state = IN_OPEN_SET;
                open_set_.insert(neighbor);
            }
        }
    }
}
