#include "Planners/ThetaStarAGRpp.hpp"

namespace Planners
{
    ThetaStarAGRpp::ThetaStarAGRpp() : ThetaStar("thetastaragrpp") {}

    ThetaStarAGRpp::ThetaStarAGRpp(std::string _name) : ThetaStar(_name) {}

    void ThetaStarAGRpp::init()
    {
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

    void ThetaStarAGRpp::setParam()
    {
        lnh_.param("thetastaragr/resolution", resolution_, -1.0);
        lnh_.param("thetastaragr/time_resolution", time_resolution_, -1.0);
        lnh_.param("thetastaragr/lambda_heuristic", lambda_heuristic_, -1.0);
        lnh_.param("thetastaragr/allocate_num", allocate_num_, -1);

        lnh_.param("thetastaragr/barrier", barrier_, 0.0);
        lnh_.param("thetastaragr/flying_cost", flying_cost_, 0.0);
        lnh_.param("thetastaragr/flying_cost_default", flying_cost_default_, 0.0);
        lnh_.param("thetastaragr/ground_judge", ground_judge_, 0.0);
        lnh_.param("thetastaragr/epsilon", epsilon_, 0.0);


        ROS_INFO("thetastaragr/resolution: %f", resolution_);
        ROS_INFO("thetastaragr/time_resolution: %f", time_resolution_);
        ROS_INFO("thetastaragr/lambda_heuristic: %f", lambda_heuristic_);
        ROS_INFO("thetastaragr/allocate_num: %d", allocate_num_);
        ROS_INFO("thetastaragr/flying_cost: %f", flying_cost_);
        ROS_INFO("thetastaragr/flying_cost_default: %f", flying_cost_default_);
        ROS_INFO("thetastaragr/ground_judge: %f", ground_judge_);
        ROS_INFO("thetastaragr/epsilon: %f", epsilon_);

        tie_breaker_ = 1.0 + 1.0 / 10000;
    }

    inline void ThetaStarAGRpp::ComputeCost(NodePtr current, NodePtr neighbor, const Eigen::Vector3d &d_pos, const Eigen::Vector3d &target)
    {
        double direct_cost = d_pos.norm();
        bool next_motion_state = (neighbor->position[2] >= ground_judge_);
        double delta_z = neighbor->position[2] - current->position[2];

        // MH
        // direct_cost *= epsilon_;
        // direct_cost += flying_cost_ * delta_z;

        // 이전에 되던거
        double alpha = max(1 + flying_cost_ * delta_z / neighbor->position[2], epsilon_);
        direct_cost *= (next_motion_state || current->motion_state == 1) ? alpha : epsilon_;

        double tmp_g_score = current->g_score + direct_cost;
        if (current->motion_state == 0 && next_motion_state) {
            tmp_g_score += flying_cost_default_;
        }

        double best_cost = tmp_g_score;
        NodePtr best_parent = current;

        if (current->parent != nullptr) {
            line_of_sight_checks_++;
            if (LineOfSight::fastLOS(current->parent, neighbor, grid_map_)) {
                double los_distance = (current->parent->position - neighbor->position).norm();
                double delta_los_z = neighbor->position[2] - current->parent->position[2];
                
                // MH
                // los_distance *= epsilon_;
                // los_distance += flying_cost_ * delta_los_z;

                // 이전에 되던거
                double alpha_los = max(1 + flying_cost_ * delta_los_z / neighbor->position[2], epsilon_);
                los_distance *= (next_motion_state || current->parent->motion_state == 1) ? alpha_los : epsilon_;

                double tmp_g_score_parent = current->parent->g_score + los_distance;

                if (current->parent->motion_state == 0 && next_motion_state) {
                    tmp_g_score_parent += flying_cost_default_;
                }
                
    
                if (tmp_g_score_parent < best_cost)
                {
                    best_cost = tmp_g_score_parent;
                    best_parent = current->parent;
                }
            }
        }

        if (best_cost < neighbor->g_score) {
            neighbor->parent = best_parent;
            neighbor->g_score = best_cost;
            neighbor->f_score = neighbor->g_score + lambda_heuristic_ * getEuclHeu(neighbor->position, target);
            neighbor->motion_state = next_motion_state ? 1 : 0;
        }
    }

    inline void ThetaStarAGRpp::UpdateVertex(NodePtr current, NodePtr neighbor, const Eigen::Vector3d &d_pos, const Eigen::Vector3d &target)
    {
        double old_g_score = neighbor->g_score;

        ComputeCost(current, neighbor, d_pos, target);

        if (neighbor->g_score < old_g_score) {
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

    PathData ThetaStarAGRpp::findPath(Eigen::Vector3d _source,
                                    Eigen::Vector3d _target)
    {
        utils::Clock timer;
        timer.tic();
        line_of_sight_checks_ = 0;

        /* ---------- initialize ---------- */
        NodePtr start_node = path_node_pool_[0];
        start_node->parent = NULL;
        start_node->position = _source;
        start_node->g_score = 0.0;
        start_node->f_score = lambda_heuristic_ * getEuclHeu(start_node->position, _target);
        start_node->node_state = IN_OPEN_SET;

        open_set_.insert(start_node);
        use_node_num_ = 1;

        if (start_node->position[2] < 0)
            start_node->position[2] = 0;

        //TODO(ChanJoon): 초기 비용 제대로 할당하기 (지금은 지상으로 가정)
        if (start_node->position[2] >= ground_judge_) {
            start_node->motion_state = 1; // flying
        } else {
            start_node->motion_state = 0; // rolling
        }

        expanded_nodes_.insert(start_node->position, start_node);

        NodePtr cur_node = NULL;
        NodePtr terminate_node = NULL;

        /* ---------- search loop ---------- */
        while (!open_set_.empty())
        {
            /* ---------- get lowest f_score node ---------- */
            cur_node = *open_set_.begin();

            /* ---------- determine termination ---------- */
            bool reach_end = abs(cur_node->position(0) - _target(0)) <= resolution_ &&
                             abs(cur_node->position(1) - _target(1)) <= resolution_ &&
                             abs(cur_node->position(2) - _target(2)) <= resolution_;

            if (reach_end) {
                terminate_node = cur_node;
                retrievePath(terminate_node);

                return createResultDataObject(terminate_node, timer, use_node_num_, true, _source, line_of_sight_checks_);
            }

            /* ---------- pop node and add to close set ---------- */
            open_set_.erase(open_set_.begin());
            cur_node->node_state = IN_CLOSE_SET;
            iter_num_ += 1;

            /* ---------- init neighbor expansion ---------- */
            Eigen::Vector3d cur_pos = cur_node->position;
            Eigen::Vector3d neighbor_pos, d_pos;

            /* ---------- expansion loop ---------- */
            for (double dx = -resolution_; dx <= resolution_ + 1e-3; dx += resolution_)
                for (double dy = -resolution_; dy <= resolution_ + 1e-3; dy += resolution_)
                    for (double dz = -resolution_; dz <= resolution_ + 1e-3; dz += resolution_) {
                        d_pos << dx, dy, dz;

                        if (d_pos.norm() < 1e-3)
                            continue;

                        neighbor_pos = cur_pos + d_pos;

                        if (!grid_map_->isInMap(neighbor_pos) || grid_map_->getInflateOccupancy(neighbor_pos)) {
                            continue;
                        }

                        /* not in close set */

                        NodePtr neighbor_node = expanded_nodes_.find(neighbor_pos);

                        if (neighbor_node != NULL) {
                            continue;
                        }

                        if (neighbor_node == nullptr) {
                            neighbor_node = path_node_pool_[use_node_num_];
                            neighbor_node->position = neighbor_pos;
                            neighbor_node->g_score = std::numeric_limits<double>::infinity();
                            neighbor_node->motion_state = (neighbor_pos[2] >= ground_judge_) ? 1 : 0;

                            expanded_nodes_.insert(neighbor_pos, neighbor_node);

                            use_node_num_ += 1;

                            if (use_node_num_ == allocate_num_) {
                                cout << "run out of memory." << endl;
                                return createResultDataObject(cur_node, timer, use_node_num_, false, _source, line_of_sight_checks_);
                            }
                        }
                        UpdateVertex(cur_node, neighbor_node, d_pos, _target);
                    }
        }

        /* ---------- open set empty, no path ---------- */
        cout << "open set empty, no path!" << endl;
        cout << "use node num: " << use_node_num_ << endl;
        cout << "iter num: " << iter_num_ << endl;
        return createResultDataObject(cur_node, timer, use_node_num_, false, _source, line_of_sight_checks_);
    }
}