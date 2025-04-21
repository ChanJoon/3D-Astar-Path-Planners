#include "Planners/ThetaStarAGR.hpp"

namespace Planners
{
    ThetaStarAGR::ThetaStarAGR(const rclcpp::Node::SharedPtr& node) : ThetaStar("thetastaragr"), node_(node) {}

    ThetaStarAGR::ThetaStarAGR(std::string _name) : ThetaStar(_name) {}

    void ThetaStarAGR::init()
    {
        this->inv_resolution_ = 1.0 / resolution_;
        inv_time_resolution_ = 1.0 / time_resolution_;

        grid_map_->getRegion(origin_, map_size_3d_);
        RCLCPP_INFO(node_->get_logger(), "origin: %f, %f, %f", origin_(0), origin_(1), origin_(2));
        RCLCPP_INFO(node_->get_logger(), "map_size_3d: %f, %f, %f", map_size_3d_(0), map_size_3d_(1), map_size_3d_(2));

        path_node_pool_.resize(allocate_num_);
        for (int i = 0; i < allocate_num_; i++) {
            path_node_pool_[i] = new Node0();
        }
        use_node_num_ = 0;
        iter_num_ = 0;
    }

    void ThetaStarAGR::setParam()
    {
        node_->declare_parameter("thetastaragr.resolution", -1.0);
        node_->declare_parameter("thetastaragr.time_resolution", -1.0);
        node_->declare_parameter("thetastaragr.lambda_heuristic", -1.0);
        node_->declare_parameter("thetastaragr.allocate_num", -1);

        node_->declare_parameter("thetastaragr.barrier", 0.0);
        node_->declare_parameter("thetastaragr.flying_cost", 0.0);
        node_->declare_parameter("thetastaragr.flying_cost_default", 0.0);
        node_->declare_parameter("thetastaragr.ground_judge", 0.0);

        node_->get_parameter("thetastaragr.resolution", resolution_);
        node_->get_parameter("thetastaragr.time_resolution", time_resolution_);
        node_->get_parameter("thetastaragr.lambda_heuristic", lambda_heuristic_);
        node_->get_parameter("thetastaragr.allocate_num", allocate_num_);
        node_->get_parameter("thetastaragr.barrier", barrier_);
        node_->get_parameter("thetastaragr.flying_cost", flying_cost_);
        node_->get_parameter("thetastaragr.flying_cost_default", flying_cost_default_);
        node_->get_parameter("thetastaragr.ground_judge", ground_judge_);

        RCLCPP_INFO(node_->get_logger(), "thetastaragr.resolution: %f", resolution_);
        RCLCPP_INFO(node_->get_logger(), "thetastaragr.time_resolution: %f", time_resolution_);
        RCLCPP_INFO(node_->get_logger(), "thetastaragr.lambda_heuristic: %f", lambda_heuristic_);
        RCLCPP_INFO(node_->get_logger(), "thetastaragr.allocate_num: %d", allocate_num_);
        RCLCPP_INFO(node_->get_logger(), "thetastaragr.flying_cost: %f", flying_cost_);
        RCLCPP_INFO(node_->get_logger(), "thetastaragr.flying_cost_default: %f", flying_cost_default_);
        RCLCPP_INFO(node_->get_logger(), "thetastaragr.ground_judge: %f", ground_judge_);

        tie_breaker_ = 1.0 + 1.0 / 10000;
    }

    inline void ThetaStarAGR::ComputeCost(NodePtr current, NodePtr neighbor, const Eigen::Vector3d &d_pos, const Eigen::Vector3d &target)
    {
        double direct_cost = d_pos.norm();
        double tmp_g_score = current->g_score + direct_cost;
        double penalty_g_score  = 0.0;
        bool next_motion_state = (neighbor->position[2] >= ground_judge_);

        if (next_motion_state) {
            penalty_g_score = flying_cost_ * neighbor->position[2] / barrier_ + flying_cost_default_;
            tmp_g_score -= current->penalty_g_score;
            tmp_g_score += penalty_g_score;
        } else {
            tmp_g_score -= current->penalty_g_score;
        }

        if (current->parent == nullptr) { // If null, use original A* cost
            if (tmp_g_score < neighbor->g_score) {
                neighbor->parent = current;
                neighbor->g_score = tmp_g_score;
                neighbor->f_score = neighbor->g_score + lambda_heuristic_ * getEuclHeu(neighbor->position, target);
                neighbor->motion_state = next_motion_state;
                neighbor->penalty_g_score = penalty_g_score;
            }
            return;
        }

        bool is_parent_ground = (current->parent != nullptr) && !current->parent->motion_state;
        bool is_neighbor_ground = !next_motion_state;
        line_of_sight_checks_++;
        if (LineOfSight::fastLOS(current->parent, neighbor, grid_map_)) {
        // if (is_parent_ground && is_neighbor_ground && LineOfSight::fastLOS(current->parent, neighbor, grid_map_)) {
            double los_distance = (current->parent->position - neighbor->position).norm();
            tmp_g_score = current->parent->g_score + los_distance;
            if (next_motion_state) {
                tmp_g_score -= current->parent->penalty_g_score;
                tmp_g_score += penalty_g_score;
            } else {
                tmp_g_score -= current->parent->penalty_g_score;
            }
            if (tmp_g_score < neighbor->g_score)
            {
                neighbor->parent = current->parent;
                neighbor->g_score = tmp_g_score;
                neighbor->f_score = neighbor->g_score + lambda_heuristic_ * getEuclHeu(neighbor->position, target);
                neighbor->motion_state = next_motion_state;
                neighbor->penalty_g_score = penalty_g_score;
            }
        } else if (tmp_g_score < neighbor->g_score) {
                neighbor->parent = current;
                neighbor->g_score = tmp_g_score;
                neighbor->f_score = neighbor->g_score + lambda_heuristic_ * getEuclHeu(neighbor->position, target);
                neighbor->motion_state = next_motion_state;
                neighbor->penalty_g_score = penalty_g_score;
        }
    }

    inline void ThetaStarAGR::UpdateVertex(NodePtr current, NodePtr neighbor, const Eigen::Vector3d &d_pos, const Eigen::Vector3d &target)
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

    PathData ThetaStarAGR::findPath(Eigen::Vector3d _source,
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

        if (start_node->position[2] >= ground_judge_)
        {
            start_node->g_score += flying_cost_ * start_node->position[2] / barrier_ + flying_cost_default_;
            start_node->f_score += flying_cost_ * start_node->position[2] / barrier_ + flying_cost_default_;
            start_node->penalty_g_score = flying_cost_ * start_node->position[2] / barrier_ + flying_cost_default_;
            start_node->motion_state = 1; // flying
        }
        else
        {
            start_node->motion_state = 0; // rolling
            start_node->penalty_g_score = 0;
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

                        /* ---------- check if in feasible space ---------- */
                        /* inside map range */
                        // if (neighbor_pos(0) <= origin_(0) || neighbor_pos(0) >= map_size_3d_(0) || neighbor_pos(1) <= origin_(1) ||
                        //     neighbor_pos(1) >= map_size_3d_(1) || neighbor_pos(2) <= origin_(2) ||
                        //     neighbor_pos(2) >= map_size_3d_(2)) {
                        //   // cout << "outside map" << endl;
                        //   continue;
                        // }
                        if (!grid_map_->isInMap(neighbor_pos) || grid_map_->getInflateOccupancy(neighbor_pos)) {
                            continue;
                        }

                        /* not in close set */

                        NodePtr neighbor_node = expanded_nodes_.find(neighbor_pos);

                        if (neighbor_node != NULL && neighbor_node->node_state == IN_CLOSE_SET) {
                            continue;
                        }

                        /* collision free */
                        // double dist = dynamic ?
                        // edt_environment_->evaluateCoarseEDT(neighbor_pos, cur_node->time + dt) :
                        //                         edt_environment_->evaluateCoarseEDT(neighbor_pos,
                        //                         -1.0);

                        // if (edt_environment_->sdf_map_->getInflateOccupancy(neighbor_pos) != 0)
                        // {
                        //     continue;
                        // }
                        // if (grid_map_->getInflateOccupancy(neighbor_pos) == true)
                        // {
                        //     continue;
                        // }

                        /* ---------- compute cost ---------- */
                        // double tmp_g_score, tmp_f_score, penalty_g_score;
                        // bool next_motion_state = false;
                        // tmp_g_score = d_pos.squaredNorm() + cur_node->g_score;
                        // if (neighbor_pos[2] > ground_judge_) {
                        //     tmp_g_score -= cur_node->penalty_g_score;
                        //     tmp_g_score += flying_cost_ * neighbor_pos[2] / 2.0 + flying_cost_default_;
                        //     penalty_g_score = flying_cost_ * neighbor_pos[2] / 2.0 + flying_cost_default_;
                        //     next_motion_state = true;
                        // } else {
                        //     tmp_g_score -= cur_node->penalty_g_score;
                        //     penalty_g_score = 0;
                        //     next_motion_state = false;
                        // }

                        // tmp_f_score = tmp_g_score + lambda_heuristic_ * getEuclHeu(neighbor_pos, _target);

                        // if (neighbor_node == NULL)
                        // {
                        //     neighbor_node = path_node_pool_[use_node_num_];
                        //     neighbor_node->position = neighbor_pos;
                        //     neighbor_node->f_score = tmp_f_score;
                        //     neighbor_node->g_score = tmp_g_score;
                        //     neighbor_node->parent = cur_node;
                        //     neighbor_node->node_state = IN_OPEN_SET;
                        //     neighbor_node->motion_state = next_motion_state;
                        //     neighbor_node->penalty_g_score = penalty_g_score;

                        //     open_set_.insert(neighbor_node);
                        //     expanded_nodes_.insert(neighbor_pos, neighbor_node);

                        //     use_node_num_ += 1;
                        //     if (use_node_num_ == allocate_num_)
                        //     {
                        //         cout << "run out of memory." << endl;
                        //         return createResultDataObject(cur_node, timer, use_node_num_, false, _source, line_of_sight_checks_);
                        //     }
                        // }
                        // else if (neighbor_node->node_state == IN_OPEN_SET)
                        // {
                        //     if (tmp_g_score < neighbor_node->g_score)
                        //     {
                        //         neighbor_node->position = neighbor_pos;
                        //         neighbor_node->f_score = tmp_f_score;
                        //         neighbor_node->g_score = tmp_g_score;
                        //         neighbor_node->parent = cur_node;
                        //         neighbor_node->motion_state = next_motion_state;
                        //         neighbor_node->penalty_g_score = penalty_g_score;
                        //     }
                        // }
                        // else
                        // {
                        //     cout << "error type in searching: " << neighbor_node->node_state << endl;
                        // }

                        if (neighbor_node == nullptr) {
                            neighbor_node = path_node_pool_[use_node_num_];
                            neighbor_node->position = neighbor_pos;
                            neighbor_node->g_score = std::numeric_limits<double>::infinity();

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