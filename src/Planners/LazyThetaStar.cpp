#include "Planners/LazyThetaStar.hpp"
#include <Eigen/Eigen>

namespace Planners
{
    LazyThetaStar::LazyThetaStar() : ThetaStar("lazythetastar") {}

    LazyThetaStar::LazyThetaStar(std::string _name = "lazythetastar") : ThetaStar(_name) {}

    void LazyThetaStar::init() {
        ThetaStar::init();
    }

    void LazyThetaStar::setParam() {
        lnh_.param("lazythetastar/resolution", resolution_, -1.0);
        lnh_.param("lazythetastar/time_resolution", time_resolution_, -1.0);
        lnh_.param("lazythetastar/lambda_heuristic", lambda_heuristic_, -1.0);
        lnh_.param("lazythetastar/allocate_num", allocate_num_, -1);
        ROS_INFO("lazythetastar/resolution: %f", resolution_);
        ROS_INFO("lazythetastar/time_resolution: %f", time_resolution_);
        ROS_INFO("lazythetastar/lambda_heuristic: %f", lambda_heuristic_);
        ROS_INFO("lazythetastar/allocate_num: %d", allocate_num_);
        tie_breaker_ = 1.0 + 1.0 / 10000;
    }

    inline void LazyThetaStar::ComputeCost(NodePtr current, NodePtr neighbor, const Eigen::Vector3d &d_pos, const Eigen::Vector3d &target)
    {
        // Assume we can connect neighbor to current->parent (if any) for a potentially cheaper path
        if (current->parent != nullptr) {
            double distanceParent = (current->parent->position - neighbor->position).norm();
            double tmp_g_score = current->parent->g_score + distanceParent;
            if (tmp_g_score < neighbor->g_score)
            {
                neighbor->parent = current->parent;
                neighbor->g_score = tmp_g_score;
                neighbor->f_score = neighbor->g_score + lambda_heuristic_ * getEuclHeu(neighbor->position, target);
                return;
            }
        }
        // Fallback to direct connection with current
        double new_g_score = current->g_score + d_pos.norm();
        if (new_g_score < neighbor->g_score) {
            neighbor->parent = current;
            neighbor->g_score = new_g_score;
            neighbor->f_score = neighbor->g_score + lambda_heuristic_ * getEuclHeu(neighbor->position, target);
        }
    }

    void LazyThetaStar::SetVertex(NodePtr current, const Eigen::Vector3d& target)
    {
        if (current->parent == nullptr) return;
        line_of_sight_checks_++;
        if (LineOfSight::fastLOS(current->parent, current, grid_map_)) return;

        // If LOS fails, pick best in-closed neighbor to re-parent
        double best_g = std::numeric_limits<double>::infinity();
        NodePtr best_parent = nullptr;

        Eigen::Vector3d cur_pos = current->position;
        Eigen::Vector3d neighbor_pos;
        Eigen::Vector3d d_pos;

        for (double dx = -resolution_; dx <= resolution_ + 1e-3; dx += resolution_)
            for (double dy = -resolution_; dy <= resolution_ + 1e-3; dy += resolution_)
                for (double dz = -resolution_; dz <= resolution_ + 1e-3; dz += resolution_) {
                    d_pos << dx, dy, dz;
                    if (d_pos.norm() < 1e-3) continue;

                    neighbor_pos = cur_pos + d_pos;
                    if (!grid_map_->isInMap(neighbor_pos)) continue;
                    if (grid_map_->getInflateOccupancy(neighbor_pos)) continue;

                    NodePtr nb = expanded_nodes_.find(neighbor_pos);
                    if (nb == NULL) continue;
                    if (nb->node_state != IN_CLOSE_SET) continue;

                    double g_candidate = nb->g_score + (nb->position - current->position).norm();
                    if (g_candidate < best_g) {
                        best_g = g_candidate;
                        best_parent = nb;
                    }
                }

        if (best_parent != nullptr) {
            current->parent = best_parent;
            current->g_score = best_g;
            current->f_score = current->g_score + lambda_heuristic_ * getEuclHeu(current->position, target);
        }
    }

    inline void LazyThetaStar::UpdateVertex(NodePtr current, NodePtr neighbor, const Eigen::Vector3d &d_pos, const Eigen::Vector3d &target)
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

    inline PathData LazyThetaStar::findPath(Eigen::Vector3d _source, Eigen::Vector3d _target) {
        utils::Clock timer;
        timer.tic();
        line_of_sight_checks_ = 0;

        NodePtr start_node = path_node_pool_[0];
        start_node->parent = NULL;
        start_node->position = _source;
        start_node->g_score = 0.0;
        start_node->f_score = lambda_heuristic_ * getEuclHeu(start_node->position, _target);
        start_node->node_state = IN_OPEN_SET;

        open_set_.insert(start_node);
        use_node_num_ += 1;
        expanded_nodes_.insert(start_node->position, start_node);

        NodePtr cur_node = NULL;
        NodePtr terminate_node = NULL;

        while (!open_set_.empty()) {
            cur_node = *open_set_.begin();
            open_set_.erase(open_set_.begin());

            // Lazy verification on pop
            SetVertex(cur_node, _target);

            bool reach_end = std::abs(cur_node->position(0) - _target(0)) <= resolution_ &&
                             std::abs(cur_node->position(1) - _target(1)) <= resolution_ &&
                             std::abs(cur_node->position(2) - _target(2)) <= resolution_;
            if (reach_end) {
                terminate_node = cur_node;
                retrievePath(terminate_node);
                return createResultDataObject(terminate_node, timer, use_node_num_, true, _source, line_of_sight_checks_);
            }

            cur_node->node_state = IN_CLOSE_SET;
            iter_num_ += 1;

            Eigen::Vector3d cur_pos = cur_node->position;
            Eigen::Vector3d neighbor_pos;
            Eigen::Vector3d d_pos;

            for (double dx = -resolution_; dx <= resolution_ + 1e-3; dx += resolution_)
                for (double dy = -resolution_; dy <= resolution_ + 1e-3; dy += resolution_)
                    for (double dz = -resolution_; dz <= resolution_ + 1e-3; dz += resolution_) {
                        d_pos << dx, dy, dz;
                        if (d_pos.norm() < 1e-3) continue;
                        neighbor_pos = cur_pos + d_pos;

                        if (!grid_map_->isInMap(neighbor_pos)) continue;
                        NodePtr neighbor_node = expanded_nodes_.find(neighbor_pos);
                        if (neighbor_node != NULL && neighbor_node->node_state == IN_CLOSE_SET) continue;
                        if (grid_map_->getInflateOccupancy(neighbor_pos)) continue;

                        if (neighbor_node == NULL) {
                            neighbor_node = path_node_pool_[use_node_num_];
                            neighbor_node->position = neighbor_pos;
                            neighbor_node->g_score = std::numeric_limits<double>::infinity();
                            expanded_nodes_.insert(neighbor_pos, neighbor_node);
                            use_node_num_ += 1;
                            if (use_node_num_ == allocate_num_) {
                                std::cout << "run out of memory." << std::endl;
                                return createResultDataObject(cur_node, timer, use_node_num_, false, _source, line_of_sight_checks_);
                            }
                        }
                        UpdateVertex(cur_node, neighbor_node, d_pos, _target);
                    }
        }

        std::cout << "open set empty, no path!" << std::endl;
        std::cout << "use node num: " << use_node_num_ << std::endl;
        std::cout << "iter num: " << iter_num_ << std::endl;
        return createResultDataObject(cur_node, timer, use_node_num_, false, _source, line_of_sight_checks_);
    }
}
