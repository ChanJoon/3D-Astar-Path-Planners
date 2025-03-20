#include "Planners/AStar.hpp"

namespace Planners {
AStar::AStar(std::string _name = "astar") : AlgorithmBase(_name) {
  setParam();
  configAlgorithm();
}

AStar::AStar() : AlgorithmBase("astar") {
  setParam();
  configAlgorithm();
}

AStar::~AStar() {
  for (int i = 0; i < use_node_num_; i++) {
    delete path_node_pool_[i];
  }
}

void AStar::setParam() {
  lnh_.param("astar/resolution", resolution_, -1.0);
  lnh_.param("astar/time_resolution", time_resolution_, -1.0);
  lnh_.param("astar/lambda_heuristic", lambda_heuristic_, -1.0);
  lnh_.param("astar/allocate_num", allocate_num_, -1);
  ROS_INFO("astar/resolution: %f", resolution_);
  ROS_INFO("astar/time_resolution: %f", time_resolution_);
  ROS_INFO("astar/lambda_heuristic: %f", lambda_heuristic_);
  ROS_INFO("astar/allocate_num: %d", allocate_num_);
  tie_breaker_ = 1.0 + 1.0 / 10000;
}

void AStar::configAlgorithm() {
  closedSet_.reserve(50000);
  openSet_.reserve(50000);
}

inline unsigned int AStar::computeG(const Node *_current,
                                    Node *_suc,
                                    unsigned int _n_i,
                                    unsigned int _dirs) {
  unsigned int cost = _current->G;

  if (_dirs == 8) {
    cost += (_n_i < 4 ? dist_scale_factor_ : dd_2D_);  // This is more efficient
  } else {
    cost +=
        (_n_i < 6 ? dist_scale_factor_ : (_n_i < 18 ? dd_2D_ : dd_3D_));  // This is more efficient
  }

  _suc->C = _suc->cost;

  return cost;
}

#pragma GCC diagnostic pop

void AStar::exploreNeighbours(Node *_current,
                              const Eigen::Vector3d &_target,
                              node_by_position &_index_by_pos) {
  for (unsigned int i = 0; i < direction.size(); ++i) {
    Eigen::Vector3d newCoordinates = _current->coordinates + direction[i];
    Node *successor = discrete_world_.getNodePtr(newCoordinates);
    // Skip the neighbour if it is not valid, occupied, or already in teh
    // closed list
    if (successor == nullptr || successor->isInClosedList || successor->occupied) continue;

    unsigned int totalCost = computeG(_current, successor, i, direction.size());

    if (!successor->isInOpenList) {
      successor->parent = _current;
      successor->G = totalCost;
      successor->H = heuristic(successor->coordinates, _target);
      successor->gplush = successor->G + successor->H;
      successor->isInOpenList = true;
      _index_by_pos.insert(successor);
    } else if (totalCost < successor->G) {
      successor->parent = _current;
      successor->G = totalCost;
      successor->gplush = successor->G + successor->H;
      auto found = _index_by_pos.find(successor->world_index);
      _index_by_pos.erase(found);
      _index_by_pos.insert(successor);
    }
  }
}
// PathData AStar::findPath(Eigen::Vector3d &_source, Eigen::Vector3d &_target) {
//   Node *current = nullptr;

//   bool solved{false};

//   discrete_world_.getNodePtr(_source)->parent = new Node(_source);
//   discrete_world_.setOpenValue(_source, true);
//   // Timer to record the execution time, not
//   // really important
//   utils::Clock main_timer;
//   main_timer.tic();

//   line_of_sight_checks_ = 0;

//   node_by_cost &indexByCost = openSet_.get<IndexByCost>();
//   node_by_position &indexByWorldPosition = openSet_.get<IndexByWorldPosition>();

//   indexByCost.insert(discrete_world_.getNodePtr(_source));

//   while (!indexByCost.empty()) {
//     // Get the element at the start of the open set ordered by cost
//     auto it = indexByCost.begin();
//     current = *it;
//     indexByCost.erase(indexByCost.begin());

//     if (current->coordinates == _target) {
//       solved = true;
//       break;
//     }

//     closedSet_.push_back(current);
//     // This flags are used to avoid search in the containers,
//     // for speed reasons.
//     current->isInOpenList = false;
//     current->isInClosedList = true;

//     exploreNeighbours(current, _target, indexByWorldPosition);
//   }
//   main_timer.toc();

//   PathData result_data = createResultDataObject(
//       current, main_timer, closedSet_.size(), solved, _source, line_of_sight_checks_);
//   closedSet_.clear();
//   openSet_.clear();
//   delete discrete_world_.getNodePtr(_source)->parent;

//   discrete_world_.resetWorld();
//   return result_data;
// }

double AStar::getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2) {
  return tie_breaker_ * (x2 - x1).norm();
}

double AStar::getDiagonalHeu(Eigen::Vector3d x1, Eigen::Vector3d x2) {
  double dx = std::abs(x1(0) - x2(0));
  double dy = std::abs(x1(1) - x2(1));
  double dz = std::abs(x1(2) - x2(2));
  double min_xyz = std::min({dx, dy, dz});
  double h = dx + dy + dz + (std::sqrt(3) - 3) * min_xyz;
  return tie_breaker_ * h;
}

inline void AStar::ComputeCost(NodePtr current, NodePtr neighbor, const Eigen::Vector3d& d_pos, const Eigen::Vector3d& target) {
  double direct_cost = d_pos.norm();
  double new_g_score = current->g_score + direct_cost;
  
  neighbor->parent = current;
  neighbor->g_score = new_g_score;
  neighbor->f_score = new_g_score + lambda_heuristic_ * getEuclHeu(neighbor->position, target);
}

inline void AStar::UpdateVertex(NodePtr current, NodePtr neighbor, const Eigen::Vector3d& d_pos, const Eigen::Vector3d& target) {
  double old_g_score = neighbor->g_score;
  
  ComputeCost(current, neighbor, d_pos, target);
  
  // If we found a better path
  if (neighbor->g_score < old_g_score) {
    // If node is in open set but not at the top, we'd need to update its position
    // Most priority queue implementations don't support this directly
    // A common approach is to add the node again with updated cost
    if (neighbor->node_state == IN_OPEN_SET) {
      // Note: This is a simplified approach. A real implementation might use a
      // priority queue that supports decreasing key operations
      open_set_.push(neighbor); 
    } else {
      neighbor->node_state = IN_OPEN_SET;
      open_set_.push(neighbor);
    }
  }
}

PathData AStar::findPath(Eigen::Vector3d _source,
                         Eigen::Vector3d _target,
                         bool dynamic,
                         double time_start) {
  utils::Clock timer;
  timer.tic();

  /* ---------- initialize ---------- */
  NodePtr start_node = path_node_pool_[0];
  start_node->parent = NULL;
  start_node->position = _source;
  start_node->g_score = 0.0;
  start_node->f_score = lambda_heuristic_ * getEuclHeu(start_node->position, _target);
  start_node->node_state = IN_OPEN_SET;

  open_set_.push(start_node);
  use_node_num_ += 1;

  expanded_nodes_.insert(start_node->position, start_node);

  NodePtr cur_node = NULL;
  NodePtr terminate_node = NULL;

  /* ---------- search loop ---------- */
  while (!open_set_.empty()) {
    /* ---------- get lowest f_score node and pop node ---------- */
    cur_node = open_set_.top();
    open_set_.pop();

    /* ---------- determine termination ---------- */

    bool reach_end = abs(cur_node->position(0) - _target(0)) <= resolution_ &&
                     abs(cur_node->position(1) - _target(1)) <= resolution_ &&
                     abs(cur_node->position(2) - _target(2)) <= resolution_;

    if (reach_end) {
      terminate_node = cur_node;
      retrievePath(terminate_node);

      return createResultDataObject(terminate_node, timer, use_node_num_, true, _source, 0);
    }

    /* ---------- add to close set ---------- */
    cur_node->node_state = IN_CLOSE_SET;
    iter_num_ += 1;
    close_list_.insert(cur_node->position, cur_node);

    /* ---------- init neighbor expansion ---------- */
    Eigen::Vector3d cur_pos = cur_node->position;
    Eigen::Vector3d neighbor_pos;
    Eigen::Vector3d d_pos;

    /* ---------- expansion loop ---------- */
    for (double dx = -resolution_; dx <= resolution_ + 1e-3; dx += resolution_)
      for (double dy = -resolution_; dy <= resolution_ + 1e-3; dy += resolution_)
        for (double dz = -resolution_; dz <= resolution_ + 1e-3; dz += resolution_) {
          d_pos << dx, dy, dz;

          if (d_pos.norm() < 1e-3) continue;

          neighbor_pos = cur_pos + d_pos;
          /* ---------- check if in feasible space ---------- */
          /* inside map range */
          // if (neighbor_pos(0) <= origin_(0) || neighbor_pos(0) >= map_size_3d_(0) ||
          //     neighbor_pos(1) <= origin_(1) || neighbor_pos(1) >= map_size_3d_(1) ||
          //     neighbor_pos(2) <= origin_(2) || neighbor_pos(2) >= map_size_3d_(2)) {
          //   continue;
          // }
          // if (edt_environment_->sdf_map_->isInMap(neighbor_pos) == false) {
          //   continue;
          // }
          if (grid_map_->isInMap(neighbor_pos) == false) {
            continue;
          }

          /* not in close set */
          if (close_list_.find(neighbor_pos) != NULL) {
            continue;
          }
          // if (neighbor_node != NULL && neighbor_node->node_state == IN_CLOSE_SET) {
          //   continue;
          // }

          // if (edt_environment_->sdf_map_->getInflateOccupancy(neighbor_pos) == true) {
          //   continue;
          // }
          // if (edt_environment_->evaluateCoarseEDT(neighbor_pos, -1.0) <= 0.3) {
          //   continue;
          // }
          if (grid_map_->getInflateOccupancy(neighbor_pos) == true) {
            continue;
          }

          NodePtr neighbor_node = expanded_nodes_.find(neighbor_pos);

          if (neighbor_node == NULL) {
            neighbor_node = path_node_pool_[use_node_num_];
            neighbor_node->position = neighbor_pos;
            neighbor_node->g_score = std::numeric_limits<double>::infinity();

            expanded_nodes_.insert(neighbor_pos, neighbor_node);

            use_node_num_ += 1;

            if (use_node_num_ == allocate_num_) {
              std::cout << "run out of memory." << std::endl;
              return createResultDataObject(cur_node, timer, use_node_num_, false, _source, 0);
            }
          }
          if (neighbor_node->node_state == IN_CLOSE_SET) {
            continue;
          }
          UpdateVertex(cur_node, neighbor_node, d_pos, _target);
        }
  }

  /* ---------- open set empty, no path ---------- */
  std::cout << "open set empty, no path!" << std::endl;
  std::cout << "use node num: " << use_node_num_ << std::endl;
  std::cout << "iter num: " << iter_num_ << std::endl;
  return createResultDataObject(cur_node, timer, use_node_num_, false, _source, 0);
}
}  // namespace Planners
