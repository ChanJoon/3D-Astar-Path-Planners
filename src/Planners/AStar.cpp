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
  // If compiled with ros and visualization
  explored_nodes_marker_pub_ = lnh_.advertise<visualization_msgs::Marker>("explored_nodes", 1);
  openset_marker_pub_ = lnh_.advertise<visualization_msgs::Marker>("openset_nodes", 1);
  closedset_marker_pub_ = lnh_.advertise<visualization_msgs::Marker>("closed_set_nodes", 1);
  best_node_marker_pub_ = lnh_.advertise<visualization_msgs::Marker>("best_node_marker", 1);
  aux_text_marker_pub_ = lnh_.advertise<visualization_msgs::Marker>("aux_text_marker", 1);
  occupancy_marker_pub_ =
      lnh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("occupancy_markers", 1, true);

  std::string frame_id;
  lnh_.param("frame_id", frame_id, std::string("map"));
  occupancy_marker_.header.frame_id = frame_id;  // "world";

  explored_node_marker_.header.frame_id = frame_id;  //"world";
  explored_node_marker_.header.stamp = ros::Time();
  explored_node_marker_.ns = "debug";
  explored_node_marker_.id = 66;
  explored_node_marker_.type = visualization_msgs::Marker::CUBE_LIST;
  explored_node_marker_.action = visualization_msgs::Marker::ADD;
  explored_node_marker_.pose.orientation.w = 1.0;
  explored_node_marker_.scale.x = 1.0 * resolution_;
  explored_node_marker_.scale.y = 1.0 * resolution_;
  explored_node_marker_.scale.z = 1.0 * resolution_;
  explored_node_marker_.color.a = 0.7;
  explored_node_marker_.color.r = 0.0;
  explored_node_marker_.color.g = 1.0;
  explored_node_marker_.color.b = 0.0;

  openset_markers_ = explored_node_marker_;
  openset_markers_.color.b = 1.0;
  openset_markers_.color.g = 0.0;
  openset_markers_.id = 67;

  closed_set_markers_ = explored_node_marker_;
  closed_set_markers_.color.g = 0.0;
  closed_set_markers_.color.r = 1.0;
  explored_node_marker_.id = 68;

  best_node_marker_ = explored_node_marker_;
  best_node_marker_.color.g = 0.7;
  best_node_marker_.color.b = 0.7;
  best_node_marker_.id = 69;
  best_node_marker_.type = visualization_msgs::Marker::SPHERE;

  aux_text_marker_ = explored_node_marker_;
  aux_text_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  aux_text_marker_.id = 70;
  aux_text_marker_.color.a = 0.7;
  aux_text_marker_.color.g = 0.0;
  aux_text_marker_.text = "";
  aux_text_marker_.scale.z = 3.0 * resolution_;
  last_publish_tamp_ = ros::Time::now();
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
PathData AStar::findPath(Eigen::Vector3d &_source, Eigen::Vector3d &_target) {
  Node *current = nullptr;

  bool solved{false};

  discrete_world_.getNodePtr(_source)->parent = new Node(_source);
  discrete_world_.setOpenValue(_source, true);
  // Timer to record the execution time, not
  // really important
  utils::Clock main_timer;
  main_timer.tic();

  line_of_sight_checks_ = 0;

  node_by_cost &indexByCost = openSet_.get<IndexByCost>();
  node_by_position &indexByWorldPosition = openSet_.get<IndexByWorldPosition>();

  indexByCost.insert(discrete_world_.getNodePtr(_source));

  while (!indexByCost.empty()) {
    // Get the element at the start of the open set ordered by cost
    auto it = indexByCost.begin();
    current = *it;
    indexByCost.erase(indexByCost.begin());

    if (current->coordinates == _target) {
      solved = true;
      break;
    }

    closedSet_.push_back(current);
    // This flags are used to avoid search in the containers,
    // for speed reasons.
    current->isInOpenList = false;
    current->isInClosedList = true;

    exploreNeighbours(current, _target, indexByWorldPosition);
  }
  main_timer.toc();

  PathData result_data = createResultDataObject(
      current, main_timer, closedSet_.size(), solved, _source, line_of_sight_checks_);
  closedSet_.clear();
  openSet_.clear();
  delete discrete_world_.getNodePtr(_source)->parent;

  discrete_world_.resetWorld();
  return result_data;
}

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

  ROS_INFO("Start A* search");
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

      ROS_INFO("A* search done");
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
          if (neighbor_pos(0) <= origin_(0) || neighbor_pos(0) >= map_size_3d_(0) ||
              neighbor_pos(1) <= origin_(1) || neighbor_pos(1) >= map_size_3d_(1) ||
              neighbor_pos(2) <= origin_(2) || neighbor_pos(2) >= map_size_3d_(2)) {
            continue;
          }

          /* not in close set */
          if (close_list_.find(neighbor_pos) != NULL) {
            continue;
          }
          // if (neighbor_node != NULL && neighbor_node->node_state == IN_CLOSE_SET) {
          //   continue;
          // }

          // if (edt_environment_->sdf_map_->getInflateOccupancy(neighbor_pos) == 1) {
          //   continue;
          // }

          /* ---------- compute cost ---------- */
          double tmp_g_score, tmp_f_score;
          tmp_g_score = d_pos.norm() + cur_node->g_score;  // MEMO(ChanJoon) squareNorm()
          tmp_f_score = tmp_g_score + lambda_heuristic_ * getEuclHeu(neighbor_pos, _target);
          NodePtr tmp_node = expanded_nodes_.find(neighbor_pos);

          if (tmp_node == NULL) {
            NodePtr neighbor_node = path_node_pool_[use_node_num_];
            neighbor_node->position = neighbor_pos;
            neighbor_node->parent = cur_node;
            neighbor_node->g_score = tmp_g_score;
            neighbor_node->f_score = tmp_f_score;
            neighbor_node->node_state = IN_OPEN_SET;
            open_set_.push(neighbor_node);

            expanded_nodes_.insert(neighbor_pos, neighbor_node);

            use_node_num_ += 1;

            if (use_node_num_ == allocate_num_) {
              std::cout << "run out of memory." << std::endl;
              return createResultDataObject(cur_node, timer, use_node_num_, false, _source, 0);
            }
          } else if (tmp_g_score < tmp_node->g_score) {
              tmp_node->parent = cur_node;
              tmp_node->g_score = tmp_g_score;
              tmp_node->f_score = tmp_f_score;
          }

          /* ----------  ---------- */
        }
  }

  /* ---------- open set empty, no path ---------- */
  std::cout << "open set empty, no path!" << std::endl;
  std::cout << "use node num: " << use_node_num_ << std::endl;
  std::cout << "iter num: " << iter_num_ << std::endl;
  return createResultDataObject(cur_node, timer, use_node_num_, false, _source, 0);
}
}  // namespace Planners
