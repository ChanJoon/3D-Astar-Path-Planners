#ifndef ALGORITHMBASE_HPP
#define ALGORITHMBASE_HPP

#include <cmath>
#include <functional>
#include <iostream>
#include <queue>
#include <set>
#include <vector>

#include <boost/functional/hash.hpp>

#include "plan_env/edt_environment.h"
#include "utils/LineOfSight.hpp"
#include "utils/geometry_utils.hpp"
#include "utils/heuristic.hpp"
#include "utils/time.hpp"
#include "utils/utils.hpp"
#include "utils/world.hpp"

namespace Planners {
using namespace utils;
using HeuristicFunction = std::function<unsigned int(Eigen::Vector3d, Eigen::Vector3d)>;

class Heuristic;
class Clock;

class AlgorithmBase {
 public:
  AlgorithmBase(const std::string &_algorithm_name);

  enum { REACH_HORIZON = 1, REACH_END = 2, NO_PATH = 3, NEAR_END = 4 };

  void setEnvironment(const EDTEnvironment::Ptr &env);
  void init();
  void reset();
  void setHeuristic(HeuristicFunction heuristic_);
  virtual void setCostFactor(const float &_factor) { cost_weight_ = _factor; }

  bool detectCollision(Eigen::Vector3d &coordinates_);

  virtual PathData findPath(Eigen::Vector3d &_source, Eigen::Vector3d &_target) = 0;
  virtual PathData findPath(Eigen::Vector3d _source,
                            Eigen::Vector3d _target,
                            bool dynamic,
                            double time_start) = 0;
  std::vector<Eigen::Vector3d> getPath();
  std::vector<NodePtr> getVisitedNodes();

 protected:
  virtual PathData createResultDataObject(const Node *_last,
                                          utils::Clock &_timer,
                                          const size_t _explored_nodes,
                                          bool _solved,
                                          const Eigen::Vector3d &_start,
                                          const unsigned int _sight_checks);
  virtual PathData createResultDataObject(const Node0 *_last,
                                          utils::Clock &_timer,
                                          const size_t _explored_nodes,
                                          bool _solved,
                                          const Eigen::Vector3d &_start,
                                          const unsigned int _sight_checks);

  HeuristicFunction heuristic;
  EDTEnvironment::Ptr edt_environment_;
  DiscreteWorld discrete_world_;
  std::vector<Eigen::Vector3d> direction = {
      {0, 1, 0},   {0, -1, 0}, {1, 0, 0},   {-1, 0, 0}, {0, 0, 1},    {0, 0, -1},  // 6 first
                                                                                   // elements
      {1, -1, 0},  {-1, 1, 0}, {-1, -1, 0}, {1, 1, 0},  {-1, 0, -1},  // 7-18 inclusive
      {1, 0, 1},   {1, 0, -1}, {-1, 0, 1},  {0, -1, 1}, {0, 1, 1},    {0, 1, -1}, {0, -1, -1},
      {-1, -1, 1}, {1, 1, 1},  {-1, 1, 1},  {1, -1, 1}, {-1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
      {1, -1, -1},
  };

  double cost_weight_{0};

  const std::string algorithm_name_{""};

  /* ---------- data structures ---------- */
  vector<NodePtr> path_node_pool_;
  int use_node_num_ = 0;
  int iter_num_ = 0;
  NodeHashTable expanded_nodes_;
  NodeHashTable close_list_;
  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> open_set_;
  std::vector<NodePtr> path_nodes_;

  /* ---------- variables ---------- */
  Eigen::Vector3d start_vel_, end_vel_, start_acc_;
  Eigen::Matrix<double, 6, 6> phi_;  // state transit matrix
  bool is_shot_succ_ = false;
  Eigen::MatrixXd coef_shot_;
  double t_shot_;

  /* ---------- parameter ---------- */
  /* search */
  double max_tau_, init_max_tau_;
  double max_vel_, max_acc_;
  double w_time_, horizon_, lambda_heuristic_;
  int allocate_num_, check_num_;
  double tie_breaker_;
  bool optimistic_;

  /* map */
  double resolution_, inv_resolution_, time_resolution_, inv_time_resolution_;
  Eigen::Vector3d origin_, map_size_3d_;
  double time_origin_ = 0.0;

  /* helper */
  Eigen::Vector3i posToIndex(Eigen::Vector3d pt);
  int timeToIndex(double time);
  void retrievePath(NodePtr end_node);

 private:
};
}  // namespace Planners
#endif