#include "Planners/KinoDynThetaStarAGR.hpp"

namespace Planners
{
  KinoDynThetaStarAGR::KinoDynThetaStarAGR() : ThetaStar("kinodynthetastaragr") {}

  KinoDynThetaStarAGR::KinoDynThetaStarAGR(std::string _name) : ThetaStar(_name) {}

  void KinoDynThetaStarAGR::setParam()
  {
    lnh_.param("kinodynthetastaragr/resolution", resolution_, -1.0);
    lnh_.param("kinodynthetastaragr/time_resolution", time_resolution_, -1.0);
    lnh_.param("kinodynthetastaragr/lambda_heuristic", lambda_heuristic_, -1.0);
    lnh_.param("kinodynthetastaragr/allocate_num", allocate_num_, -1);

    lnh_.param("kinodynthetastaragr/max_tau", max_tau_, -1.0);
    lnh_.param("kinodynthetastaragr/init_max_tau", init_max_tau_, -1.0);
    lnh_.param("kinodynthetastaragr/max_vel", max_vel_, -1.0);
    lnh_.param("kinodynthetastaragr/max_acc", max_acc_, -1.0);
    lnh_.param("kinodynthetastaragr/w_time", w_time_, -1.0);
    lnh_.param("kinodynthetastaragr/horizon", horizon_, -1.0);
    lnh_.param("kinodynthetastaragr/check_num", check_num_, -1);
    lnh_.param("kinodynthetastaragr/proximity_cost", proximity_cost_, 0.0);

    lnh_.param("kinodynthetastaragr/flying_cost", flying_cost_, 0.0);
    lnh_.param("kinodynthetastaragr/flying_cost_default", flying_cost_default_, 0.0);
    lnh_.param("kinodynthetastaragr/ground_judge", ground_judge_, 0.0);

    ROS_INFO("kinodynthetastaragr/resolution: %f", resolution_);
    ROS_INFO("kinodynthetastaragr/time_resolution: %f", time_resolution_);
    ROS_INFO("kinodynthetastaragr/lambda_heuristic: %f", lambda_heuristic_);
    ROS_INFO("kinodynthetastaragr/allocate_num: %d", allocate_num_);

    ROS_INFO("kinodynthetastaragr/max_tau: %f", max_tau_);
    ROS_INFO("kinodynthetastaragr/init_max_tau: %f", init_max_tau_);
    ROS_INFO("kinodynthetastaragr/max_vel: %f", max_vel_);
    ROS_INFO("kinodynthetastaragr/max_acc: %f", max_acc_);
    ROS_INFO("kinodynthetastaragr/w_time: %f", w_time_);
    ROS_INFO("kinodynthetastaragr/horizon: %f", horizon_);
    ROS_INFO("kinodynthetastaragr/check_num: %d", check_num_);
    ROS_INFO("kinodynthetastaragr/proximity_cost: %f", proximity_cost_);

    ROS_INFO("kinodynthetastaragr/flying_cost: %f", flying_cost_);
    ROS_INFO("kinodynthetastaragr/flying_cost_default: %f", flying_cost_default_);
    ROS_INFO("kinodynthetastaragr/ground_judge: %f", ground_judge_);

    tie_breaker_ = 1.0 + 1.0 / 10000;
  }

  inline void KinoDynThetaStarAGR::ComputeCost(NodePtr current, NodePtr neighbor, const Eigen::Vector3d &d_pos, const Eigen::Vector3d &target)
  {
    double time_to_goal = 0.0;
    double direct_cost = d_pos.norm();
    double tmp_g_score = current->g_score + direct_cost;
    double penalty_g_score = 0.0;
    bool next_motion_state = (neighbor->position[2] > ground_judge_);

    if (next_motion_state)
    {
      penalty_g_score = flying_cost_ * neighbor->position[2] / 2.0 + flying_cost_default_;
      tmp_g_score -= current->penalty_g_score;
      tmp_g_score += penalty_g_score;
    }
    else
    {
      tmp_g_score -= current->penalty_g_score;
    }

    if (current->parent == nullptr)
    { // If null, use original A* cost
      if (tmp_g_score < neighbor->g_score)
      {
        neighbor->parent = current;
        neighbor->g_score = tmp_g_score;
        neighbor->f_score = neighbor->g_score + lambda_heuristic_ * estimateHeuristic(neighbor->position, target, time_to_goal);
        neighbor->motion_state = next_motion_state;
        neighbor->penalty_g_score = penalty_g_score;
      }
      return;
    }

    bool is_parent_ground = (current->parent != nullptr) && !current->parent->motion_state;
    bool is_neighbor_ground = !next_motion_state;
    line_of_sight_checks_++;
    if (LineOfSight::fastLOS(current->parent, neighbor, grid_map_))
    {
      // if (is_parent_ground && is_neighbor_ground && LineOfSight::fastLOS(current->parent, neighbor, grid_map_)) {
      double los_distance = (current->parent->position - neighbor->position).norm();
      tmp_g_score = current->parent->g_score + los_distance;
      if (next_motion_state)
      {
        tmp_g_score -= current->penalty_g_score;
        tmp_g_score += penalty_g_score;
      }
      if (tmp_g_score < neighbor->g_score)
      {
        neighbor->parent = current->parent;
        neighbor->g_score = tmp_g_score;
        neighbor->f_score = neighbor->g_score + lambda_heuristic_ * estimateHeuristic(neighbor->position, target, time_to_goal);
        neighbor->motion_state = next_motion_state;
        neighbor->penalty_g_score = penalty_g_score;
      }
    }
    else if (tmp_g_score < neighbor->g_score)
    {
      neighbor->parent = current;
      neighbor->g_score = tmp_g_score;
      neighbor->f_score = neighbor->g_score + lambda_heuristic_ * estimateHeuristic(neighbor->position, target, time_to_goal);
      neighbor->motion_state = next_motion_state;
      neighbor->penalty_g_score = penalty_g_score;
    }
  }

  inline void KinoDynThetaStarAGR::UpdateVertex(NodePtr current, NodePtr neighbor, const Eigen::Vector3d &d_pos, const Eigen::Vector3d &target)
  {
    double old_g_score = neighbor->g_score;

    ComputeCost(current, neighbor, d_pos, target);

    if (neighbor->g_score < old_g_score)
    {
      if (neighbor->node_state == IN_OPEN_SET)
      {
        open_set_.erase(neighbor);
        open_set_.insert(neighbor);
      }
      else
      {
        neighbor->node_state = IN_OPEN_SET;
        open_set_.insert(neighbor);
      }
    }
  }

  PathData KinoDynThetaStarAGR::findPath(Eigen::Vector3d _source,
                                         Eigen::Vector3d _target,
                                         Eigen::Vector3d start_v,
                                         Eigen::Vector3d start_a,
                                         Eigen::Vector3d target_v)
  {
    utils::Clock timer;
    timer.tic();
    line_of_sight_checks_ = 0;

    start_vel_ = start_v;
    start_acc_ = start_a;

    /* ---------- initialize ---------- */
    NodePtr start_node = path_node_pool_[0];
    start_node->parent = NULL;
    start_node->state.head(3) = _source;
    start_node->state.tail(3) = start_v;
    start_node->g_score = 0.0;

    double time_to_goal = 0.0;
    start_node->f_score = lambda_heuristic_ * estimateHeuristic(start_node->state, _target, time_to_goal);
    start_node->penalty_g_score = 0.0;
    start_node->node_state = IN_OPEN_SET;

    Eigen::VectorXd end_state(6);
    end_state.head(3) = _target;
    end_state.tail(3) = target_v;

    open_set_.insert(start_node);
    use_node_num_ = 1;

    const bool is_flying = start_node->state[2] >= ground_judge_;
    start_node->motion_state = is_flying ? 1 : 0; // 1 for flying, 0 for rolling

    if (is_flying) {
      const double penalty = flying_cost_ * start_node->state[2] / 2.0 + flying_cost_default_;
      start_node->g_score += penalty;
      start_node->f_score += penalty;
      start_node->penalty_g_score = penalty;
    } else {
      start_node->penalty_g_score = 0;
    }

    expanded_nodes_.insert(start_node->state.head(3), start_node);

    NodePtr cur_node = NULL;
    NodePtr terminate_node = NULL;

    bool init_search = false;
    std::vector<Eigen::Vector3d> primitive_to_pub;

    /* ---------- search loop ---------- */
    while (!open_set_.empty())
    {
      primitive_to_pub.clear();
      /* ---------- get lowest f_score node ---------- */
      cur_node = *open_set_.begin();

      /* ---------- determine termination ---------- */
      bool reach_horizon = (cur_node->state.head(3) - _target).norm() < horizon_;
      bool reach_end = abs(cur_node->state(0) - _target(0)) <= resolution_ &&
                       abs(cur_node->state(1) - _target(1)) <= resolution_ &&
                       abs(cur_node->state(2) - _target(2)) <= resolution_;

      if (reach_horizon || reach_end) {
        terminate_node = cur_node;
        //TODO(ChanJoon)
        retrievePath(terminate_node);

        if (reach_end) {
          // Check whether shot traj exist
          //TODO(ChanJoon)
          // estimateHeuristic(cur_node->state, end_state, time_to_goal);
          // computeShotTraj(cur_node->state, end_state, time_to_goal);
          if (init_search)
            ROS_ERROR("Shot in first search loop!");
        }

        return createResultDataObject(terminate_node, timer, use_node_num_, true, _source, line_of_sight_checks_);
      }

      if (reach_horizon) {
        if (is_shot_succ_) {
          std::cout << "reach end, " << "iter num: " << iter_num_ << std::endl;
          // return REACH_END;
        } else {
          std::cout << "reach horizon, " << "iter num: " << iter_num_ << std::endl;
          // return REACH_HORIZON;
        }
      }

      if (reach_end) {
        if (is_shot_succ_) {
          std::cout << "reach end, " << "iter num: " << iter_num_ << std::endl;
          // return REACH_END;
        } else if (cur_node->parent != NULL) {
          std::cout << "near end, " << "iter num: " << iter_num_ << std::endl;
          // return NEAR_END;
        } else {
          std::cout << "no path, " << "iter num: " << iter_num_ << std::endl;
          // return NO_PATH;
        }
      }

      /* ---------- pop node and add to close set ---------- */
      open_set_.erase(open_set_.begin());
      cur_node->node_state = IN_CLOSE_SET;
      primitive_to_pub.push_back(cur_node->state.head(3));
      iter_num_ += 1;

      /* ---------- init neighbor expansion ---------- */
      double res = 1 / 2.0, time_res = 1 / 1.0, time_res_init = 1 / 20.0;
      Eigen::Matrix<double, 6, 1> cur_state = cur_node->state;
      Eigen::Matrix<double, 6, 1> neighbor_state;
      std::vector<NodePtr> tmp_expand_nodes;
      Eigen::Vector3d um;
      double pro_t;
      std::vector<Eigen::Vector3d> inputs;
      std::vector<double> durations;

      if (init_search) {
        inputs.push_back(start_acc_);
        for (double tau = time_res_init * init_max_tau_; tau <= init_max_tau_ + 1e-3;
              tau += time_res_init * init_max_tau_)
          durations.push_back(tau);
        init_search = false;
      } else {
        for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ * res)
          for (double ay = -max_acc_; ay <= max_acc_ + 1e-3; ay += max_acc_ * res)
            for (double az =  -max_acc_; az <= max_acc_ + 1e-3; az += max_acc_ * res)
            {
              um << ax, ay, az;
              inputs.push_back(um);
            }
        for (double tau = time_res * max_tau_; tau <= max_tau_; tau += time_res * max_tau_)
          durations.push_back(tau);
      }

      /* ---------- expansion loop ---------- */
      for (int i = 0; i < inputs.size(); i++)
        for (int j = 0; j < durations.size(); j++) {
          um = inputs[i];
          double tau = durations[j];
          stateTransit(cur_state, neighbor_state, um, tau);

          Eigen::Vector3d neighbor_pos = neighbor_state.head(3);
          primitive_to_pub.push_back(neighbor_pos);

          /* ---------- Check if in close set ---------- */
          NodePtr neighbor_node = expanded_nodes_.find(neighbor_pos);
          if (neighbor_node != NULL && neighbor_node->node_state == IN_CLOSE_SET) {
            if (init_search)
              std::cout << "close" << std::endl;
            continue;
          }

          /* ---------- Check maximum velocity ---------- */
          Eigen::Vector3d neighbor_v = neighbor_state.tail(3);
          if (fabs(neighbor_v(0)) > max_vel_ || fabs(neighbor_v(1)) > max_vel_ || fabs(neighbor_v(2)) > max_vel_) {
            if (init_search)
              std::cout << "vel" << std::endl;
            continue;
          }

          /* ---------- Check if in feasible space ---------- */
          Eigen::Vector3d d_pos = neighbor_pos - cur_node->state.head(3);
          if (d_pos.norm() < resolution_ * sqrt(3)) {
            if (init_search)
              std::cout << "norm" << std::endl;
            continue;
          }

          Eigen::Vector3d pos;
          Eigen::Matrix<double, 6, 1> xt;
          bool is_occ = false;
          for (int k = 0; k < check_num_; k++) {
            double dt = tau * double(k) / double(check_num_);
            stateTransit(cur_state, xt, um, dt);
            pos = xt.head(3);
            if (grid_map_->getInflateOccupancy(pos)) {
              is_occ = true;
              break;
            }
          }
          if (is_occ) {
            if (init_search)
              std::cout << "occ" << std::endl;
            continue;
          }

          double time_to_goal, tmp_g_score, tmp_f_score, penalty_g_score;
          bool next_motion_state = neighbor_state[2] > ground_judge_;
          tmp_g_score = (um.squaredNorm() + w_time_) * tau + cur_node->g_score;

          if (next_motion_state) {
            tmp_g_score -= cur_node->penalty_g_score;
            tmp_g_score += flying_cost_ * neighbor_pos[2] / 2.0 + flying_cost_default_;
            penalty_g_score = flying_cost_ * neighbor_pos[2] / 2.0 + flying_cost_default_;
          } else {
            tmp_g_score -= cur_node->penalty_g_score;
            penalty_g_score = 0;
          }
          
          tmp_f_score = tmp_g_score + lambda_heuristic_ * estimateHeuristic(neighbor_state, end_state, time_to_goal);

          /* ---------- Compare nodes expanded from the same parent (pruning) ---------- */
          bool prune = false;
          for (int j = 0; j < tmp_expand_nodes.size(); j++) {
            NodePtr tmp_node = tmp_expand_nodes[j];
            if ((neighbor_pos - tmp_node->state.head(3)).norm() < resolution_ * sqrt(3)) {
              prune = true;
              if (tmp_f_score < tmp_node->f_score) {
                tmp_node->f_score = tmp_f_score;
                tmp_node->g_score = tmp_g_score;
                tmp_node->state = neighbor_state;
                tmp_node->input = um;
                tmp_node->duration = tau;
                tmp_node->motion_state = next_motion_state;
                tmp_node->penalty_g_score = penalty_g_score;
              }
              break;
            }
          }

          if (!prune) {
            if (neighbor_node == nullptr) {
              neighbor_node = path_node_pool_[use_node_num_];
              neighbor_node->state = neighbor_state;
              neighbor_node->f_score = tmp_f_score;
              neighbor_node->g_score = tmp_g_score;
              neighbor_node->input = um;
              neighbor_node->duration = tau;
              neighbor_node->parent = cur_node;
              neighbor_node->node_state = IN_OPEN_SET;
              neighbor_node->motion_state = next_motion_state;
              neighbor_node->penalty_g_score = penalty_g_score;

              open_set_.insert(neighbor_node);
              expanded_nodes_.insert(neighbor_pos, neighbor_node);
              tmp_expand_nodes.push_back(neighbor_node);
              use_node_num_ += 1;
              if (use_node_num_ == allocate_num_) {
                cout << "run out of memory." << endl;
                return createResultDataObject(cur_node, timer, use_node_num_, false, _source, line_of_sight_checks_);
              }
            }
          } else if (neighbor_node->node_state == IN_OPEN_SET) {
            if (tmp_g_score < neighbor_node->g_score) {
              neighbor_node->state = neighbor_state;
              neighbor_node->f_score = tmp_f_score;
              neighbor_node->g_score = tmp_g_score;
              neighbor_node->input = um;
              neighbor_node->duration = tau;
              neighbor_node->parent = cur_node;
              neighbor_node->motion_state = next_motion_state;
              neighbor_node->penalty_g_score = penalty_g_score;
            }
          } else {
            cout << "error type in searching: " << neighbor_node->node_state << endl;
          }
          // UpdateVertex(cur_node, neighbor_node, d_pos, _target);
        }
    }

    /* ---------- open set empty, no path ---------- */
    cout << "open set empty, no path!" << endl;
    cout << "use node num: " << use_node_num_ << endl;
    cout << "iter num: " << iter_num_ << endl;
    return createResultDataObject(cur_node, timer, use_node_num_, false, _source, line_of_sight_checks_);
  }

  double KinoDynThetaStarAGR::estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double &optimal_time)
  {
    const Eigen::Vector3d dp = x2.head(3) - x1.head(3);
    const Eigen::Vector3d v0 = x1.segment(3, 3);
    const Eigen::Vector3d v1 = x2.segment(3, 3);

    double c1 = -36 * dp.dot(dp);
    double c2 = 24 * (v0 + v1).dot(dp);
    double c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
    double c4 = 0;
    double c5 = w_time_;

    std::vector<double> ts = quartic(c5, c4, c3, c2, c1);

    double v_max = max_vel_ * 0.5;
    double t_bar = (x1.head(3) - x2.head(3)).lpNorm<Eigen::Infinity>() / v_max;
    ts.push_back(t_bar);

    double cost = 100000000;
    double t_d = t_bar;

    for (auto t : ts)
    {
      if (t < t_bar)
        continue;
      double c = -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + w_time_ * t;
      if (c < cost)
      {
        cost = c;
        t_d = t;
      }
    }

    optimal_time = t_d;

    return 1.0 * (1 + tie_breaker_) * cost;
  }
  std::vector<double> KinoDynThetaStarAGR::cubic(double a, double b, double c, double d)
  {
    std::vector<double> dts;

    double a2 = b / a;
    double a1 = c / a;
    double a0 = d / a;

    double Q = (3 * a1 - a2 * a2) / 9;
    double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
    double D = Q * Q * Q + R * R;
    if (D > 0)
    {
      double S = std::cbrt(R + sqrt(D));
      double T = std::cbrt(R - sqrt(D));
      dts.push_back(-a2 / 3 + (S + T));
      return dts;
    }
    else if (D == 0)
    {
      double S = std::cbrt(R);
      dts.push_back(-a2 / 3 + S + S);
      dts.push_back(-a2 / 3 - S);
      return dts;
    }
    else
    {
      double theta = acos(R / sqrt(-Q * Q * Q));
      dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
      dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
      dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
      return dts;
    }
  }

  std::vector<double> KinoDynThetaStarAGR::quartic(double a, double b, double c, double d, double e)
  {
    std::vector<double> dts;

    double a3 = b / a;
    double a2 = c / a;
    double a1 = d / a;
    double a0 = e / a;

    std::vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
    double y1 = ys.front();
    double r = a3 * a3 / 4 - a2 + y1;
    if (r < 0)
      return dts;

    double R = sqrt(r);
    double D, E;
    if (R != 0)
    {
      D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 + 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
      E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 - 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    }
    else
    {
      D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
      E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
    }

    if (!std::isnan(D))
    {
      dts.push_back(-a3 / 4 + R / 2 + D / 2);
      dts.push_back(-a3 / 4 + R / 2 - D / 2);
    }
    if (!std::isnan(E))
    {
      dts.push_back(-a3 / 4 - R / 2 + E / 2);
      dts.push_back(-a3 / 4 - R / 2 - E / 2);
    }

    return dts;
  }
  void KinoDynThetaStarAGR::stateTransit(Eigen::Matrix<double, 6, 1> &state0, Eigen::Matrix<double, 6, 1> &state1,
                                      Eigen::Vector3d um, double tau)
  {
    for (int i = 0; i < 3; ++i)
      phi_(i, i + 3) = tau;

    Eigen::Matrix<double, 6, 1> integral;
    integral.head(3) = 0.5 * pow(tau, 2) * um;
    integral.tail(3) = tau * um;

    state1 = phi_ * state0 + integral;
  }
}