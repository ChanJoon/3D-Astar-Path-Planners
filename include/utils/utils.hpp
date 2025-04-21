#ifndef UTILS_HPP
#define UTILS_HPP

#include <iostream>
#include <vector>
#include <set>
#include <map>
#include <variant>
#include <cmath>
#include <memory>
#include <Eigen/Eigen>

#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'

class Node0 {
 public:
  /* -------------------- */
  Eigen::Vector3i index;
  Eigen::Vector3d position;
  double g_score, f_score, penalty_g_score;
  Node0 *parent;
  char node_state;

  int motion_state {0};
  int ground_penalty_flag {0};

  // kinodynamic
  Eigen::Matrix<double, 6, 1> state;
  Eigen::Vector3d input;
  double duration;
  double time;  // dyn
  int time_idx;

  /* -------------------- */
  Node0() {
    parent = NULL;
    node_state = NOT_EXPAND;
  }
  ~Node0() {};
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
typedef Node0 *NodePtr;

class NodeComparator {
 public:
  // bool operator()(NodePtr node1, NodePtr node2) { return node1->f_score > node2->f_score; }
  bool operator()(const NodePtr& a, const NodePtr& b) const {
    if (a->f_score != b->f_score) {
        return a->f_score < b->f_score;
    }
    return a < b;
  }
};

template <typename T>
struct matrix_hash0 : std::unary_function<T, size_t> {
  std::size_t operator()(T const &matrix) const {
    size_t seed = 0;
    for (auto i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

class NodeHashTable {
 private:
  /* data */
  std::unordered_map<Eigen::Vector3d, NodePtr, matrix_hash0<Eigen::Vector3d>> data_3d_;
  std::unordered_map<Eigen::Vector4d, NodePtr, matrix_hash0<Eigen::Vector4d>> data_4d_;

 public:
  NodeHashTable(/* args */) {}
  ~NodeHashTable() {}
  void insert(Eigen::Vector3d idx, NodePtr node) { data_3d_.insert(std::make_pair(idx, node)); }
  void insert(Eigen::Vector3d idx, int time_idx, NodePtr node) {
    data_4d_.insert(std::make_pair(Eigen::Vector4d(idx(0), idx(1), idx(2), time_idx), node));
  }

  NodePtr find(Eigen::Vector3d idx) {
    auto iter = data_3d_.find(idx);
    return iter == data_3d_.end() ? NULL : iter->second;
  }
  NodePtr find(Eigen::Vector3d idx, int time_idx) {
    auto iter = data_4d_.find(Eigen::Vector4d(idx(0), idx(1), idx(2), time_idx));
    return iter == data_4d_.end() ? NULL : iter->second;
  }

  void clear() {
    data_3d_.clear();
    data_4d_.clear();
  }
};

namespace Planners
{
    namespace utils
    {
        class Node;

        using DataVariant        = std::variant<std::string, Eigen::Vector3d, std::vector<Eigen::Vector3d>, double, size_t, int, bool, unsigned int>;
        using PathData           = std::map<std::string, DataVariant>;
    }
}

#endif