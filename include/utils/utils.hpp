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
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/identity.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/key_extractors.hpp>

#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'

class Node0 {
 public:
  /* -------------------- */
  Eigen::Vector3i index;
  Eigen::Vector3d position;
  double g_score, f_score;
  Node0 *parent;
  char node_state;

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

        //Compile time constants
        static constexpr int const dist_scale_factor_{100};
        //To use with costs
        static constexpr int const dist_scale_factor_reduced_{ dist_scale_factor_ / 100 };
        //Dont touch these ones (diagonal distances in 2D and 3D)
        //The static cast from floating point to integer returns the truncated value 
        //i.e discards the decimal part
        static constexpr int const dd_2D_{static_cast<int>( dist_scale_factor_ * 1.41421356237 )}; //sqrt(2)
        static constexpr int const dd_3D_{static_cast<int>( dist_scale_factor_ * 1.73205080757 )}; //sqrt(3)

        /**
         * @brief This object store the main information used by the algorithms 
         * 
         */
        class Node
        {
            public:
            Node *parent{nullptr};

            Eigen::Vector3d coordinates;

            unsigned int G{0}, H{0}, C{0};
            
            unsigned int gplush{0};
            unsigned int world_index{0};
            
            double cost{0};

            bool occupied{false};
            bool isInOpenList{false};
            bool isInClosedList{false};

            Node(Eigen::Vector3d coord_, Node *parent_ = nullptr)
            {
                parent = parent_;
                coordinates = coord_;
                G = H =0;
            }
            Node(){}

        };
                
        struct IndexByCost {};
        struct IndexByWorldPosition {};

        using MagicalMultiSet = boost::multi_index_container<
          Node*, // the data type stored
          boost::multi_index::indexed_by< // list of indexes
            boost::multi_index::hashed_unique<  //hashed index over 'l'
              boost::multi_index::tag<IndexByWorldPosition>, // give that index a name
              boost::multi_index::member<Node, unsigned int, &Node::world_index> // what will be the index's key
            >,
            boost::multi_index::ordered_non_unique<  //ordered index over 'i1'
              boost::multi_index::tag<IndexByCost>, // give that index a name
              boost::multi_index::member<Node, unsigned int, &Node::gplush> // what will be the index's key
            >
          >
        >;

        typedef MagicalMultiSet::index<IndexByWorldPosition>::type node_by_position;
        typedef MagicalMultiSet::index<IndexByCost>::type          node_by_cost;

    }
}

#endif