#ifndef ALGORITHMBASE_HPP
#define ALGORITHMBASE_HPP
/**
 * @file AlgorithmBase.hpp
 * @author Rafael Rey (reyarcenegui@gmail.com)
 * @author Jose Antonio Cobano (jacobsua@upo.es)
 * 
 * @brief Generic AlgorithmBase Base Class. The algorithms should inherit from this class as far as possible.
 * It implements some generic functions used by all the algorithms
 * It could be extended to add functionalities required by other type of algorithms, not only heuristics ones
 * 
 * @version 0.1
 * @date 2021-06-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <iostream>
#include <vector>
#include <set>
#include <functional>
#include <cmath>
#include <queue>
#include <boost/functional/hash.hpp>

#include "utils/heuristic.hpp"
#include "utils/utils.hpp"
#include "utils/time.hpp"
#include "utils/geometry_utils.hpp"
#include "utils/LineOfSight.hpp"
#include "utils/world.hpp"
#include "plan_env/edt_environment.h"

#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'

class Node0 {
    public:
     /* -------------------- */
     Eigen::Vector3i index;
     Eigen::Vector3d position;
     double g_score, f_score;
     Node0* parent;
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
     ~Node0(){};
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   };
   typedef Node0* NodePtr;
   
   class NodeComparator {
    public:
     bool operator()(NodePtr node1, NodePtr node2) {
       return node1->f_score > node2->f_score;
     }
   };
   
   template <typename T>
   struct matrix_hash0 : std::unary_function<T, size_t> {
     std::size_t operator()(T const& matrix) const {
       size_t seed = 0;
       for (size_t i = 0; i < matrix.size(); ++i) {
         auto elem = *(matrix.data() + i);
         seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) +
                 (seed >> 2);
       }
       return seed;
     }
   };
   
   class NodeHashTable {
    private:
     /* data */
     std::unordered_map<Eigen::Vector3d, NodePtr, matrix_hash0<Eigen::Vector3d>>
         data_3d_;
     std::unordered_map<Eigen::Vector4d, NodePtr, matrix_hash0<Eigen::Vector4d>>
         data_4d_;
   
    public:
     NodeHashTable(/* args */) {}
     ~NodeHashTable() {}
     void insert(Eigen::Vector3d idx, NodePtr node) {
       data_3d_.insert(std::make_pair(idx, node));
     }
     void insert(Eigen::Vector3d idx, int time_idx, NodePtr node) {
       data_4d_.insert(std::make_pair(
           Eigen::Vector4d(idx(0), idx(1), idx(2), time_idx), node));
     }
   
     NodePtr find(Eigen::Vector3d idx) {
       auto iter = data_3d_.find(idx);
       return iter == data_3d_.end() ? NULL : iter->second;
     }
     NodePtr find(Eigen::Vector3d idx, int time_idx) {
       auto iter =
           data_4d_.find(Eigen::Vector4d(idx(0), idx(1), idx(2), time_idx));
       return iter == data_4d_.end() ? NULL : iter->second;
     }
   
     void clear() {
       data_3d_.clear();
       data_4d_.clear();
     }
   };
   

namespace Planners
{
    using namespace utils;
    using HeuristicFunction = std::function<unsigned int(Eigen::Vector3d, Eigen::Vector3d)>;

    class Heuristic;
    class Clock;

    class AlgorithmBase
    {

    public:
        AlgorithmBase(const std::string &_algorithm_name);

        enum { REACH_HORIZON = 1, REACH_END = 2, NO_PATH = 3, NEAR_END = 4 };

        void setEnvironment(const EDTEnvironment::Ptr &env);
        void init();
        void reset();
        void setHeuristic(HeuristicFunction heuristic_);
        virtual void setCostFactor(const float &_factor){ cost_weight_ = _factor; }

        bool detectCollision(Eigen::Vector3d &coordinates_);

        virtual PathData findPath(Eigen::Vector3d &_source, Eigen::Vector3d &_target) = 0;
        virtual PathData findPath(Eigen::Vector3d _source, Eigen::Vector3d _target, bool dynamic, double time_start) = 0;
        std::vector<Eigen::Vector3d> getPath();
        std::vector<NodePtr> getVisitedNodes();

    protected:
        virtual PathData createResultDataObject(const Node* _last, utils::Clock &_timer, 
                                                const size_t _explored_nodes, bool _solved, 
                                                const Eigen::Vector3d &_start, const unsigned int _sight_checks);
        virtual PathData createResultDataObject(const Node0* _last, utils::Clock &_timer, 
                                                const size_t _explored_nodes, bool _solved, 
                                                const Eigen::Vector3d &_start, const unsigned int _sight_checks);

                                                        
        HeuristicFunction heuristic;
        EDTEnvironment::Ptr edt_environment_;
        DiscreteWorld discrete_world_;
        std::vector<Eigen::Vector3d> direction = {
            { 0, 1, 0 }, {0, -1, 0}, { 1, 0, 0 }, { -1, 0, 0 }, { 0, 0, 1}, { 0, 0, -1}, //6 first elements
            { 1, -1, 0 }, { -1, 1, 0 }, { -1, -1, 0 }, { 1, 1, 0 },  { -1, 0, -1 }, //7-18 inclusive
            { 1, 0, 1 }, { 1, 0, -1 }, {-1, 0, 1}, { 0, -1, 1 }, { 0, 1, 1 }, { 0, 1, -1 },  { 0, -1, -1 }, 
            { -1, -1, 1 }, { 1, 1, 1 },  { -1, 1, 1 }, { 1, -1, 1 }, { -1, -1, -1 }, { 1, 1, -1 }, { -1, 1, -1 }, { 1, -1, -1 }, 
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
}
#endif