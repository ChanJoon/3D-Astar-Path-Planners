#include "Planners/AlgorithmBase.hpp"

namespace Planners
{

    AlgorithmBase::AlgorithmBase(const std::string &_algorithm_name = "generic_3d_algorithm"): algorithm_name_(_algorithm_name){
        setHeuristic(&Heuristic::euclidean);
    }

    void AlgorithmBase::setGridMap(GridMap::Ptr& grid_map)
    {
        grid_map_ = grid_map;
    }

    void AlgorithmBase::init(){}

    void AlgorithmBase::reset() {
        expanded_nodes_.clear();
        path_nodes_.clear();
      
        std::set<NodePtr, NodeComparator> empty_set;
        open_set_.swap(empty_set);
      
        for (int i = 0; i < use_node_num_; i++) {
          NodePtr node = path_node_pool_[i];
          node->parent = NULL;
          node->node_state = NOT_EXPAND;
        }
      
        use_node_num_ = 0;
        iter_num_ = 0;
      }

    void AlgorithmBase::setHeuristic(HeuristicFunction heuristic_)
    {
        heuristic = std::bind(heuristic_, std::placeholders::_1, std::placeholders::_2);
    }
    bool AlgorithmBase::detectCollision(Eigen::Vector3d &coordinates_)
    {
        // double dist;
        // dist = edt_environment_->evaluateCoarseEDT(coordinates_, -1.0);
        // return dist <= 0.3;
        return grid_map_->getInflateOccupancy(coordinates_);
    }

    // PathData AlgorithmBase::createResultDataObject(const Node* _last, utils::Clock &_timer, 
    //                                                 const size_t _explored_nodes, bool _solved,
    //                                                 const Eigen::Vector3d &_start, const unsigned int _sight_checks){
                                    
    //     PathData result_data;

    //     result_data["solved"]       = _solved;
    //     result_data["goal_coords"]  = _last->coordinates;
    //     result_data["g_final_node"] = _last->G;

    //     std::vector<Eigen::Vector3d> path;
        
    //     if(_solved){
    //         while (_last != nullptr) {
                
    //             path.push_back(_last->coordinates);
    //             _last = _last->parent;
    //         }
    //     }else{
    //         std::cout << "Error impossible to calculate a solution" << std::endl;
    //     }
        
    //     unsigned int total_cost1{0};
    //     unsigned int total_cost2{0};
        
    //     unsigned int total_H{0};
    //     unsigned int total_G1{0};
    //     unsigned int total_G2{0};
    //     unsigned int total_C{0};

    //     unsigned int total_grid_cost1{0};
    //     unsigned int total_grid_cost2{0};

    //     // auto adjacent_path = utils::geometry::getAdjacentPath(path, edt_environment_);
        
    //     for(size_t i = 0; i < adjacent_path.size() - 1; ++i){
    //         auto node_current = discrete_world_.getNodePtr(adjacent_path[i]);
    //         auto node         = discrete_world_.getNodePtr(adjacent_path[i+1]);
    //         if( node == nullptr )
    //             continue;
            
    //         total_H         += node->H;
    //         total_C         += node->C;

    //         total_grid_cost1 += static_cast<unsigned int>( cost_weight_ * node_current->cost * dist_scale_factor_reduced_ );  // CAA*+M1
    //         total_grid_cost2 += static_cast<unsigned int>( ( ( node->cost + node_current->cost ) / 2) *  cost_weight_ * dist_scale_factor_reduced_); ; //CAA*+M2         

    //         unsigned int g_real1 = utils::geometry::distanceBetween2Nodes(adjacent_path[i], adjacent_path[i+1]); 
    //         unsigned int g_real2 = utils::geometry::distanceBetween2Nodes(adjacent_path[i], adjacent_path[i+1]);  // Conmensurable

    //         total_G1 += g_real1; 
    //         total_G2 += g_real2;            
    //     }
        
    //     total_cost1 = total_G1 + total_grid_cost1; 
    //     total_cost2 = total_G2 + total_grid_cost2; 

    //     result_data["algorithm"]               = algorithm_name_;
    //     result_data["path"]                    = path;
    //     result_data["time_spent"]              = _timer.getElapsedMicroSeconds();
    //     result_data["explored_nodes"]          = _explored_nodes;
    //     result_data["start_coords"]            = _start;
    //     // result_data["path_length"]             = geometry::calculatePathLength(path, edt_environment_->sdf_map_->getResolution());

    //     result_data["total_cost1"]              = total_cost1;
    //     result_data["total_cost2"]              = total_cost2;
    //     result_data["h_cost"]                   = total_H;
    //     result_data["g_cost1"]                  = total_G1;
    //     result_data["g_cost2"]                  = total_G2;
    //     result_data["c_cost"]                   = total_C;
    //     result_data["grid_cost1"]               = total_grid_cost1;
    //     result_data["grid_cost2"]               = total_grid_cost2;

    //     result_data["line_of_sight_checks"]    = _sight_checks;
    
    //     return result_data;
    // }
    PathData AlgorithmBase::createResultDataObject(const Node0* _last, utils::Clock &_timer, 
                                                    const size_t _explored_nodes, bool _solved,
                                                    const Eigen::Vector3d &_start, const unsigned int _sight_checks){
                                    
        PathData result_data;

        result_data["solved"]       = _solved;
        result_data["goal_coords"]  = _last->position;
        result_data["g_final_node"] = _last->g_score;

        std::vector<Eigen::Vector3d> path;

        if(_solved){
            path = getPath();
        }else{
            std::cout << "Error impossible to calculate a solution" << std::endl;
        }
        
        unsigned int total_cost1{0};
        unsigned int total_cost2{0};
        
        unsigned int total_H{0};
        unsigned int total_G1{0};
        unsigned int total_G2{0};
        unsigned int total_C{0};
        
        unsigned int total_grid_cost1{0};
        unsigned int total_grid_cost2{0};
        
        total_cost1 = total_G1 + total_grid_cost1; 
        total_cost2 = total_G2 + total_grid_cost2; 
        
        result_data["algorithm"]               = algorithm_name_;
        result_data["path"]                    = path;
        result_data["time_spent"]              = _timer.getElapsedMicroSeconds();
        result_data["explored_nodes"]          = _explored_nodes;
        result_data["start_coords"]            = _start;
        // result_data["path_length"]             = geometry::calculatePathLength(path, edt_environment_->sdf_map_->getResolution());
        result_data["path_length"]             = geometry::calculatePathLength(path, grid_map_->getResolution());
        
        result_data["total_cost1"]              = total_cost1;
        result_data["total_cost2"]              = total_cost2;
        result_data["h_cost"]                   = total_H;
        result_data["g_cost1"]                  = total_G1;
        result_data["g_cost2"]                  = total_G2;
        result_data["c_cost"]                   = total_C;
        result_data["grid_cost1"]               = total_grid_cost1;
        result_data["grid_cost2"]               = total_grid_cost2;
        
        result_data["line_of_sight_checks"]    = _sight_checks;
        
        return result_data;
    }

    std::vector<Eigen::Vector3d> AlgorithmBase::getPath(){
        std::vector<Eigen::Vector3d> path;
        for (size_t i = 0; i < path_nodes_.size(); ++i) {
          path.push_back(path_nodes_[i]->position);
        }
        return path;
    }

    std::vector<NodePtr> AlgorithmBase::getVisitedNodes() {
        vector<NodePtr> visited;
        visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
        return visited;
    }

    Eigen::Vector3i AlgorithmBase::posToIndex(Eigen::Vector3d pt) {
        Eigen::Vector3i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
        
        return idx;
    }

    int AlgorithmBase::timeToIndex(double time) {
        int idx = floor((time - time_origin_) * inv_time_resolution_);
        return idx;
    }

    void AlgorithmBase::retrievePath(NodePtr end_node) {
        NodePtr cur_node = end_node;
        path_nodes_.push_back(cur_node);
      
        while (cur_node->parent != NULL) {
          cur_node = cur_node->parent;
          path_nodes_.push_back(cur_node);
        }

        reverse(path_nodes_.begin(), path_nodes_.end());
      }
}
