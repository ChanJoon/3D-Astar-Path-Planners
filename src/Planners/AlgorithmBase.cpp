#include "Planners/AlgorithmBase.hpp"
#include "plan_env/sdf_map.h"

namespace Planners
{

    AlgorithmBase::AlgorithmBase(const std::string &_algorithm_name = "generic_3d_algorithm"): algorithm_name_(_algorithm_name){
        setHeuristic(&Heuristic::euclidean);
    }
    void AlgorithmBase::setEnvironment(const EDTEnvironment::Ptr &env){
        edt_environment_ = env;
    }

    void AlgorithmBase::setHeuristic(HeuristicFunction heuristic_)
    {
        heuristic = std::bind(heuristic_, std::placeholders::_1, std::placeholders::_2);
    }
    bool AlgorithmBase::detectCollision(Eigen::Vector3d &coordinates_)
    {
        double dist;
        dist = edt_environment_->evaluateCoarseEDT(coordinates_, -1.0);
        return dist <= 0.3;
    }

    PathData AlgorithmBase::createResultDataObject(const Node* _last, utils::Clock &_timer, 
                                                    const size_t _explored_nodes, bool _solved,
                                                    const Eigen::Vector3d &_start, const unsigned int _sight_checks){
                                    
        PathData result_data;

        result_data["solved"]       = _solved;
        result_data["goal_coords"]  = _last->coordinates;
        result_data["g_final_node"] = _last->G;

        CoordinateList path;
        
        if(_solved){
            while (_last != nullptr) {
                
                path.push_back(_last->coordinates);
                _last = _last->parent;
            }
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

        auto adjacent_path = utils::geometry::getAdjacentPath(path, edt_environment_);
        
        for(size_t i = 0; i < adjacent_path.size() - 1; ++i){
            auto node_current = discrete_world_.getNodePtr(adjacent_path[i]);
            auto node         = discrete_world_.getNodePtr(adjacent_path[i+1]);
            if( node == nullptr )
                continue;
            
            total_H         += node->H;
            total_C         += node->C;

            total_grid_cost1 += static_cast<unsigned int>( cost_weight_ * node_current->cost * dist_scale_factor_reduced_ );  // CAA*+M1
            total_grid_cost2 += static_cast<unsigned int>( ( ( node->cost + node_current->cost ) / 2) *  cost_weight_ * dist_scale_factor_reduced_); ; //CAA*+M2         

            unsigned int g_real1 = utils::geometry::distanceBetween2Nodes(adjacent_path[i], adjacent_path[i+1]); 
            unsigned int g_real2 = utils::geometry::distanceBetween2Nodes(adjacent_path[i], adjacent_path[i+1]);  // Conmensurable

            total_G1 += g_real1; 
            total_G2 += g_real2;            
        }
        
        total_cost1 = total_G1 + total_grid_cost1; 
        total_cost2 = total_G2 + total_grid_cost2; 

        result_data["algorithm"]               = algorithm_name_;
        result_data["path"]                    = path;
        result_data["time_spent"]              = _timer.getElapsedMicroSeconds();
        result_data["explored_nodes"]          = _explored_nodes;
        result_data["start_coords"]            = _start;
        result_data["path_length"]             = geometry::calculatePathLength(path, edt_environment_->sdf_map_->getResolution());

        result_data["total_cost1"]              = total_cost1;
        result_data["total_cost2"]              = total_cost2;
        result_data["h_cost"]                   = total_H;
        result_data["g_cost1"]                  = total_G1;
        result_data["g_cost2"]                  = total_G2;
        result_data["c_cost"]                   = total_C;
        result_data["grid_cost1"]               = total_grid_cost1;
        result_data["grid_cost2"]               = total_grid_cost2;

        result_data["line_of_sight_checks"]    = _sight_checks;
        result_data["cost_weight"]             = cost_weight_;
    
        return result_data;
    }

}
