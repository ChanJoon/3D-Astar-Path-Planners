#include "Planners/ThetaStarGeneratorSafetyCost.hpp"

namespace Planners
{
    ThetaStarGeneratorSafetyCost::ThetaStarGeneratorSafetyCost(bool _use_3d):ThetaStarGenerator(_use_3d, "thetastarsafetycost") {}
    
    ThetaStarGeneratorSafetyCost::ThetaStarGeneratorSafetyCost(bool _use_3d, std::string _name = "thetastarsafetycost" ):ThetaStarGenerator(_use_3d, _name) {}
    
    void ThetaStarGeneratorSafetyCost::ComputeCost(Node *_s_aux, Node *_s2_aux)
    {
        utils::CoordinateListPtr checked_nodes, checked_nodes_current;
        checked_nodes.reset(new CoordinateList);
        checked_nodes_current.reset(new CoordinateList);

        line_of_sight_checks_++;
        if (LineOfSight::bresenham3D((_s_aux->parent), _s2_aux, discrete_world_, checked_nodes))  
        {
            auto dist2   = geometry::distanceBetween2Nodes(_s_aux->parent, _s2_aux);
            auto edge2   = ComputeEdgeCost(checked_nodes, _s_aux->parent, _s2_aux);

            line_of_sight_checks_++;
            LineOfSight::bresenham3D(_s_aux, _s2_aux, discrete_world_, checked_nodes_current);

            auto dist1   = geometry::distanceBetween2Nodes(_s_aux, _s2_aux);  
            auto edge1   =  ComputeEdgeCost(checked_nodes_current, _s_aux, _s2_aux);

            if ( ( _s_aux->parent->G + dist2 + edge2 ) < ( _s_aux->G + dist1 + edge1)) 
            {
                _s2_aux->parent = _s_aux->parent;
                _s2_aux->G      = _s_aux->parent->G + dist2 + edge2;  // This is the same than A*
                _s2_aux->gplush = _s2_aux->G + _s2_aux->H;
                _s2_aux->C      = edge2;
            }
            else{
                _s2_aux->parent =_s_aux;
                _s2_aux->G      = _s_aux->G + dist1 + edge1;   // This is the same than A*      
                _s2_aux->gplush = _s2_aux->G + _s2_aux->H;
                _s2_aux->C      = edge1; 
            }
        } else {

            _s2_aux->parent=_s_aux;

            line_of_sight_checks_++;
            LineOfSight::bresenham3D(_s_aux, _s2_aux, discrete_world_, checked_nodes);
            
            auto dist1     = geometry::distanceBetween2Nodes(_s_aux, _s2_aux);  
            auto edge1   =  ComputeEdgeCost(checked_nodes, _s_aux, _s2_aux);

            _s2_aux->G     =  _s_aux->G + dist1 + edge1;  // This is the same than A*
            _s2_aux->gplush = _s2_aux->G + _s2_aux->H;
            _s2_aux->C     =  edge1;
        }
    }

    unsigned int ThetaStarGeneratorSafetyCost::ComputeEdgeCost(const utils::CoordinateListPtr _checked_nodes, const Node* _s, const Node* _s2){ 
        
        double dist_cost{0};
        double mean_dist_cost{0};
            
        auto n_checked_nodes = _checked_nodes->size();
        if( n_checked_nodes >= 1 )
            for(auto &it: *_checked_nodes)
                dist_cost += discrete_world_.getNodePtr(it)->cost;

        if( n_checked_nodes > 1){
            mean_dist_cost = (( _s->cost - _s2->cost ) / 2) + dist_cost;
        }
        else if (n_checked_nodes == 1){
            mean_dist_cost = (( _s->cost + _s2->cost ) / 2) + dist_cost;
        }
        else{ 
            mean_dist_cost = (( _s->cost + _s2->cost ) / 2);
        }
        return static_cast<unsigned int>( mean_dist_cost * cost_weight_ * dist_scale_factor_reduced_);
    }

    unsigned int ThetaStarGeneratorSafetyCost::computeG(const Node* _current, Node* _suc,  unsigned int _n_i, unsigned int _dirs){
        
        unsigned int cost = 0;

        if(_dirs  == 8){
            cost += (_n_i < 4 ? dist_scale_factor_ : dd_2D_); //This is more efficient
        }else{
            cost += (_n_i < 6 ? dist_scale_factor_ : (_n_i < 18 ? dd_2D_ : dd_3D_)); //This is more efficient
        }

        double cc = (_current->cost + _suc->cost) / 2;
        
        auto edge_neighbour = static_cast<unsigned int>( cc *  cost_weight_ * dist_scale_factor_reduced_); 
    
        cost += _current->G;
        cost += edge_neighbour;
        _suc->C = edge_neighbour;

        return cost;
    }

}
