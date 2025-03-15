#include "Planners/ThetaStarAGR.hpp"

namespace Planners
{
    ThetaStarAGR::ThetaStarAGR(bool _use_3d, float _ground_height_threshold, unsigned int _ground_to_air_transition_cost, float _air_movement_factor, unsigned int _flying_cost) 
        : ThetaStar(_use_3d, "thetastaragr"), 
        ground_height_threshold_(_ground_height_threshold), 
        ground_to_air_transition_cost_(_ground_to_air_transition_cost), 
        air_movement_factor_(_air_movement_factor),
        flying_cost_(_flying_cost)
        {}
        
        ThetaStarAGR::ThetaStarAGR(bool _use_3d, std::string _name, float _ground_height_threshold, unsigned int _ground_to_air_transition_cost, float _air_movement_factor, unsigned int _flying_cost) 
        : ThetaStar(_use_3d, _name), 
        ground_height_threshold_(_ground_height_threshold), 
        ground_to_air_transition_cost_(_ground_to_air_transition_cost), 
        air_movement_factor_(_air_movement_factor),
        flying_cost_(_flying_cost)
    {}

    inline void ThetaStarAGR::ComputeCost(Node* _s_aux, Node* _s2_aux)
    {
        auto distanceParent2 = geometry::distanceBetween2Nodes(_s_aux->parent, _s2_aux);
        line_of_sight_checks_++;

        if (LineOfSight::bresenham3D(_s_aux->parent, _s2_aux, discrete_world_))
        {
            // Determine ground/air modes based on z-coordinate
            bool parent_is_ground = (_s_aux->parent->coordinates.z() < ground_height_threshold_);
            bool s2_is_ground = (_s2_aux->coordinates.z() < ground_height_threshold_);
            unsigned int transition_cost = 0;
            unsigned int mode_factor = 1; // Default for 
            unsigned int flying_cost = 0;
            
            if (parent_is_ground && !s2_is_ground) {
                transition_cost = ground_to_air_transition_cost_; // Only apply for ground-to-air
                mode_factor = air_movement_factor_;
            } else if (!parent_is_ground && !s2_is_ground) {
                mode_factor = air_movement_factor_; // Air-to-air movement
            }
            if (!s2_is_ground) {
                flying_cost = flying_cost_;
            }
            
            unsigned int total_cost = _s_aux->parent->G + (distanceParent2 * mode_factor) + transition_cost + flying_cost;
            
            if (total_cost < _s2_aux->G)
            {
                _s2_aux->parent = _s_aux->parent;
                _s2_aux->G = total_cost;
                _s2_aux->gplush = _s2_aux->G + _s2_aux->H;
            }
        }
        else
        {
            // No line of sight, use direct path from _s_aux to _s2_aux
            auto distance2 = geometry::distanceBetween2Nodes(_s_aux, _s2_aux);
            bool s_aux_is_ground = (_s_aux->coordinates.z() < ground_height_threshold_);
            bool s2_is_ground = (_s2_aux->coordinates.z() < ground_height_threshold_);
            unsigned int transition_cost = 0;
            unsigned int mode_factor = 1;
            unsigned int flying_cost = 0;
            
            if (s_aux_is_ground && !s2_is_ground) {
                transition_cost = ground_to_air_transition_cost_; // Only ground-to-air
                mode_factor = air_movement_factor_;
            } else if (!s_aux_is_ground && !s2_is_ground) {
                mode_factor = air_movement_factor_; // Air-to-air
            }
            if (!s2_is_ground) {
                flying_cost = flying_cost_;
            }
            
            unsigned int total_cost = _s_aux->parent->G + (distance2 * mode_factor) + transition_cost + flying_cost;
            
            if (total_cost < _s2_aux->G)
            {
                _s2_aux->parent = _s_aux;
                _s2_aux->G = total_cost;
                _s2_aux->gplush = _s2_aux->G + _s2_aux->H;
            }
        }
    }
    
    inline unsigned int ThetaStarAGR::computeG(const Node* _current, Node* _suc, unsigned int _n_i, unsigned int _dirs)
    {
        unsigned int cost = _current->G;
        
        // Base distance cost based on direction
        if (_dirs == 8) {
            cost += (_n_i < 4 ? dist_scale_factor_ : dd_2D_);
        } else {
            cost += (_n_i < 6 ? dist_scale_factor_ : (_n_i < 18 ? dd_2D_ : dd_3D_));
        }
        
        // Apply mode-specific costs
        bool current_is_ground = (_current->coordinates.z() < ground_height_threshold_);
        bool suc_is_ground = (_suc->coordinates.z() < ground_height_threshold_);
        unsigned int transition_cost = 0;
        unsigned int mode_factor = 1;
        unsigned int flying_cost = 0;
        
        if (current_is_ground && !suc_is_ground) {
            transition_cost = ground_to_air_transition_cost_; // Only ground-to-air
            mode_factor = air_movement_factor_;
        } else if (!current_is_ground && !suc_is_ground) {
            mode_factor = air_movement_factor_; // Air-to-air
        }
        if (!suc_is_ground) {
            flying_cost = flying_cost_;
        }
        
        cost = cost + (cost * (mode_factor - 1)) + transition_cost + flying_cost; // Adjust base cost with mode factor
        _suc->C = transition_cost; // Store transition cost separately if needed
        
        return cost;
    }
}