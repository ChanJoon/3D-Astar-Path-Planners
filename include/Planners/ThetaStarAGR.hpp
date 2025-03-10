#ifndef THETASTARAGR_HPP
#define THETASTARAGR_HPP

#include "Planners/ThetaStar.hpp"

namespace Planners
{
    class ThetaStarAGR : public ThetaStar
    {
    public:
        ThetaStarAGR(bool _use_3d,
                     float _ground_height_threshold, unsigned int _ground_to_air_transition_cost, float _air_movement_factor, unsigned int _flying_cost);
        ThetaStarAGR(bool _use_3d, std::string _name,
                     float _ground_height_threshold, unsigned int _ground_to_air_transition_cost, float _air_movement_factor, unsigned int _flying_cost);

    protected:
        inline void ComputeCost(Node* _s_aux, Node* _s2_aux) override;
        inline unsigned int computeG(const Node* _current, Node* _suc, unsigned int _n_i, unsigned int _dirs) override;

        // Configuration parameters
        float ground_height_threshold_{9.0f};           // z < 5.0 is ground
        unsigned int ground_to_air_transition_cost_{0};  // Set to 0 to disable, or e.g., 1000 to enable
        float air_movement_factor_{1.2f};            // Air movement is 2x more expensive
        unsigned int flying_cost_{0};
    };
}

#endif // THETASTARAGR_HPP