#ifndef THETASTARAGR_HPP
#define THETASTARAGR_HPP

#include "Planners/ThetaStar.hpp"

namespace Planners
{
    class ThetaStarAGR : public ThetaStar
    {
    public:
        ThetaStarAGR();
        ThetaStarAGR(std::string _name);
        void setParam();

    protected:
        inline void ComputeCost(Node* _s_aux, Node* _s2_aux) override;
        inline unsigned int computeG(const Node* _current, Node* _suc, unsigned int _n_i, unsigned int _dirs) override;
        void exploreNeighbours(Node* _current, const Eigen::Vector3d &_target, node_by_position &_index_by_pos) override;

        // Configuration parameters
        double ground_height_threshold_{9.0};           // z < 5.0 is ground
        int ground_to_air_transition_cost_{0};  // Set to 0 to disable, or e.g., 1000 to enable
        double air_movement_factor_{1.2};            // Air movement is 2x more expensive
        int flying_cost_{0};
    };
}

#endif // THETASTARAGR_HPP