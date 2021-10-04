#include "Planners/CostAwareThetaStarGenerator.hpp"

namespace Planners
{
    CostAwareThetaStarGenerator::CostAwareThetaStarGenerator(bool _use_3d):ThetaStarGenerator(_use_3d, "costthetastar") {}
    CostAwareThetaStarGenerator::CostAwareThetaStarGenerator(bool _use_3d, std::string _name = "costthetastar" ):ThetaStarGenerator(_use_3d, _name) {}

    void CostAwareThetaStarGenerator::ComputeCost(Node *_s_aux, Node *_s2_aux)
    {
        auto distanceParent2 = geometry::distanceBetween2Nodes(_s_aux->parent, _s2_aux);
        line_of_sight_checks_++;
        if (LineOfSight::bresenham3DWithMaxThreshold((_s_aux->parent), _s2_aux, discrete_world_, max_line_of_sight_cells_))
        {
            if ((_s_aux->parent->G + distanceParent2 + static_cast<unsigned int>(cost_weight_ * _s_aux->cost)) < (_s2_aux->G))
            {
                _s2_aux->parent = _s_aux->parent;
                _s2_aux->G = _s_aux->parent->G + distanceParent2 +  static_cast<unsigned int>(cost_weight_ * _s_aux->cost);
            }

        } else {
            auto distance2 = geometry::distanceBetween2Nodes(_s_aux, _s2_aux);
            unsigned int G_new = _s_aux->G + distance2 + static_cast<unsigned int>(cost_weight_ * _s_aux->cost);

            if ( G_new < _s2_aux->G){
                _s2_aux->parent=_s_aux;
                _s2_aux->G=_s_aux->G + distance2 +  static_cast<unsigned int>(cost_weight_ * _s_aux->cost);
            }
        }
    }
    unsigned int CostAwareThetaStarGenerator::computeG(const Node* _current, const Node* _suc,  unsigned int _n_i, unsigned int _dirs){
        
        unsigned int cost = _current->G;

        if(_dirs  == 8){
            cost += (_n_i < 4 ? dist_scale_factor_ : dd_2D_); //This is more efficient
        }else{
            cost += (_n_i < 6 ? dist_scale_factor_ : (_n_i < 18 ? dd_2D_ : dd_3D_)); //This is more efficient
        }

        //TODO CHECK that this should be as this
        // cost += _suc->parent->G;

        cost += static_cast<int>(cost_weight_ * _suc->cost);

        return cost;
    }
}
