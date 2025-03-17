#ifndef ASTARM1_HPP
#define ASTARM1_HPP
/**
 * @file AStarM1.hpp
 * @author Rafael Rey (reyarcenegui@gmail.com)
 * @author Jose Antonio Cobano (jacobsua@upo.es)
 * 
 * @brief This algorithm is a variation of the A*. The only
 * difference is that it reimplements the computeG method adding the
 * following cost term to the resturned result:
 * 
 *  auto cost_term = static_cast<unsigned int>(cost_weight_ * _suc->cost * dist_scale_factor_reduced_);
 * 
 * @version 0.1
 * @date 2021-06-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <Planners/AStar.hpp>

namespace Planners{
    class AStarM1 : public AStar
    {
        
    public:
        AStarM1();
        AStarM1(std::string _name);

    protected:
        inline virtual unsigned int computeG(const Node* _current, Node* _suc,  unsigned int _n_i, unsigned int _dirs) override;

    };

}

#endif 
