#ifndef ASTARM2_HPP
#define ASTARM2_HPP
/**
 * @file AStarM2.hpp
 * @author Rafael Rey (reyarcenegui@gmail.com)
 * @author Jose Antonio Cobano (jacobsua@upo.es)
 * 
 * @brief This Algorithm is the same as the original A* with
 * the only difference in the ComputeG function that is re-implemented
 * It adds a edge-neighbour term to the total G value.
 * 
 * @version 0.1
 * @date 2021-09-21
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <Planners/AStar.hpp>

namespace Planners{
    class AStarM2 : public AStar
    {
        
    public:
        AStarM2();
        AStarM2(std::string _name);


    protected:
        inline virtual unsigned int computeG(const Node* _current, Node* _suc,  unsigned int _n_i, unsigned int _dirs) override;

    };

}

#endif 
