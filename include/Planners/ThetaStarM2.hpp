#ifndef THETASTARM2_HPP
#define THETASTARM2_HPP
/**
 * @file ThetaStarM2.hpp
 * @author Rafael Rey (reyarcenegui@gmail.com)
 * @author Jose Antonio Cobano (jacobsua@upo.es)
 * 
 * @brief This class implements the Theta* Safety Cost
 * version of the original Theta* algorithm. It inherits 
 * from the Theta* class overriding 2 functions:
 * 1.ComputeCost
 * 2. ComputeG
 * It also implements a new function that characterizes this 
 * algorithm which is ComputeEdgeCost
 * 
 * @version 0.1
 * @date 2021-09-10
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <Planners/ThetaStar.hpp>

namespace Planners
{

    /**
     * @brief Theta* Safety Cost Algorithm Class
     * 
     */
    class ThetaStarM2 : public ThetaStar
    {

    public:
        ThetaStarM2();
        ThetaStarM2(std::string _name);

    protected:
        inline virtual void ComputeCost(Node *_s_aux, Node *_s2_aux) override;
        virtual unsigned int ComputeEdgeCost(const utils::CoordinateListPtr _checked_nodes, const Node* _s, const Node* _s2);
        virtual unsigned int computeG(const Node* _current, Node* _suc,  unsigned int _n_i, unsigned int _dirs) override;

    
    };

}

#endif
