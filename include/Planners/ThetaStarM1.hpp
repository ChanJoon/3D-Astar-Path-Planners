#ifndef THETASTARM1_HPP
#define THETASTARM1_HPP
/**
 * @file ThetaStarM1.hpp
 * @author Rafael Rey (reyarcenegui@gmail.com)
 * @author Jose Antonio Cobano (jacobsua@upo.es)
 * 
 * @brief This header declares the functions and class 
 * associated to the Cost Aware Theta* Algorithm. It inherits
 * from the original Theta* algorithm and override two functions:
 *  1. ComputeCost  
 *  2. ComputeG
 * 
 * @version 0.1
 * @date 2021-09-20
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <Planners/ThetaStar.hpp>

namespace Planners
{
    class ThetaStarM1 : public ThetaStar
    {

    public:
        ThetaStarM1();
        ThetaStarM1(std::string _name);

    protected:
        inline virtual void ComputeCost(Node *_s_aux, Node *_s2_aux) override;
        inline virtual unsigned int computeG(const Node* _current, Node* _suc,  unsigned int _n_i, unsigned int _dirs) override;

    };

}

#endif
