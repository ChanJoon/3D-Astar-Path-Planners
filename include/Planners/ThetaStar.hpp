#ifndef THETASTAR_HPP
#define THETASTAR_HPP
/**
 * @file ThetaStar.hpp
 * @author Rafael Rey (reyarcenegui@gmail.com)
 * @author Jose Antonio Cobano (jacobsua@upo.es)
 * 
 * @brief This class inherit from the AStar Algorithm and 
 * implements the UpdateVertex and ComputeCost functions and
 * re-implements ExploreNeighbours method fromthe AStar class.
 * 
 * @version 0.1
 * @date 2021-06-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <Planners/AStar.hpp>

namespace Planners
{
    class ThetaStar : public AStar
    {

    public:
        ThetaStar();
        ThetaStar(std::string _name);

    protected:
        virtual void UpdateVertex(Node *_s, Node *_s2, node_by_position &_index_by_pos);
        inline virtual void ComputeCost(Node *_s_aux, Node *_s2_aux);
        virtual void exploreNeighbours(Node* _current, const Eigen::Vector3d &_target,node_by_position &_index_by_pos) override;

        std::shared_ptr<std::vector<Eigen::Vector3d>> checked_nodes, checked_nodes_current;

    };

}

#endif
