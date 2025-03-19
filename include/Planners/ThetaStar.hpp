#ifndef THETASTAR_HPP
#define THETASTAR_HPP

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
        inline void ComputeCost(NodePtr current, NodePtr neighbor, const Eigen::Vector3d& d_pos, const Eigen::Vector3d& target);
        inline bool LineOfSight(const Eigen::Vector3d& start, const Eigen::Vector3d& end);
        inline void UpdateVertex(NodePtr current, NodePtr neighbor, const Eigen::Vector3d& d_pos, const Eigen::Vector3d& target);

    };
}

#endif
