#include "Planners/ThetaStar.hpp"

namespace Planners
{
    ThetaStar::ThetaStar() : AStar("thetastar") {}

    ThetaStar::ThetaStar(std::string _name = "thetastar") : AStar(_name)
    {
        checked_nodes.reset(new std::vector<Eigen::Vector3d>);
        checked_nodes_current.reset(new std::vector<Eigen::Vector3d>);

        checked_nodes->reserve(5000);
        checked_nodes_current->reserve(5000);
    }

    inline void ThetaStar::UpdateVertex(Node *_s, Node *_s2, node_by_position &_index_by_pos)
    {
        unsigned int g_old = _s2->G;

        ComputeCost(_s, _s2);
        if (_s2->G < g_old)
        {
            /*
            The node is erased and after that inserted to simply
            re-order the open list thus we can be sure that the node at
            the front of the list will be the one with the lowest cost
            */
            auto found = _index_by_pos.find(_s2->world_index);
            _index_by_pos.erase(found);
            _index_by_pos.insert(_s2);
        }
    }

    inline void ThetaStar::ComputeCost(Node *_s_aux, Node *_s2_aux)
    {
        auto distanceParent2 = geometry::distanceBetween2Nodes(_s_aux->parent, _s2_aux);
        line_of_sight_checks_++;
        if (LineOfSight::bresenham3D(_s_aux->parent, _s2_aux, edt_environment_))
        {
            if ((_s_aux->parent->G + distanceParent2) < _s2_aux->G)
            {
                _s2_aux->parent = _s_aux->parent;
                _s2_aux->G = _s_aux->parent->G + distanceParent2;
                _s2_aux->gplush = _s2_aux->G + _s2_aux->H;
            }
        }
        else
        {

            auto distance2 = geometry::distanceBetween2Nodes(_s_aux, _s2_aux);
            if ((_s_aux->G + distance2) < _s2_aux->G)
            {
                _s2_aux->parent = _s_aux;
                _s2_aux->G = _s_aux->G + distance2;
                _s2_aux->gplush = _s2_aux->G + _s2_aux->H;
            }
        }
    }

    void ThetaStar::exploreNeighbours(Node *_current, const Eigen::Vector3d &_target, node_by_position &_index_by_pos)
    {

        for (unsigned int i = 0; i < direction.size(); ++i)
        {

            Eigen::Vector3d newCoordinates = _current->coordinates + direction[i];
            Node *successor = discrete_world_.getNodePtr(newCoordinates);

            if (successor == nullptr ||
                successor->isInClosedList ||
                successor->occupied)
                continue;

            if (!successor->isInOpenList)
            {

                successor->parent = _current;
                successor->G = computeG(_current, successor, i, direction.size());
                successor->H = heuristic(successor->coordinates, _target);
                successor->gplush = successor->G + successor->H;
                successor->isInOpenList = true;
                _index_by_pos.insert(successor);
            }

            UpdateVertex(_current, successor, _index_by_pos);
        }
    }
    inline void ThetaStar::ComputeCost(NodePtr current, NodePtr neighbor, const Eigen::Vector3d &d_pos, const Eigen::Vector3d &target)
    {
        double direct_cost = d_pos.norm();
        double new_g_score = current->g_score + direct_cost;

        neighbor->parent = current;
        neighbor->g_score = new_g_score;
        neighbor->f_score = new_g_score + lambda_heuristic_ * getEuclHeu(neighbor->position, target);
    }

    // Theta* specific: Line of sight check between two nodes
    inline bool ThetaStar::LineOfSight(const Eigen::Vector3d &start, const Eigen::Vector3d &end)
    {
        Eigen::Vector3d direction = end - start;
        double distance = direction.norm();
        direction.normalize();

        // Number of steps to check along the line
        int steps = std::ceil(distance / (resolution_ * 0.5));
        steps = std::max(steps, 1); // Ensure at least one check

        for (int i = 1; i < steps; ++i)
        {
            Eigen::Vector3d point = start + direction * (i * distance / steps);

            // Check if point is inside map bounds
            if (point(0) <= origin_(0) || point(0) >= map_size_3d_(0) ||
                point(1) <= origin_(1) || point(1) >= map_size_3d_(1) ||
                point(2) <= origin_(2) || point(2) >= map_size_3d_(2))
            {
                return false;
            }

            // Check if the point is in collision
            // if (edt_environment_->sdf_map_->getInflateOccupancy(point) == true)
            // {
            //     return false;
            // }
            if (edt_environment_->evaluateCoarseEDT(point, -1.0) <= 0.3)
            {
                return false;
            }
        }

        return true;
    }

    // Modified UpdateVertex for Theta*
    inline void ThetaStar::UpdateVertex(NodePtr current, NodePtr neighbor, const Eigen::Vector3d &d_pos, const Eigen::Vector3d &target)
    {
        double old_g_score = neighbor->g_score;
        bool path_changed = false;

        // Theta* core difference: Check if we can connect neighbor to current's parent
        if (current->parent != NULL && LineOfSight(current->parent->position, neighbor->position))
        {
            // If line of sight exists, we can create a direct path
            double direct_cost = (neighbor->position - current->parent->position).norm();
            double new_g_score = current->parent->g_score + direct_cost;

            // If this path is better, use it
            if (new_g_score < neighbor->g_score)
            {
                neighbor->parent = current->parent;
                neighbor->g_score = new_g_score;
                neighbor->f_score = new_g_score + lambda_heuristic_ * getEuclHeu(neighbor->position, target);
                path_changed = true;
            }
        }

        // If we couldn't improve the path using line of sight, fall back to regular A*
        if (!path_changed)
        {
            ComputeCost(current, neighbor, d_pos, target);
            path_changed = neighbor->g_score < old_g_score;
        }

        // If we found a better path via either method
        if (path_changed)
        {
            if (neighbor->node_state == IN_OPEN_SET)
            {
                open_set_.push(neighbor);
            }
            else
            {
                neighbor->node_state = IN_OPEN_SET;
                open_set_.push(neighbor);
            }
        }
    }
}
