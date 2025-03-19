#include "Planners/ThetaStar.hpp"

namespace Planners
{
    ThetaStar::ThetaStar():AStar("thetastar") {}
    
    ThetaStar::ThetaStar(std::string _name = "thetastar" ):AStar(_name) {
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
        if ( LineOfSight::bresenham3D(_s_aux->parent, _s2_aux, edt_environment_) )
        {
            if ( ( _s_aux->parent->G + distanceParent2 ) < _s2_aux->G )
            {
                _s2_aux->parent = _s_aux->parent;
                _s2_aux->G      = _s_aux->parent->G + distanceParent2;
                _s2_aux->gplush = _s2_aux->G + _s2_aux->H;
            }

        } else {

            auto distance2 = geometry::distanceBetween2Nodes(_s_aux, _s2_aux);
            if ( ( _s_aux->G + distance2 ) < _s2_aux->G )
            {
                _s2_aux->parent = _s_aux;
                _s2_aux->G      = _s_aux->G + distance2;
                _s2_aux->gplush = _s2_aux->G + _s2_aux->H;
            }
        }
    }

    void ThetaStar::exploreNeighbours(Node* _current, const Eigen::Vector3d &_target,node_by_position &_index_by_pos){

        for (unsigned int i = 0; i < direction.size(); ++i) {

            Eigen::Vector3d newCoordinates = _current->coordinates + direction[i];
            Node *successor = discrete_world_.getNodePtr(newCoordinates);

            if ( successor == nullptr || 
                 successor->isInClosedList || 
                 successor->occupied )
                continue;
    
            if (! successor->isInOpenList ) { 

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
}
