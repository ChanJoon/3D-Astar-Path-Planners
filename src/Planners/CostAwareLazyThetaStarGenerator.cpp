#include "Planners/CostAwareLazyThetaStarGenerator.hpp"

namespace Planners
{
    void CostAwareLazyThetaStarGenerator::SetVertex(Node *s_aux)
    {   
        if (!LineOfSight::bresenham3DWithMaxThreshold((s_aux->parent), s_aux, discrete_world_, max_line_of_sight_cells_ ))
        {
            unsigned int G_max = std::numeric_limits<unsigned int>::max(); 
            unsigned int G_new;

            for (const auto &i: direction)
            {
                Vec3i newCoordinates(s_aux->coordinates + i);

                if ( discrete_world_.isOccupied(newCoordinates) ) continue;

                if ( discrete_world_.isInClosedList(newCoordinates) )
                {
                    Node *successor2 = discrete_world_.getNodePtr(newCoordinates);
                    if (successor2 == nullptr) continue;

                    G_new = successor2->G +  geometry::distanceBetween2Nodes(successor2, s_aux) + static_cast<int>(cost_weight_ * successor2->cost);
                    if (G_new < G_max)
                    {
                        G_max = G_new;
                        s_aux->parent = successor2;
                        s_aux->G = G_new;
                    }
                }
            }
        }
    }
    void CostAwareLazyThetaStarGenerator::ComputeCost(Node *s_aux, Node *s2_aux)
    {
        auto distanceParent2 = geometry::distanceBetween2Nodes(s_aux->parent, s2_aux);

        if ((s_aux->parent->G + distanceParent2 ) < (s2_aux->G))
        {
            s2_aux->parent = s_aux->parent;
            s2_aux->G = s2_aux->parent->G + geometry::distanceBetween2Nodes(s2_aux->parent, s2_aux) +  static_cast<int>(cost_weight_ * s_aux->cost);
        }
    }

    PathData CostAwareLazyThetaStarGenerator::findPath(const Vec3i &source_, const Vec3i &target_)
    {
        Node *current = nullptr;
        NodeSet openSet, closedSet;
        bool solved{false};

        openSet.insert(discrete_world_.getNodePtr(source_));

        discrete_world_.getNodePtr(source_)->parent = new Node(source_);
        discrete_world_.setOpenValue(source_, true);

        utils::Clock main_timer;
        main_timer.tic();

        int line_of_sight_checks{0};

        while (!openSet.empty())
        {

            current = *openSet.begin();

            if (current->coordinates == target_)
            {
                solved = true;
                break;
            }

            openSet.erase(openSet.begin());
            closedSet.insert(current);

            discrete_world_.setOpenValue(current->coordinates, false);
            discrete_world_.setClosedValue(current->coordinates, true);

            SetVertex(current);
            //in every setVertex the line of sight function is called 
            line_of_sight_checks++;
#if defined(ROS) && defined(PUB_EXPLORED_NODES)
            publishROSDebugData(current, openSet, closedSet);
#endif

            for (unsigned int i = 0; i < direction.size(); ++i)
            {

                Vec3i newCoordinates(current->coordinates + direction[i]);

                if (discrete_world_.isOccupied(newCoordinates) ||
                    discrete_world_.isInClosedList(newCoordinates))
                    continue;
                // unsigned int totalCost = current->G + (i < 6 ? 100 : (i < 18 ? 141 : 173)); //This is more efficient
                Node *successor = discrete_world_.getNodePtr(newCoordinates);

                if (successor == nullptr)
                    continue;

                if (!discrete_world_.isInOpenList(newCoordinates))
                {
                    unsigned int totalCost = current->G;

                    if(direction.size()  == 8){
                        totalCost += (i < 4 ? 100 : 141); //This is more efficient
                    }else{
                        totalCost += (i < 6 ? 100 : (i < 18 ? 141 : 173)); //This is more efficient
                    }

                    successor->parent = current;
                    successor->G = totalCost + static_cast<int>(cost_weight_ * successor->cost);
                    successor->H = heuristic(successor->coordinates, target_);
                    openSet.insert(successor);
                    discrete_world_.setOpenValue(successor->coordinates, true);
                }
                UpdateVertex(current, successor, openSet);
            }
        }
        main_timer.toc();

        PathData result_data;
        result_data["solved"] = solved;

        CoordinateList path;
        if (solved)
        {
            while (current != nullptr)
            {
                path.push_back(current->coordinates);
                current = current->parent;
            }
        }
        else
        {
            std::cout << "Error impossible to calcualte a solution" << std::endl;
        }
        result_data["algorithm"] = std::string("lazythetastar");
        result_data["path"] = path;
        result_data["time_spent"] = main_timer.getElapsedMillisecs();
        result_data["explored_nodes"] = closedSet.size();
        result_data["start_coords"] = source_;
        result_data["goal_coords"] = target_;
        result_data["path_length"] = geometry::calculatePathLength(path, discrete_world_.getResolution());
        result_data["line_of_sight_checks"] = line_of_sight_checks;

#if defined(ROS) && defined(PUB_EXPLORED_NODES)
        explored_node_marker_.points.clear();
#endif

        discrete_world_.resetWorld();
        return result_data;
    }

}
