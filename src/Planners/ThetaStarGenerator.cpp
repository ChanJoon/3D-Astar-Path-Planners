#include "Planners/ThetaStarGenerator.hpp"

namespace Planners
{
    void ThetaStarGenerator::UpdateVertex(Node *s, Node *s2, NodeSet &openset)
    {
        float g_old = s2->G;

        ThetaStarGenerator::ComputeCost(s, s2);
        if (s2->G < g_old)
        {
            // This is needed to order the open list.
            if (discrete_world_.isInOpenList(*s2))
                openset.erase(s2);

            openset.insert(s2);
        }
    }

    void ThetaStarGenerator::ComputeCost(Node *s_aux, Node *s2_aux)
    {
        //Include floor to compute de distance
        int factor=10;
        float distanceParent2 = factor * geometry::distanceBetween2Nodes(s_aux->parent, s2_aux);

        if (LineOfSight::bresenham3D((s_aux->parent), s2_aux, discrete_world_))
        {
            if ((s_aux->parent->G + distanceParent2 + s2_aux->H) <
                (s2_aux->G + s2_aux->H))
            {
                s2_aux->parent = s_aux->parent;
                s2_aux->G = s2_aux->parent->G + factor * geometry::distanceBetween2Nodes(s2_aux->parent, s2_aux);
            }
        }
        // JAC: No es necesario este else porque ya se calcula dentro del if (!discrete_world_.isInOpenList(newCoordinates))
        // else
        // {
        //     //JAC: Debo multiplicar por 10 y redondear? Sí
        //     float distanceParent = factor * geometry::distanceBetween2Nodes(s_aux, s2_aux);
        //     //std::cout << "Distance: " << distanceParent << std::endl;

        //     if ((s_aux->G + distanceParent + s2_aux->H) <
        //         (s2_aux->G + s2_aux->H))
        //     {

        //         s2_aux->parent = s_aux;
        //         s2_aux->G = s2_aux->parent->G + factor * geometry::distanceBetween2Nodes(s2_aux->parent, s2_aux);
        //     }
        // }
    }

    PathData ThetaStarGenerator::findPath(const Vec3i &source_, const Vec3i &target_)
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

#if defined(ROS) && defined(PUB_EXPLORED_NODES)
            geometry_msgs::Point point;
            point.x = current->coordinates.x * resolution_;
            point.y = current->coordinates.y * resolution_;
            point.z = current->coordinates.z * resolution_;
            explored_node_marker_.header.stamp = ros::Time();
            explored_node_marker_.header.seq++;
            explored_node_marker_.points.push_back(point);
            explored_nodes_marker_pub_.publish(explored_node_marker_);
#endif

            for (unsigned int i = 0; i < direction.size(); ++i)
            {

                Vec3i newCoordinates(current->coordinates + direction[i]);

                if (discrete_world_.isOccupied(newCoordinates) ||
                    discrete_world_.isInClosedList(newCoordinates))
                    continue;
                unsigned int totalCost = current->G + (i < 6 ? 10 : (i < 18 ? 14.14 : 17.32));

                Node *successor = discrete_world_.getNodePtr(newCoordinates);

                if (successor == nullptr) continue;

                if (!discrete_world_.isInOpenList(newCoordinates))
                {
                    successor->parent = current;
                    successor->G = totalCost;
                    successor->H = heuristic(successor->coordinates, target_);
                    openSet.insert(successor);
                    discrete_world_.setOpenValue(successor->coordinates, true);
                }
                
                UpdateVertex(current, successor, openSet); 
                //Every time a vertex is updated, a line of sight check is 
                line_of_sight_checks++;
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
        result_data["algorithm"] = std::string("thtetastar");
        result_data["path"] = path;
        result_data["time_spent"] = main_timer.getElapsedMillisecs();
        result_data["explored_nodes"] = closedSet.size();
        result_data["start_coords"] = source_;
        result_data["goal_coords"] = target_;
        result_data["path_length"] = geometry::calculatePathLength(path, discrete_world_.getResolution());
        std::cout << "Line of sight checks: " << line_of_sight_checks << std::endl;
        result_data["line_of_sight_checks"] = line_of_sight_checks;

#if defined(ROS) && defined(PUB_EXPLORED_NODES)
        explored_node_marker_.points.clear();
#endif

        discrete_world_.resetWorld();
        return result_data;
    }

}
