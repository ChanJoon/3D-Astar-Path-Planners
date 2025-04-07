#include "utils/LineOfSight.hpp"

namespace Planners
{
    namespace utils
    {
        namespace LineOfSight
        {
            bool fastLOS(const NodePtr& s_node, const NodePtr& e_node, const GridMap::Ptr& grid_map) {
                Eigen::Vector3d start = s_node->position;
                Eigen::Vector3d end = e_node->position;
                
                double resolution = 0.1;
                double step_size = resolution / 2.0;  // 0.05
                
                double distance = (end - start).norm();
                Eigen::Vector3d direction = (end - start).normalized();
                
                for (double d = 0; d <= distance; d += step_size) {
                     Eigen::Vector3d sample = start + direction * d;
                     if (grid_map->getInflateOccupancy(sample)) {
                         return false;
                     }
                }
                
                return true;
            }
            bool bresenham3D(const Node *_lnode, const Node *_rnode, const GridMap::Ptr &_grid_map, std::shared_ptr<std::vector<Eigen::Vector3d>> _visited_nodes)
            {
                return bresenham3D(_lnode->coordinates, _rnode->coordinates, _grid_map, _visited_nodes);
            }
            bool bresenham3D(const Node0 *_lnode, const Node0 *_rnode, const GridMap::Ptr &_grid_map, std::shared_ptr<std::vector<Eigen::Vector3d>> _visited_nodes)
            {
                return bresenham3D(_lnode->position, _rnode->position, _grid_map, _visited_nodes);
            }
            bool bresenham3D(const Eigen::Vector3d _lnode, const Eigen::Vector3d _rnode, const GridMap::Ptr &_grid_map, std::shared_ptr<std::vector<Eigen::Vector3d>> _visited_nodes)
            {
                // if( geometry::distanceBetween2Nodes(_lnode, _rnode) <= dd_3D_ )
                //     return true;
                
                // if( _visited_nodes == nullptr ){ //Case in which its not used
                    // _visited_nodes.reset(new std::vector<Eigen::Vector3d>);
                // }
                int d1, d2;
                Eigen::Vector3d vecS, vecDiff, vec0{_lnode}, vec1{_rnode};
                vecDiff = geometry::abs(vec1 - vec0);

                vec1.x() > vec0.x() ? vecS.x() = 1 : vecS.x() = -1;
                vec1.y() > vec0.y() ? vecS.y() = 1 : vecS.y() = -1;
                vec1.z() > vec0.z() ? vecS.z() = 1 : vecS.z() = -1;

                //Driving axis is X-axis
                if (vecDiff.x() >= vecDiff.y() &&
                    vecDiff.x() >= vecDiff.z())
                {
                    d1 = 2 * vecDiff.y() - vecDiff.x();
                    d2 = 2 * vecDiff.z() - vecDiff.x();
                    while (vec0.x() != vec1.x())
                    {
                        vec0.x() += vecS.x();
                        if (d1 >= 0)
                        {
                            vec0.y() += vecS.y();
                            d1 -= 2 * vecDiff.x();
                        }
                        if (d2 >= 0)
                        {
                            vec0.z() += vecS.z();
                            d2 -= 2 * vecDiff.x();
                        }
                        d1 += 2 * vecDiff.y();
                        d2 += 2 * vecDiff.z();
                        //Check if visitor is occupied and add visitor
                        if (_grid_map->getInflateOccupancy(vec0) == true)
                            return false;
                        if ( _visited_nodes != nullptr )
                            _visited_nodes->push_back(vec0);
                    }
                }
                //Driving axis is Y-axis
                else if (vecDiff.y() >= vecDiff.x() &&
                         vecDiff.y() >= vecDiff.z())
                {
                    d1 = 2 * vecDiff.x() - vecDiff.y();
                    d2 = 2 * vecDiff.z() - vecDiff.y();
                    while (vec0.y() != vec1.y())
                    {
                        vec0.y() += vecS.y();
                        if (d1 >= 0)
                        {
                            vec0.x() += vecS.x();
                            d1 -= 2 * vecDiff.y();
                        }
                        if (d2 >= 0)
                        {
                            vec0.z() += vecS.z();
                            d2 -= 2 * vecDiff.y();
                        }
                        d1 += 2 * vecDiff.x();
                        d2 += 2 * vecDiff.z();
                        //Check if visitor is occupied and add visitor
                        if (_grid_map->getInflateOccupancy(vec0) == true)
                            return false;
                        if ( _visited_nodes != nullptr )
                          _visited_nodes->push_back(vec0);
                    }
                }
                //Driving axis is Z-axis
                else
                {
                    d1 = 2 * vecDiff.y() - vecDiff.z();
                    d2 = 2 * vecDiff.x() - vecDiff.z();
                    while (vec0.z() != vec1.z())
                    {
                        vec0.z() += vecS.z();
                        if (d1 >= 0)
                        {
                            vec0.y() += vecS.y();
                            d1 -= 2 * vecDiff.z();
                        }
                        if (d2 >= 0)
                        {
                            vec0.x() += vecS.x();
                            d2 -= 2 * vecDiff.z();
                        }
                        d1 += 2 * vecDiff.y();
                        d2 += 2 * vecDiff.x();
                        //Check if visitor is occupied and add visitor
                        if (_grid_map->getInflateOccupancy(vec0) == true)
                            return false;
                        if ( _visited_nodes != nullptr )
                            _visited_nodes->push_back(vec0);
                    }
                }

                return true;
            }
        }
    }
}