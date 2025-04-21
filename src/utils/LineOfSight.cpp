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
        }
    }
}