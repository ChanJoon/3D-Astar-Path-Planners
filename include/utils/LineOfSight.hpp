#ifndef BRESENHAM_HPP
#define BRESENHAM_HPP

#include "utils/utils.hpp"
#include "utils/geometry_utils.hpp"
// #include "plan_env/edt_environment.h"
#include "plan_env/grid_map.h"

namespace Planners
{
    namespace utils
    {
        namespace LineOfSight
        {
            bool fastLOS(const NodePtr& s_node, const NodePtr& e_node, const GridMap::Ptr& grid_map);
            bool bresenham3D(const Eigen::Vector3d _lnode, const Eigen::Vector3d _rnode, const GridMap::Ptr &_grid_map, std::shared_ptr<std::vector<Eigen::Vector3d>> _visited_nodes);

            bool bresenham3D(const Node *_lnode, const Node *_rnode, const GridMap::Ptr &_grid_map, std::shared_ptr<std::vector<Eigen::Vector3d>> _visited_nodes = nullptr);
            bool bresenham3D(const Node0 *_lnode, const Node0 *_rnode, const GridMap::Ptr &_grid_map, std::shared_ptr<std::vector<Eigen::Vector3d>> _visited_nodes = nullptr);

        }
    }
}

#endif