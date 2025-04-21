#ifndef BRESENHAM_HPP
#define BRESENHAM_HPP

#include "utils/utils.hpp"
#include "utils/geometry_utils.hpp"
#include "plan_env/grid_map.h"

namespace Planners
{
    namespace utils
    {
        namespace LineOfSight
        {
            bool fastLOS(const NodePtr& s_node, const NodePtr& e_node, const GridMap::Ptr& grid_map);
        }
    }
}

#endif