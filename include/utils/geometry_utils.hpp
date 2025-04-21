#ifndef GEOMETRY_UTILS_HPP
#define GEOMETRY_UTILS_HPP
/**
 * @file geometry_utils.hpp
 * @author Rafael Rey (reyarcenegui@gmail.com)
* @author Jose Antonio Cobano (jacobsua@upo.es)
 * @brief A set of geometry utilities functions
 * @version 0.1
 * @date 2021-06-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include "utils/LineOfSight.hpp"


namespace Planners
{
    namespace utils
    {
        namespace geometry
        {
            float calculatePathLength(const std::vector<Eigen::Vector3d> &_path, const double &_resolution);
        }//namespace geometry
    }//namespace utils
}

#endif