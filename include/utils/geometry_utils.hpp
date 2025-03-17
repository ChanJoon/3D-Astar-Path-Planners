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
#include "utils/utils.hpp"
#include "utils/LineOfSight.hpp"
#include "plan_env/edt_environment.h"


namespace Planners
{
    namespace utils
    {
        namespace geometry
        {
            float calculatePathLength(const CoordinateList &_path, const double &_resolution);

            utils::CoordinateList getAdjacentPath(const utils::CoordinateList &_path, const EDTEnvironment::Ptr &_edt_env);

            unsigned int distanceBetween2Nodes(const Node &_n1, const Node &_n2);

            unsigned int distanceBetween2Nodes(const Node *_n1, const Node *_n2);
            
            unsigned int distanceBetween2Nodes(const Eigen::Vector3d &_v1, const Eigen::Vector3d &_v2);

            unsigned int NodesBetween2Nodes(const Node &_n1, const Node &_n2);

            unsigned int NodesBetween2Nodes(const Node *_n1, const Node *_n2);
            
            unsigned int NodesBetween2Nodes(const Eigen::Vector3d &_v1, const Eigen::Vector3d &_v2);

            Eigen::Vector3d abs(const Eigen::Vector3d &_vec);

            int dotProduct(const Eigen::Vector3i &_v1, const Eigen::Vector3i &_v2);

            double moduleVector(const Eigen::Vector3i &_v);

            double angleBetweenThreePoints(const Eigen::Vector3i &_v1, const Eigen::Vector3i &_v2, const Eigen::Vector3i &_v3);

            double angleBetweenThreePoints(const Eigen::Vector3d &_v1, const Eigen::Vector3d &_v2, const Eigen::Vector3d &_v3);

            double getCircunferenceRadius(const Eigen::Vector3i &_v1, const Eigen::Vector3i &_v2, const Eigen::Vector3i &_v3);

            double getCircunferenceRadius(const Eigen::Vector3d &_v1, const Eigen::Vector3d &_v2, const Eigen::Vector3d &_v3);
        }//namespace geometry
    }//namespace utils
}

#endif