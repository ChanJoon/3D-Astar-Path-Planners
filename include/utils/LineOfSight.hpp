#ifndef BRESENHAM_HPP
#define BRESENHAM_HPP
/**
 * @file LineOfSight.hpp
 * @author Rafael Rey (reyarcenegui@gmail.com)
* @author Jose Antonio Cobano (jacobsua@upo.es)
 * @brief Implementation of line of sight related algorithms
 * @version 0.1
 * @date 2021-06-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "utils/utils.hpp"
#include "utils/geometry_utils.hpp"
#include "plan_env/edt_environment.h"

namespace Planners
{
    namespace utils
    {
        namespace LineOfSight
        {
            bool bresenham3D(const Eigen::Vector3d _lnode, const Eigen::Vector3d _rnode, const EDTEnvironment::Ptr &_edt_env, CoordinateListPtr _visited_nodes);

            /**
             * @brief Implementation of bressenham 3D line of sight algorithm
             * 
             * @param _lnode First node
             * @param _rnode Second node
             * @param _world discrete world in which the nodes are stored. It is used
             * to check if the coordinates that the algorithm iterates are occupied/valids
             * @param _visited_nodes 
             * 
             * @return true If there exists line of sight between both nodes
             * @return false If there is no line of sight between both nodes
             */
            bool bresenham3D(const Node *_lnode, const Node *_rnode, const EDTEnvironment::Ptr &_edt_env, CoordinateListPtr _visited_nodes = nullptr);

            bool bresenham3DWithMaxThreshold(const Node *_lnode, const Node *_rnode, const EDTEnvironment::Ptr &_edt_env, const unsigned int _threshold);
        
            int nodesInLineBetweenTwoNodes(const Node *_lnode, const Node *_rnode, const EDTEnvironment::Ptr &_edt_env, const unsigned int _threshold);
        }
    }
}

#endif