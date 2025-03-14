#ifndef ROSINTERFACES_HPP
#define ROSINTERFACES_HPP
/**
 * @file ROSInterfaces.hpp
 * @author Rafael Rey (reyarcenegui@gmail.com)
* @author Jose Antonio Cobano (jacobsua@upo.es)
 * @brief This header contains a set of functions that allows to easily use ROS with the algorithms
 * but allowing separation between the main core algorithms headers and implementation and the ROS features
 * @version 0.1
 * @date 2021-06-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/cost_values.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include "utils/utils.hpp"
#include "Grid3D/grid3d.hpp"
#include "Planners/AlgorithmBase.hpp"

#include <cmath>

namespace Planners
{
    namespace utils
    {

        /**
         * @brief get a geometry_msgs::Point from a discrete position and resolution
         * 
         * @param _vec discrete position
         * @param _res resolution
         * @return geometry_msgs::Point 
         */
        geometry_msgs::Point continousPoint(const Eigen::Vector3i &_vec, const double &_res);

        /**
         * @brief Helper function for the configureWorldFromOccupancy (Grid) function 
         * Given a index an a grid(matrix) width, returns the associated (x,y,0) 
         * discrete coordiantes vector using the origin as (0,0,0) the first element of the array.
         * 
         * @param _index Element of the array
         * @param _grid_width 2D array width or horizontal step
         * @return Eigen::Vector3i 
         */
        inline Eigen::Vector3i indexToXY(const unsigned int &_index, const unsigned int _grid_width);
        /**
         * @brief Configure the inner world of the algorihtm _algorithm using a nav_msgs::OccupancyGrid
         * It used the resolution in the metadata 
         * @param _grid the grid msgs
         * @param _algorithm the algorithm core used
         * @param _set_size = false by default. Set to true to resize the world in every call using the meta data provided by the 
         * occupancy grid
         * @return true Currently always return true
         * @return false Never returns false at this version
         */
        bool configureWorldFromOccupancy(const nav_msgs::OccupancyGrid &_grid, AlgorithmBase &_algorithm, bool _set_size = false);
        
        /**
         * @brief 
         * 
         * @param _grid 
         * @param _algorithm 
         * @param _set_size 
         * @return true 
         * @return false 
         */
        bool configureWorldFromOccupancyWithCosts(const nav_msgs::OccupancyGrid &_grid, AlgorithmBase &_algorithm, bool _set_size = false);

        /**
         * @brief Same as configureWorldFromOccupancy but to use with a pcl::PointCloud
         * 
         * @param _points PointCloud
         * @param _algorithm algorithm object
         * @param _resolution Resolution used to discretize the continous points
         * @return true Currently always return true
         * @return false Never returns false at this version
         */
        bool configureWorldFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &_points, AlgorithmBase &_algorithm, const double &_resolution);

        /**
         * @brief 
         * 
         * @param _grid 
         * @param _algorithm 
         * @return true 
         * @return false 
         */
        bool configureWorldCosts(Grid3d &_grid, AlgorithmBase &_algorithm);

    }
}

#endif