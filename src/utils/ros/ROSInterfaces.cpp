#include "utils/ros/ROSInterfaces.hpp"

namespace Planners
{
    namespace utils
    {

        geometry_msgs::Point continousPoint(const Eigen::Vector3i &_vec, const double &_res)
        {
            geometry_msgs::Point ret;

            ret.x = _vec.x() * _res;
            ret.y = _vec.y() * _res;
            ret.z = _vec.z() * _res;

            return ret;
        }
        inline Eigen::Vector3i indexToXY(const unsigned int &_index, const unsigned int _grid_width)
        {

            return {static_cast<int>(std::floor(_index % _grid_width)),
                    static_cast<int>(std::floor(_index / _grid_width)),
                    0};
        }

        // bool configureWorldFromOccupancy(const nav_msgs::OccupancyGrid &_grid, AlgorithmBase &_algorithm, bool _set_size)
        // {

        //     if (_set_size)
        //     {
        //         Eigen::Vector3i world_size;
        //         world_size.x() = std::floor(_grid.info.width / _grid.info.resolution);
        //         world_size.y() = std::floor(_grid.info.height / _grid.info.resolution);
        //         world_size.z() = 1;
        //         _algorithm.setWorldSize(world_size, _grid.info.resolution);
        //     }

        //     Eigen::Vector3i cell;
        //     for (long unsigned int i = 0; i < _grid.data.size(); ++i)
        //     {
        //         // 100 should be costmap_2d::LETHAL_OBSTACLE but by the values that map server publishes are between 0 and 100
        //         if (_grid.data[i] == 100)
        //         {
        //             cell = indexToXY(i, _grid.info.width);
        //             _algorithm.addCollision(cell);
        //         }
        //     }

        //     return true;
        // }
        // bool configureWorldFromOccupancyWithCosts(const nav_msgs::OccupancyGrid &_grid, AlgorithmBase &_algorithm, bool _set_size)
        // {

        //     if (_set_size)
        //     {
        //         Eigen::Vector3i world_size;
        //         world_size.x() = std::floor(_grid.info.width / _grid.info.resolution);
        //         world_size.y() = std::floor(_grid.info.height / _grid.info.resolution);
        //         world_size.z() = 1;
        //         _algorithm.setWorldSize(world_size, _grid.info.resolution);
        //     }

        //     Eigen::Vector3i cell;
        //     for (long unsigned int i = 0; i < _grid.data.size(); ++i)
        //     {
        //         // int data = _grid.data[i];
        //         // std::cout << "Cost: " << data << std::endl;
        //         // 100 should be costmap_2d::LETHAL_OBSTACLE but by the values that map server publishes are between 0 and 100
        //         cell = indexToXY(i, _grid.info.width);
        //         if (_grid.data[i] >= 99)
        //             _algorithm.addCollision(cell);
        //         _algorithm.configureCellCost( cell, _grid.data[i] );
        //     }

        //     return true;
        // }
        // bool configureWorldFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &_points, AlgorithmBase &_algorithm, const double &_resolution)
        // {

        //     for (auto &it : *_points)
        //         Eigen::Vector3i discrete_point = {static_cast<int>(std::round(it.x / _resolution)),
        //                                           static_cast<int>(std::round(it.y / _resolution)),
        //                                           static_cast<int>(std::round(it.z / _resolution))};
        //         _algorithm.addCollision(discrete_point);

        //     return true;
        // }

        // bool configureWorldCosts(Grid3d &_grid, AlgorithmBase &_algorithm)
        // {

        //     auto world_size = _algorithm.getWorldSize();
        //     auto resolution = _algorithm.getWorldResolution();

        //     if (world_size.x() <= 0 || world_size.y() <= 0 || world_size.z() <= 0)
        //     {
        //         ROS_ERROR("Invalid world size: [%d, %d, %d]", world_size.x(), world_size.y(), world_size.z());
        //         return false;
        //     }
        //     try {
        //         #pragma omp parallel for collapse(3)
        //         for (int k = 0; k < world_size.z(); k++)
        //         {
        //             for (int j = 0; j < world_size.y(); j++)
        //             {
        //                 for (int i = 0; i < world_size.x(); i++)
        //                 {
        //                     //JAC: Precision
        //                     float cost = _grid.getCellCost(i * resolution, j * resolution, k * resolution);
        //                     _algorithm.configureCellCost({i, j, k}, cost);
        //                 }
        //             }
        //         }
        //     }
        //     catch (const std::exception &e)
        //     {
        //         ROS_ERROR("Exception: %s", e.what());
        //     }

        //     return true;
        // }

    } //ns utils
} //ns planners