#include "utils/ros/ROSInterfaces.hpp"

namespace Planners
{
    namespace utils
    {
        geometry_msgs::Point continousPoint(const Eigen::Vector3d &_vec, const double &_res)
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
    } //ns utils
} //ns planners