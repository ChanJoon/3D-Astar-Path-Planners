#ifndef ROSINTERFACES_HPP
#define ROSINTERFACES_HPP

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <math.h>

namespace Planners
{
    namespace utils
    {
        template <typename T, typename U>
        Vec3i discretePoint(const T _point, const U &_res)
        { //Take care of negative values

            return {static_cast<int>(std::floor(_point.x / _res)),
                    static_cast<int>(std::floor(_point.y / _res)),
                    static_cast<int>(std::floor(_point.z / _res))};
        }
        Vec3i discretePoint(const geometry_msgs::Point &_msg, const double &_res)
        {

            return {static_cast<int>(std::floor(_msg.x / _res)),
                    static_cast<int>(std::floor(_msg.y / _res)),
                    static_cast<int>(std::floor(_msg.z / _res))};
        }
        geometry_msgs::Point continousPoint(const Vec3i &_vec, const double &_res)
        {

            geometry_msgs::Point ret;

            ret.x = _vec.x * _res;
            ret.y = _vec.y * _res;
            ret.z = _vec.z * _res;

            return ret;
        }

    }
}

#endif