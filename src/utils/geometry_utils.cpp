#include "utils/geometry_utils.hpp"

namespace Planners
{
    namespace utils
    {
        namespace geometry
        {

            float calculatePathLength(const std::vector<Eigen::Vector3d> &_path, const double &_resolution)
            {
                float len = 0;
                for (long unsigned int i = 1; i < _path.size(); i++)
                {
                    len += sqrtf(pow((_path[i].x() - _path[i - 1].x()) * _resolution, 2) +
                                 pow((_path[i].y() - _path[i - 1].y()) * _resolution, 2) +
                                 pow((_path[i].z() - _path[i - 1].z()) * _resolution, 2));
                }
                return len;
            }
        }
    }
}