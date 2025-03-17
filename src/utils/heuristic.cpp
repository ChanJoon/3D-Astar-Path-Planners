#include "utils/heuristic.hpp"

namespace Planners
{
    Eigen::Vector3d Heuristic::getDelta(const Eigen::Vector3d &_source, const Eigen::Vector3d &_target)
    {
        return {abs(_source.x() - _target.x()), abs(_source.y() - _target.y()), abs(_source.z() - _target.z())};
    }

    unsigned int Heuristic::manhattan(const Eigen::Vector3d &_source, const Eigen::Vector3d &_target)
    {
        const auto delta = getDelta(_source, _target);
        return static_cast<unsigned int>(dist_scale_factor_ * (delta.x() + delta.y() + delta.z()));
    }

    unsigned int Heuristic::euclidean(const Eigen::Vector3d &_source, const Eigen::Vector3d &_target)
    {
        const auto delta = getDelta(_source, _target);
        return static_cast<unsigned int>(dist_scale_factor_ * sqrt(delta.x()*delta.x() + delta.y()*delta.y() + delta.z()*delta.z()));
    }

    unsigned int Heuristic::euclideanOptimized(const Eigen::Vector3d &_source, const Eigen::Vector3d &_target)
    {
        const auto delta = getDelta(_source, _target);
        return static_cast<unsigned int>(1.2 * dist_scale_factor_ * sqrt(delta.x()*delta.x() + delta.y()*delta.y() + delta.z()*delta.z()));
    }
    
    unsigned int Heuristic::euclideanAttractive(const float _factor, const Eigen::Vector3d &_source, const Eigen::Vector3d &_target)
    {
        const auto delta = getDelta(_source, _target);
        return static_cast<unsigned int>(_factor * dist_scale_factor_ * sqrt(delta.x()*delta.x() + delta.y()*delta.y() + delta.z()*delta.z()));
    }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
    unsigned int Heuristic::dijkstra(const Eigen::Vector3d &_source, const Eigen::Vector3d &_target)
    {
        return 0;
    }
#pragma GCC diagnostic pop

    unsigned int Heuristic::octagonal(const Eigen::Vector3d &_source, const Eigen::Vector3d &_target)
    {
        const auto delta = getDelta(_source, _target);
        return dist_scale_factor_ * (delta.x() + delta.y() + delta.z()) + (-6) * std::min(delta.x(), delta.y());
    }
}
