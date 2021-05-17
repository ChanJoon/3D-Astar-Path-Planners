#include "Planners/PathGenerator.hpp"

namespace Planners
{

    PathGenerator::PathGenerator(){

    }
    void PathGenerator::setWorldSize(Vec3i worldSize_)
    {
        worldSize = worldSize_;
        discrete_world_.resizeWorld(worldSize.x, worldSize.y, worldSize.z);
    }

    void PathGenerator::setHeuristic(HeuristicFunction heuristic_)
    {
        heuristic = std::bind(heuristic_, std::placeholders::_1, std::placeholders::_2);
    }

    void PathGenerator::addCollision(Vec3i coordinates_, bool do_inflate, bool steps)
    {
        if (do_inflate)
        {
            inflateNodeAsCube(coordinates_, direction, steps);
        }
        else
        {
            discrete_world_.setOccupied(coordinates_);
        }
    }
    bool PathGenerator::detectCollision(Vec3i coordinates_)
    {
        if (discrete_world_.isOccupied(coordinates_))
        {
            return true;
        }
        return false;
    }
    void PathGenerator::inflateNodeAsCube(const Vec3i &_ref, const CoordinateList &_directions, const unsigned int &_inflate_steps)
    {
        for (const auto &it : _directions)
        {
            for (int i = 0; i < _inflate_steps; ++i)
            {
                auto new_vec = _ref + (i + 1) * it;
                discrete_world_.setOccupied(new_vec);
            }
        }
    }

}
