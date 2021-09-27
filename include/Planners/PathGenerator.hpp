#ifndef PATHGENERATOR_HPP
#define PATHGENERATOR_HPP
/**
 * @file PathGenerator.hpp
 * @author Rafael Rey (rreyarc@upo.es)
 * @brief Generic PathGenerator class. The algorithms should inherit from this class as far as possible.
 * It implements some generic functions used by all the algorithms
 * @version 0.1
 * @date 2021-06-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <iostream>
#include <vector>
#include <set>
#include <functional>

#include <math.h>

#include "utils/world.hpp"
#include "utils/heuristic.hpp"
#include "utils/utils.hpp"
#include "utils/time.hpp"
#include "utils/geometry_utils.hpp"
#include "utils/LineOfSight.hpp"

namespace Planners
{
    using namespace utils;
    using HeuristicFunction = std::function<unsigned int(Vec3i, Vec3i)>;

    class Heuristic;
    class Clock;

    /**
     * @brief Main base class that implements useful functions for children algorithm class 
     * and provides a guide to implement any new algorithm.
     */
    class PathGenerator
    {

    public:
        /**
         * @brief Construct a new Path Generator object
         * 
         * @param _use_3d This parameter allows the user to choose between planning on a plane (8 directions possibles) or in the 3D full space (26 directions)
         */
        PathGenerator(bool _use_3d);
        /**
         * @brief Set the World Size object
         * 
         * @param worldSize_ Discrete world size vector
         * @param _resolution resolution to save inside the world object
         */
        void setWorldSize(const Vec3i &worldSize_,const double _resolution);
        /**
         * @brief Get the World Size object
         * 
         * @return Vec3i discrete world bounds
         */
        Vec3i getWorldSize();
        /**
         * @brief Get the World Resolution object
         * 
         * @return double resolution
         */
        double getWorldResolution();
        /**
         * @brief Configure the heuristic
         * 
         * @param heuristic_ Should be one of the static functions of the Heuristic Class
         */
        void setHeuristic(HeuristicFunction heuristic_);
        
        /**
         * @brief Mark a set of coordinates of the map as occupied (blocked)
         * 
         * @param coordinates_ Discrete vector of coordinates
         * @param do_inflate enable inflation (mark surrounding coordinates as occupied)
         * @param steps inflation steps (in multiples of the resolution value)
         */
        void addCollision(const Vec3i &coordinates_, bool do_inflate, unsigned int steps);
        
        /**
         * @brief Calls the addCollision with the internal inflation configuration values
         * 
         * @param coordinates_ Discrete coordinates vector
         */
        void addCollision(const Vec3i &coordinates_);

        /**
         * @brief Function to use in the future to configure the cost of each node
         * 
         * @param coordinates_ Discrete coordinates
         * @param _cost cost value 
         * @return true 
         * @return false 
         */
        bool configureCellCost(const Vec3i &coordinates_, const unsigned int &_cost);

        /**
         * @brief Check if a set of discrete coordinates are marked as occupied
         * 
         * @param coordinates_ Discrete vector of coordinates
         * @return true Occupied
         * @return false not occupied
         */
        bool detectCollision(const Vec3i &coordinates_);

        /**
         * @brief Main function that should be inherit by each algorithm. 
         * This function should accept two VALID start and goal discrete coordinates and return
         * a PathData object containing the necessary information (path, time....)
         * 
         * @param _source Start discrete coordinates
         * @param _target Goal discrete coordinates
         * @return PathData Results stored as PathData object
         */
        virtual PathData findPath(const Vec3i &_source, const Vec3i &_target) = 0;

        /**
         * @brief Configure the simple inflation implementation
         * By default the inflation way is "As a Cube"
         * @param _inflate If inflate by default
         * @param _inflation_steps The number of adjacent cells to inflate
         */
        void setInflationConfig(const bool _inflate, const unsigned int _inflation_steps) 
        { do_inflate_ = _inflate; inflate_steps_ = _inflation_steps;}

        /**
         * @brief Set the Cost Factor object
         * 
         */
        virtual void setCostFactor(const float &_factor){ cost_weight_ = _factor; }

        /**
         * @brief Set the Max Line Of Sight object
         * 
         * @param _max_line_of_sight 
         */
        virtual void setMaxLineOfSight(const float &_max_line_of_sight){ max_line_of_sight_cells_ = std::floor(_max_line_of_sight/discrete_world_.getResolution()); }
        /**
         * @brief Deleted function to be inherit from
         * 
         */
        virtual void publishOccupationMarkersMap() = 0;

    protected:

        /**
         * @brief Basic inflation function 
         * 
         * @param _ref Discrete coordinates vector
         * @param _directions Directions vector to inflate
         * @param _inflate_steps number of cells to inflate in each direction
         */
        void inflateNodeAsCube(const Vec3i &_ref,
                               const CoordinateList &_directions,
                               const unsigned int &_inflate_steps);
        
        HeuristicFunction heuristic;
        CoordinateList direction;

        utils::DiscreteWorld discrete_world_;
        unsigned int inflate_steps_{1};
        bool do_inflate_{true};

        double cost_weight_{0};
        unsigned int max_line_of_sight_cells_;

    private:
    };
}
#endif