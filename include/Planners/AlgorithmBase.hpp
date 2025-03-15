#ifndef ALGORITHMBASE_HPP
#define ALGORITHMBASE_HPP
/**
 * @file AlgorithmBase.hpp
 * @author Rafael Rey (reyarcenegui@gmail.com)
 * @author Jose Antonio Cobano (jacobsua@upo.es)
 * 
 * @brief Generic AlgorithmBase Base Class. The algorithms should inherit from this class as far as possible.
 * It implements some generic functions used by all the algorithms
 * It could be extended to add functionalities required by other type of algorithms, not only heuristics ones
 * 
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

#include <cmath>

#include "utils/world.hpp"
#include "utils/heuristic.hpp"
#include "utils/utils.hpp"
#include "utils/time.hpp"
#include "utils/geometry_utils.hpp"
#include "utils/LineOfSight.hpp"
#include "plan_env/edt_environment.h"

namespace Planners
{
    using namespace utils;
    using HeuristicFunction = std::function<unsigned int(Eigen::Vector3i, Eigen::Vector3i)>;

    class Heuristic;
    class Clock;

    class AlgorithmBase
    {

    public:
        /**
         * @brief Construct a new AlgorithmBase object
         * 
         * @param _use_3d: This params allows the algorithm to choose a set of 3D directions of explorations 
         * or a set or 2D directions of explorations. The 2D case is simply 3D but without the directions with Z!=0
         * directions2d = {
         *  { 0, 1, 0 }, {0, -1, 0}, { 1, 0, 0 }, { -1, 0, 0 }, //4 straight elements
         *  { 1, -1, 0 }, { -1, 1, 0 }, { -1, -1, 0 }, { 1, 1, 0 } //4 diagonal elements
         * };
         * directions3d = {
         *
         *  { 0, 1, 0 }, {0, -1, 0}, { 1, 0, 0 }, { -1, 0, 0 }, { 0, 0, 1}, { 0, 0, -1}, //6 first elements
         *
         *  { 1, -1, 0 }, { -1, 1, 0 }, { -1, -1, 0 }, { 1, 1, 0 },  { -1, 0, -1 }, //7-18 inclusive
         *  { 1, 0, 1 }, { 1, 0, -1 }, {-1, 0, 1}, { 0, -1, 1 }, { 0, 1, 1 }, { 0, 1, -1 },  { 0, -1, -1 }, 
         *
         *  { -1, -1, 1 }, { 1, 1, 1 },  { -1, 1, 1 }, { 1, -1, 1 }, { -1, -1, -1 }, { 1, 1, -1 }, { -1, 1, -1 }, { 1, -1, -1 }, 
         *};
         * Note that the the ordering is not trivial, the inner loop that explorates node take advantage of this order to directly use
         * pre-compiled distance depending if the squared norm of the vector is 1,2 or 3.
         * This pre compiled distances appears in the header utils.hpp
         * 
         * 
         * @param _algorithm_name Algorithm name to uniquely identify the type of algorithm. 
         */
        AlgorithmBase(bool _use_3d, const std::string &_algorithm_name);

        void setEnvironment(EDTEnvironment::Ptr &env);

        void setWorldSize(const Eigen::Vector3i &worldSize_,const double _resolution);
        Eigen::Vector3i getWorldSize();
        double getWorldResolution();
        utils::DiscreteWorld* getInnerWorld();
        void setHeuristic(HeuristicFunction heuristic_);
        
        /**
         * @brief Mark a set of coordinates of the map as occupied (blocked)
         * 
         * @param coordinates_ Discrete vector of coordinates
         * @param do_inflate enable inflation (mark surrounding coordinates as occupied)
         * @param steps inflation steps (in multiples of the resolution value)
         */
        void addCollision(const Eigen::Vector3i &coordinates_, bool do_inflate, unsigned int steps);
        
        /**
         * @brief Calls the addCollision with the internal inflation configuration values
         * 
         * @param coordinates_ Discrete coordinates vector
         */
        void addCollision(const Eigen::Vector3i &coordinates_);

        /**
         * @brief Function to use in the future to configure the cost of each node
         * 
         * @param coordinates_ Discrete coordinates
         * @param _cost cost value 
         * @return true 
         * @return false 
         */
        bool configureCellCost(const Eigen::Vector3i &coordinates_, const double &_cost);

        /**
         * @brief Check if a set of discrete coordinates are marked as occupied
         * 
         * @param coordinates_ Discrete vector of coordinates
         * @return true Occupied
         * @return false not occupied
         */
        bool detectCollision(const Eigen::Vector3i &coordinates_);

        /**
         * @brief Main function that should be inherit by each algorithm. 
         * This function should accept two VALID start and goal discrete coordinates and return
         * a PathData object containing the necessary information (path, time....)
         * 
         * @param _source Start discrete coordinates
         * @param _target Goal discrete coordinates
         * @return PathData Results stored as PathData object
         */
        virtual PathData findPath(const Eigen::Vector3i &_source, const Eigen::Vector3i &_target) = 0;

        /**
         * @brief Configure the simple inflation implementation
         * By default the inflation way is "As a Cube"
         * @param _inflate If inflate by default
         * @param _inflation_steps The number of adjacent cells to inflate
         */
        void setInflationConfig(const bool _inflate, const unsigned int _inflation_steps) 
        { do_inflate_ = _inflate; inflate_steps_ = _inflation_steps;}

        virtual void setCostFactor(const float &_factor){ cost_weight_ = _factor; }
        virtual void setMaxLineOfSight(const float &_max_line_of_sight){ max_line_of_sight_cells_ = std::floor(_max_line_of_sight/discrete_world_.getResolution()); }
        virtual void publishOccupationMarkersMap() = 0;

    protected:

        /**
         * @brief Basic inflation function 
         * 
         * @param _ref Discrete coordinates vector
         * @param _directions Directions vector to inflate
         * @param _inflate_steps number of cells to inflate in each direction
         */
        void inflateNodeAsCube(const Eigen::Vector3i &_ref,
                               const CoordinateList &_directions,
                               const unsigned int &_inflate_steps);
        
        /**
         * @brief Create a Result Data Object object
         * 
         * @param _last 
         * @param _timer 
         * @param _explored_nodes 
         * @param _solved 
         * @param _start 
         * @param _sight_checks 
         * @return PathData 
         */
        virtual PathData createResultDataObject(const Node* _last, utils::Clock &_timer, 
                                                const size_t _explored_nodes, bool _solved, 
                                                const Eigen::Vector3i &_start, const unsigned int _sight_checks);

                                                        
        HeuristicFunction heuristic;
        CoordinateList direction;

        utils::DiscreteWorld discrete_world_;
        EDTEnvironment::Ptr edt_environment_;
        unsigned int inflate_steps_{1};
        bool do_inflate_{true};

        double cost_weight_{0};
        unsigned int max_line_of_sight_cells_{0};

        const std::string algorithm_name_{""};

    private:
    };
}
#endif