#ifndef UTILS_HPP
#define UTILS_HPP
/**
 * @file utils.hpp
 * @author Rafael Rey (reyarcenegui@gmail.com)
* @author Jose Antonio Cobano (jacobsua@upo.es)
 * @brief A set of utils used alongside the project 
 * @version 0.1
 * @date 2021-06-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <iostream>
#include <vector>
#include <set>
#include <map>
#include <variant>
#include <cmath>
#include <memory>
#include <Eigen/Eigen>
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/identity.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/key_extractors.hpp>


namespace Planners
{
    namespace utils
    {
        class Node;

        using CoordinateList     = std::vector<Eigen::Vector3d>;
        using CoordinateListPtr  = std::shared_ptr<std::vector<Eigen::Vector3d>>;
        using DataVariant        = std::variant<std::string, Eigen::Vector3d, CoordinateList, double, size_t, int, bool, unsigned int>;
        using PathData           = std::map<std::string, DataVariant>;

        //Compile time constants
        static constexpr int const dist_scale_factor_{100};
        //To use with costs
        static constexpr int const dist_scale_factor_reduced_{ dist_scale_factor_ / 100 };
        //Dont touch these ones (diagonal distances in 2D and 3D)
        //The static cast from floating point to integer returns the truncated value 
        //i.e discards the decimal part
        static constexpr int const dd_2D_{static_cast<int>( dist_scale_factor_ * 1.41421356237 )}; //sqrt(2)
        static constexpr int const dd_3D_{static_cast<int>( dist_scale_factor_ * 1.73205080757 )}; //sqrt(3)

        /**
         * @brief 
         * 
         */
        struct gridCell
	    {
	    	float dist{0};
	    	float prob{0};
	    };
        /**
         * @brief 
         * 
         */
        struct gridData
        {
            unsigned int grid_size{0};
            unsigned int grid_size_x{0};
            unsigned int grid_size_y{0};
            unsigned int grid_size_z{0};
            unsigned int sensor_dev{0};
        };
        /**
         * @brief Overload ofstream operator << for std::vector<<CoordinateList>
         * 
         * @tparam T 
         * @param os 
         * @param v 
         * @return std::ostream& 
         */
        template <class T>
        inline std::ostream& operator << (std::ostream& os, const std::vector<T>& v) 
        {

            for (typename std::vector<T>::const_iterator ii = v.begin(); ii != v.end()-1; ++ii)
                os << *ii << ", ";
            
            os << *(v.end()-1);

            return os;
        }

        /**
         * @brief This object store the main information used by the algorithms 
         * 
         */
        class Node
        {
            public:
            Node *parent{nullptr};

            Eigen::Vector3d coordinates;

            unsigned int G{0}, H{0}, C{0};
            
            unsigned int gplush{0};
            unsigned int world_index{0};
            
            double cost{0};

            bool occupied{false};
            bool isInOpenList{false};
            bool isInClosedList{false};
            /**
             * @brief Construct a new Node object
             * 
             * @param coord_ 
             * @param parent_ 
             */
            Node(Eigen::Vector3d coord_, Node *parent_ = nullptr)
            {
                parent = parent_;
                coordinates = coord_;
                G = H =0;
            }
            /**
             * @brief Construct a new Node object
             * 
             */
            Node(){}

            /**
             * @brief Get the Score object
             * 
             * @return unsigned int 
             */
            unsigned int getScore()
            {
                return G + H;
            }

            /**
             * @brief Get the Score With Safety Cost object
             * 
             * @return unsigned int 
             */
            unsigned int getScoreWithSafetyCost()
            {
                return G + H + static_cast<unsigned int>(cost);  //Add the distance cost.
            }

        };
                
        struct IndexByCost {};
        struct IndexByWorldPosition {};

        /**
         * @brief 
         * 
         */
        using MagicalMultiSet = boost::multi_index_container<
          Node*, // the data type stored
          boost::multi_index::indexed_by< // list of indexes
            boost::multi_index::hashed_unique<  //hashed index over 'l'
              boost::multi_index::tag<IndexByWorldPosition>, // give that index a name
              boost::multi_index::member<Node, unsigned int, &Node::world_index> // what will be the index's key
            >,
            boost::multi_index::ordered_non_unique<  //ordered index over 'i1'
              boost::multi_index::tag<IndexByCost>, // give that index a name
              boost::multi_index::member<Node, unsigned int, &Node::gplush> // what will be the index's key
            >
          >
        >;

        typedef MagicalMultiSet::index<IndexByWorldPosition>::type node_by_position;
        typedef MagicalMultiSet::index<IndexByCost>::type          node_by_cost;

    }
}

#endif