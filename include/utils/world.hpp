#ifndef WORLD_HPP
#define WORLD_HPP
/**
 * @file world.hpp
 * @author Rafael Rey (reyarcenegui@gmail.com)
* @author Jose Antonio Cobano (jacobsua@upo.es)
 * @brief This header contains an implementation of a discrete cell-like world to use with the planning algorithms
 * @version 0.1
 * @date 2021-06-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <vector>
#include <cmath>
#include <Eigen/Eigen>
#include "utils/utils.hpp"

namespace Planners{

namespace utils
{
    class Node;

    /**
     * @brief Class implementing the data structure storing the map related data that algorithms uses internally
     * 
     */
    class DiscreteWorld
    {

    public:
        /**
         * @brief Construct a new Discrete World object
         * 
         */
        DiscreteWorld()
        {
        }
        /**
         * @brief Set to its default state the flags, cost values, and parent values inside 
         * each world's node
         */
        void resetWorld(){
            
            for(auto &it: discrete_world_vector_){
                it.isInClosedList = false;
                it.isInOpenList = false;
                it.H = it.G = it.C = 0;
                it.parent = nullptr;
            }
        }
        /**
         * @brief Function to check is the node is valid 
         * 
         * @param _x discrete coordinate
         * @param _y discrete coordinate
         * @param _z discrete coordinate
         * @return true if node valid and not occupied
         * @return false if node is outside the workspace of is marked as occupied
         */
        bool isOccupied(const int _x, const int _y, const int _z) const
        {
            if (!checkValid(_x, _y, _z))
                return true;

            if (discrete_world_vector_[getWorldIndex(_x, _y, _z)].occupied)
            {
                return true;
            }

            return false;
        }
        /**
         * @brief Overloaded isOccupied function for Eigen::Vector3d objects
         * 
         * @param _coord discrete coordinates vector
         * @return true if node valid and not occupied 
         * @return false if node is outside the workspace of is marked as occupied 
         */
        bool isOccupied(const Eigen::Vector3d &_coord) const{
            return isOccupied(_coord.x(), _coord.y(), _coord.z());
        }
        /**
         * @brief Overloaded isOccupied function for Node objects
         * 
         * @param _node node object
         * @return true if node valid and not occupied 
         * @return false if node is outside the workspace of is marked as occupied 
         */
        bool isOccupied(const Node &_node) const{
            return isOccupied(_node.coordinates.x(), _node.coordinates.y(), _node.coordinates.z());
        }

        /**
         * @brief Checks the value of the internal flag of the node
         * that is used to mark that the node is in the open list
         * @param _x discrete coordinates
         * @param _y discrete coordinates
         * @param _z discrete coordinates
         * @return true if the node is valid and is in the open list
         * @return false if the node is not valid (outside workspace) or not in the open list
         */
        bool isInOpenList(const int _x, const int _y, const int _z){
            if (!checkValid(_x, _y, _z))
                return false;

            if (discrete_world_vector_[getWorldIndex(_x, _y, _z)].isInOpenList)
                return true;

            return false;
        }
        /**
         * @brief Overloaded function for Eigen::Vector3i objects
         * 
         * @param _coord Discrete coordinates
         * @return true if the node is valid and is in the open list 
         * @return false if the node is not valid (outside workspace) or not in the open list 
         */
        bool isInOpenList(const Eigen::Vector3i &_coord){
            return isInOpenList(_coord.x(), _coord.y(), _coord.z());
        }
        /**
         * @brief Overloaded function for Node objects
         * 
         * @param _node Node object 
         * @return true if the node is valid and is in the open list 
         * @return false if the node is not valid (outside workspace) or not in the open list 
         */
        bool isInOpenList(const Node &_node){
            return isInOpenList(_node.coordinates.x(), _node.coordinates.y(), _node.coordinates.z());
        }
        /**
         * @brief Analogous to isInOpenList
         * 
         * @param _x discrete coordinates
         * @param _y discrete coordinates
         * @param _z discrete coordinates
         * @return true if node is valid and it is in the closed list
         * @return false if node is not in the closed list or it is not valid
         */
        bool isInClosedList(const int _x, const int _y, const int _z){
            if (!checkValid(_x, _y, _z))
                return false;

            if (discrete_world_vector_[getWorldIndex(_x, _y, _z)].isInClosedList)
                return true;

            return false;
        }
        /**
         * @brief Overloaded function for Eigen::Vector3i objects
         * 
         * @param _node 
         * @return true if node is valid and it is in the closed list
         * @return false if node is not in the closed list or it is not valid
         */
        bool isInClosedList(const Eigen::Vector3i &_node){
            return isInClosedList(_node.x(), _node.y(), _node.z());
        }
        /**
         * @brief Overloaded function for Node objects
         * 
         * @param _node 
         * @return true if node is valid and it is in the closed list
         * @return false if node is not in the closed list or it is not valid
         */
        bool isInClosedList(const Node &_node){
            return isInClosedList(_node.coordinates.x(), _node.coordinates.y(), _node.coordinates.z());
        }

        /**
         * @brief SSet the is in open list internal flag of the node associated to 
         * the discrete coordinates
         * 
         * @param _x discrete coordinates
         * @param _y discrete coordinates
         * @param _z discrete coordinates
         * @param _value desired value
         */
        void setOpenValue(const int _x, const int _y, const int _z, const bool _value){
            if (!checkValid(_x, _y, _z))
                return;

            discrete_world_vector_[getWorldIndex(_x, _y, _z)].isInOpenList = _value;
        }
        /**
         * @brief Overloaded function for Eigen::Vector3d. Set the is in open list internal flag of the node associated to 
         * the discrete coordinates
         * 
         * @param _pos discrete coordinates
         * @param _value desired value
         */
        void setOpenValue(const Eigen::Vector3d &_pos, const bool _value){

            if(!checkValid(_pos))
                return;
            
            discrete_world_vector_[getWorldIndex(_pos)].isInOpenList = _value;
        }
        /**
         * @brief Set the Open Value object
         * 
         * @param _node 
         * @param _value 
         */
        void setOpenValue(const Node &_node, const bool _value){
            
            setOpenValue(_node.coordinates, _value);
        }
        /**
         * @brief Get the pointer to the node corresponding to a discrete set of coordinates
         * 
         * @param _vec discrete coordinates
         * @return Pointer to the corresponding node* object or nullptr if requested set of coordinates are not valid
         */
        Node* getNodePtr(const Eigen::Vector3d &_vec) {

            if(!checkValid(_vec))
                return nullptr;

            return &discrete_world_vector_[getWorldIndex(_vec)];
        }
        /**
         * @brief get the inner world object
         * 
         * @return const std::vector<Planners::utils::Node>& Reference to the world object stored inside
         */
        const std::vector<Planners::utils::Node>& getElements() const{
            return discrete_world_vector_;
        }
        /**
         * @brief Get the Resolution stored inside
         * 
         * @return double resolution used internally
         */
        double getResolution() const{
            return resolution_;
        }
        /**
         * @brief Get the World Size internal object (discrete)
         * 
         * @return Eigen::Vector3i 
         */
        Eigen::Vector3i getWorldSize() const{
            return { static_cast<int>(world_x_size_),
                     static_cast<int>(world_y_size_), 
                     static_cast<int>(world_z_size_)};
        }
    private:
        /**
         * @brief checkValid overloaded function for Eigen::Vector3d objects
         * 
         * @param _pos discrete position object
         * @return true if position inside the workspace 
         * @return false if any of the coordinates is bigger than the associated world size dimension
         */
        inline bool checkValid(const Eigen::Vector3d &_pos) const{

            return checkValid(_pos.x(), _pos.y(), _pos.z());
        }
        /**
         * @brief 
         * 
         * @param _x 
         * @param _y 
         * @param _z 
         * @return true if position inside the workspace 
         * @return false if any of the coordinates is bigger than the associated world size dimension
         */
        inline bool checkValid(const int _x, 
                        const int _y, 
                        const int _z) const{

            if ( _x >= world_x_size_ ||
                 _y >= world_y_size_ ||
                 _z >= world_z_size_ )
                return false;

            return true;
        }
        /**
         * @brief getWorldIndex overloaded function for Eigen::Vector3d coordinates
         * 
         * @param _pos discrete position 
         * @return int world index associated to the requested discrete position
         */
        inline long int getWorldIndex(const Eigen::Vector3d &_pos) const{

            return getWorldIndex(_pos.x(), _pos.y(), _pos.z());
        }
        /**
         * @brief Get the world index associated to a set of discrete coordinates
         * 
         * @param x discrete coordinates
         * @param y discrete coordinates
         * @param z discrete coordinates
         * @return int world index of the vector
         */
        inline long int getWorldIndex(const int _x, const int _y, const int _z) const
        {
            // This could be a potential issue but it will never be greater
            // than static_cast<long>(world_x_size_) * world_y_size_ * _world_z_size
            // which is the size of discrete_world_vector_
            // Also, all the functions using this method always should check if the _x _y and _z values
            // are inside the bounds
            return static_cast<long unsigned int>( _z * x_y_size_ + _y * world_x_size_ + _x);
        }
        /**
         * @brief Get the Discrete World Position From Index object
         * 
         * @param _index Discrete index of the internal vector
         * @return Eigen::Vector3d discrete coordinates vector
         */
        inline Eigen::Vector3d getDiscreteWorldPositionFromIndex(const int _index){

            int z = std::floor(_index / x_y_size_ );
            int ind = _index - ( z * x_y_size_ );
            int y = std::floor(ind / world_x_size_);
            int x = std::floor(ind % world_x_size_);

            return Eigen::Vector3d{static_cast<double>(x), static_cast<double>(y), static_cast<double>(z)};
        }

        std::vector<Planners::utils::Node> discrete_world_vector_;

        long unsigned int x_y_size_;

        int world_x_size_, world_y_size_, world_z_size_;
        double resolution_;
    };

}
}

#endif