#include "utils/LineOfSight.hpp"

namespace Planners
{
    namespace utils
    {
        namespace LineOfSight
        {
            bool bresenham3D(const Node *_lnode, const Node *_rnode, const EDTEnvironment::Ptr &_edt_env, std::shared_ptr<std::vector<Eigen::Vector3d>> _visited_nodes)
            {
                return bresenham3D(_lnode->coordinates, _rnode->coordinates, _edt_env, _visited_nodes);
            }
            bool bresenham3D(const Node0 *_lnode, const Node0 *_rnode, const EDTEnvironment::Ptr &_edt_env, std::shared_ptr<std::vector<Eigen::Vector3d>> _visited_nodes)
            {
                return bresenham3D(_lnode->position, _rnode->position, _edt_env, _visited_nodes);
            }
            bool bresenham3D(const Eigen::Vector3d _lnode, const Eigen::Vector3d _rnode, const EDTEnvironment::Ptr &_edt_env, std::shared_ptr<std::vector<Eigen::Vector3d>> _visited_nodes)
            {
                // if( geometry::distanceBetween2Nodes(_lnode, _rnode) <= dd_3D_ )
                //     return true;
                
                // if( _visited_nodes == nullptr ){ //Case in which its not used
                    // _visited_nodes.reset(new std::vector<Eigen::Vector3d>);
                // }
                int d1, d2;
                Eigen::Vector3d vecS, vecDiff, vec0{_lnode}, vec1{_rnode};
                vecDiff = geometry::abs(vec1 - vec0);

                vec1.x() > vec0.x() ? vecS.x() = 1 : vecS.x() = -1;
                vec1.y() > vec0.y() ? vecS.y() = 1 : vecS.y() = -1;
                vec1.z() > vec0.z() ? vecS.z() = 1 : vecS.z() = -1;

                //Driving axis is X-axis
                if (vecDiff.x() >= vecDiff.y() &&
                    vecDiff.x() >= vecDiff.z())
                {
                    d1 = 2 * vecDiff.y() - vecDiff.x();
                    d2 = 2 * vecDiff.z() - vecDiff.x();
                    while (vec0.x() != vec1.x())
                    {
                        vec0.x() += vecS.x();
                        if (d1 >= 0)
                        {
                            vec0.y() += vecS.y();
                            d1 -= 2 * vecDiff.x();
                        }
                        if (d2 >= 0)
                        {
                            vec0.z() += vecS.z();
                            d2 -= 2 * vecDiff.x();
                        }
                        d1 += 2 * vecDiff.y();
                        d2 += 2 * vecDiff.z();
                        //Check if visitor is occupied and add visitor
                        // if (_edt_env->sdf_map_->getInflateOccupancy(vec0) == 1)
                        //     return false;
                        if (_edt_env->evaluateCoarseEDT(vec0, -1.0) <= 0.3)
                            return false;
                        if ( _visited_nodes != nullptr )
                            _visited_nodes->push_back(vec0);
                    }
                }
                //Driving axis is Y-axis
                else if (vecDiff.y() >= vecDiff.x() &&
                         vecDiff.y() >= vecDiff.z())
                {
                    d1 = 2 * vecDiff.x() - vecDiff.y();
                    d2 = 2 * vecDiff.z() - vecDiff.y();
                    while (vec0.y() != vec1.y())
                    {
                        vec0.y() += vecS.y();
                        if (d1 >= 0)
                        {
                            vec0.x() += vecS.x();
                            d1 -= 2 * vecDiff.y();
                        }
                        if (d2 >= 0)
                        {
                            vec0.z() += vecS.z();
                            d2 -= 2 * vecDiff.y();
                        }
                        d1 += 2 * vecDiff.x();
                        d2 += 2 * vecDiff.z();
                        //Check if visitor is occupied and add visitor
                        // if (_edt_env->sdf_map_->getInflateOccupancy(vec0) == 1)
                        //     return false;
                        if (_edt_env->evaluateCoarseEDT(vec0, -1.0) <= 0.3)
                            return false;
                        if ( _visited_nodes != nullptr )
                          _visited_nodes->push_back(vec0);
                    }
                }
                //Driving axis is Z-axis
                else
                {
                    d1 = 2 * vecDiff.y() - vecDiff.z();
                    d2 = 2 * vecDiff.x() - vecDiff.z();
                    while (vec0.z() != vec1.z())
                    {
                        vec0.z() += vecS.z();
                        if (d1 >= 0)
                        {
                            vec0.y() += vecS.y();
                            d1 -= 2 * vecDiff.z();
                        }
                        if (d2 >= 0)
                        {
                            vec0.x() += vecS.x();
                            d2 -= 2 * vecDiff.z();
                        }
                        d1 += 2 * vecDiff.y();
                        d2 += 2 * vecDiff.x();
                        //Check if visitor is occupied and add visitor
                        // if (_edt_env->sdf_map_->getInflateOccupancy(vec0) == 1)
                        //     return false;
                        if (_edt_env->evaluateCoarseEDT(vec0, -1.0) <= 0.3)
                            return false;
                        if ( _visited_nodes != nullptr )
                            _visited_nodes->push_back(vec0);
                    }
                }

                return true;
            }
            bool bresenham3DWithMaxThreshold(const Node *_lnode, const Node *_rnode, const EDTEnvironment::Ptr &_edt_env, const unsigned int _threshold){
                
                if( utils::geometry::distanceBetween2Nodes(_lnode, _rnode) >= ( dist_scale_factor_ * _threshold ) ) //100 is because of the internal distance units
                    return false;
                
                return bresenham3D(_lnode, _rnode, _edt_env);
            }
            int nodesInLineBetweenTwoNodes(const Node *_lnode, const Node *_rnode, const EDTEnvironment::Ptr &_edt_env, const unsigned int _threshold){
                if( utils::geometry::distanceBetween2Nodes(_lnode, _rnode) >= ( dist_scale_factor_ * _threshold ) ){
                    return 0;
                }
                std::shared_ptr<std::vector<Eigen::Vector3d>> nodes;
                nodes.reset(new std::vector<Eigen::Vector3d>);

                if(bresenham3D(_lnode, _rnode, _edt_env, nodes)){
                    return nodes->size();
                }else{
                    return 0;
                }
            }
        }
    }
}