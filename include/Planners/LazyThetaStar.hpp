#ifndef LAZYTHETASTAR_HPP
#define LAZYTHETASTAR_HPP

#include "Planners/ThetaStar.hpp"
#include <string>
#include <Eigen/Eigen>

namespace Planners
{
    class LazyThetaStar : public ThetaStar
    {
    public:
        LazyThetaStar();
        LazyThetaStar(std::string _name);

        void init() override;
        void setParam();
        virtual inline PathData findPath(Eigen::Vector3d _source, Eigen::Vector3d _target) override;

    protected:
        virtual inline void ComputeCost(NodePtr current, NodePtr neighbor, const Eigen::Vector3d& d_pos, const Eigen::Vector3d& target) override;
        virtual inline void UpdateVertex(NodePtr current, NodePtr neighbor, const Eigen::Vector3d& d_pos, const Eigen::Vector3d& target) override;
        void SetVertex(NodePtr current, const Eigen::Vector3d& target);
    };
}

#endif
