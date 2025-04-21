#ifndef THETASTARAGR_HPP
#define THETASTARAGR_HPP

#include "Planners/ThetaStar.hpp"

namespace Planners
{
    class ThetaStarAGR : public ThetaStar
    {
    public:
        ThetaStarAGR(const rclcpp::Node::SharedPtr& node);
        ThetaStarAGR(std::string _name);

        void init() override;
        void setParam();
        virtual inline PathData findPath(Eigen::Vector3d _source, Eigen::Vector3d _target) override;
    protected:
        virtual inline void ComputeCost(NodePtr current, NodePtr neighbor, const Eigen::Vector3d& d_pos, const Eigen::Vector3d& target) override;
        virtual inline void UpdateVertex(NodePtr current, NodePtr neighbor, const Eigen::Vector3d& d_pos, const Eigen::Vector3d& target) override;

        double flying_cost_, flying_cost_default_, ground_judge_, barrier_;
        rclcpp::Node::SharedPtr node_;
    };
}

#endif // THETASTARAGR_HPP