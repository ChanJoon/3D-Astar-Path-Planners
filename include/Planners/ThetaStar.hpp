#ifndef THETASTAR_HPP
#define THETASTAR_HPP

#include <Planners/AStar.hpp>

namespace Planners
{
    class ThetaStar : public AStar
    {
    public:
        ThetaStar(const rclcpp::Node::SharedPtr& node);
        ThetaStar(std::string _name);

        void init() override;
        void setParam();
    protected:
        virtual inline void ComputeCost(NodePtr current, NodePtr neighbor, const Eigen::Vector3d& d_pos, const Eigen::Vector3d& target) override;
        virtual inline void UpdateVertex(NodePtr current, NodePtr neighbor, const Eigen::Vector3d& d_pos, const Eigen::Vector3d& target) override;
        rclcpp::Node::SharedPtr node_;
    };
}

#endif
