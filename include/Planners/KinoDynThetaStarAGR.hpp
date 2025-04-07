#ifndef KINODYNTHETASTARAGR_HPP
#define KINODYNTHETASTARAGR_HPP

#include "Planners/ThetaStar.hpp"

namespace Planners
{
    class KinoDynThetaStarAGR : public ThetaStar
    {
    public:
        KinoDynThetaStarAGR();
        KinoDynThetaStarAGR(std::string _name);

        void setParam();
        virtual inline PathData findPath(Eigen::Vector3d _source,
                                         Eigen::Vector3d _target,
                                         Eigen::Vector3d start_v,
                                         Eigen::Vector3d start_a,
                                         Eigen::Vector3d end_v) override;
    protected:
        virtual inline void ComputeCost(NodePtr current, NodePtr neighbor, const Eigen::Vector3d& d_pos, const Eigen::Vector3d& target) override;
        virtual inline void UpdateVertex(NodePtr current, NodePtr neighbor, const Eigen::Vector3d& d_pos, const Eigen::Vector3d& target) override;
        double estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double& optimal_time);
        std::vector<double> cubic(double a, double b, double c, double d);
        std::vector<double> quartic(double a, double b, double c, double d, double e);
        void stateTransit(Eigen::Matrix<double, 6, 1> &state0, Eigen::Matrix<double, 6, 1> &state1, Eigen::Vector3d um, double tau);

        double flying_cost_, flying_cost_default_, ground_judge_;
        double max_tau_, init_max_tau_, max_vel_, max_acc_, w_time_, horizon_, proximity_cost_;
        int check_num_;
        Eigen::Vector3d start_vel_, start_acc_;
    };
}

#endif // KINODYNTHETASTARAGR_HPP