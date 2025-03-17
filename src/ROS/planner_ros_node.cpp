#include <iostream>

#include "Planners/AStar.hpp"
#include "Planners/AStarM2.hpp"
#include "Planners/AStarM1.hpp"
#include "Planners/ThetaStar.hpp"
#include "Planners/ThetaStarM1.hpp"
#include "Planners/ThetaStarM2.hpp"
#include "Planners/ThetaStarAGR.hpp"
#include "utils/misc.hpp"
#include "utils/geometry_utils.hpp"
#include "utils/metrics.hpp"

// Map
#include "plan_env/edt_environment.h"

#include <ros/ros.h>

#include <visualization_msgs/Marker.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <heuristic_planners/GetPath.h>
#include <heuristic_planners/SetAlgorithm.h>

/**
 * @brief Demo Class that demonstrate how to use the algorithms classes and utils 
 * with ROS 
 * 
 */
class HeuristicPlannerROS
{

public:
    HeuristicPlannerROS()
    {
        
        std::string algorithm_name;
        lnh_.param("algorithm", algorithm_name, (std::string)"astar");
        lnh_.param("heuristic", heuristic_, (std::string)"euclidean");
        
        configureAlgorithm(algorithm_name, heuristic_);
        
        request_path_server_   = lnh_.advertiseService("request_path",  &HeuristicPlannerROS::requestPathService, this);
        change_planner_server_ = lnh_.advertiseService("set_algorithm", &HeuristicPlannerROS::setAlgorithm, this);
        
        line_markers_pub_  = lnh_.advertise<visualization_msgs::Marker>("path_line_markers", 1);
        point_markers_pub_ = lnh_.advertise<visualization_msgs::Marker>("path_points_markers", 1);
        global_path_pub_ = lnh_.advertise<nav_msgs::Path>("global_path", 1);
        
    }
    EDTEnvironment::Ptr edt_environment_;

private:

    bool setAlgorithm(heuristic_planners::SetAlgorithmRequest &_req, heuristic_planners::SetAlgorithmResponse &rep){
        
        configureAlgorithm(_req.algorithm.data, _req.heuristic.data);
        rep.result.data = true;
        return true;
    }
    bool requestPathService(heuristic_planners::GetPathRequest &_req, heuristic_planners::GetPathResponse &_rep){

        if( !_req.algorithm.data.empty() ){
            if( !_req.heuristic.data.empty() ){
                configureAlgorithm(_req.algorithm.data, _req.heuristic.data);
            }else{
                configureAlgorithm(_req.algorithm.data, heuristic_);
            }
        }else if( !_req.heuristic.data.empty() ){
            configureHeuristic(_req.heuristic.data);
        }

        ROS_INFO("Path requested, computing path");
        //delete previous markers
        publishMarker(path_line_markers_, line_markers_pub_);
        publishMarker(path_points_markers_, point_markers_pub_);

        //Astar coordinate list is std::vector<Eigen::Vector3i>
        auto start_pos = Eigen::Vector3d((_req.start.x), (_req.start.y), (_req.start.z));
        auto goal_pos = Eigen::Vector3d((_req.goal.x), (_req.goal.y), (_req.goal.z));

        if( algorithm_->detectCollision(start_pos) ){
            std::cout << start_pos << ": Start not valid" << std::endl;
            return false;
        }

        if( algorithm_->detectCollision(goal_pos) ){
            std::cout << goal_pos << ": Goal not valid" << std::endl;
            return false;
        }

        Eigen::Vector3d origin, size;
        sdf_map_->getRegion(origin, size);
        auto isWithinBounds = [&origin, &size](Eigen::Vector3d &pos){
            return (pos.x() >= origin.x() && pos.y() >= origin.y() && pos.z() >= origin.z() &&
                    pos.x() <= size.x() && pos.y() <= size.y() && pos.z() <= size.z());
        };
        if (!isWithinBounds(start_pos)) {
            std::cout << start_pos << ": Start not within bounds" << std::endl;
            return false;
        }
        if (!isWithinBounds(goal_pos)) {
            std::cout << goal_pos << ": Goal not within bounds" << std::endl;
            return false;
        }

        std::vector<double> times;
        times.reserve(_req.tries.data);
        int real_tries = _req.tries.data;
        if(real_tries == 0) real_tries = 1;

        nav_msgs::Path global_path;
        global_path.header.frame_id = "map";
        global_path.header.stamp = ros::Time::now();

        for(int i = 0; i < real_tries; ++i){
            //TODO(ChanJoon)
            auto path_data = algorithm_->findPath(start_pos, goal_pos);

            if( std::get<bool>(path_data["solved"]) ){
                std::vector<Eigen::Vector3d> path;
                try{
                    _rep.time_spent.data           = std::get<double>(path_data["time_spent"] );
                    _rep.time_spent.data /= 1000;
                    times.push_back(_rep.time_spent.data);

                    if(_req.tries.data < 2 || i == ( _req.tries.data - 1) ){
                        _rep.path_length.data          = std::get<double>(path_data["path_length"] );
                        _rep.explored_nodes.data       = std::get<size_t>(path_data["explored_nodes"] );
                        _rep.line_of_sight_checks.data = std::get<unsigned int>(path_data["line_of_sight_checks"] );

                        _rep.total_cost1.data           = std::get<unsigned int>(path_data["total_cost1"] );
                        _rep.total_cost2.data           = std::get<unsigned int>(path_data["total_cost2"] );
                        _rep.h_cost.data               = std::get<unsigned int>(path_data["h_cost"]);
                        _rep.g_cost1.data               = std::get<unsigned int>(path_data["g_cost1"]);
                        _rep.g_cost2.data               = std::get<unsigned int>(path_data["g_cost2"]);
                        _rep.c_cost.data               = std::get<unsigned int>(path_data["c_cost"]);

                        _rep.cost_weight.data          = std::get<double>(path_data["cost_weight"]);
                        _rep.max_los.data              = std::get<unsigned int>(path_data["max_line_of_sight_cells"]);
                    }
                    path = std::get<std::vector<Eigen::Vector3d>>(path_data["path"]);

                }catch(std::bad_variant_access const& ex){
                    std::cerr << "Bad variant error: " << ex.what() << std::endl;
                }

                if(_req.tries.data < 2 || i == ( _req.tries.data - 1) ){

                    for(const auto &it: path){
                        geometry_msgs::PoseStamped pose;
                        pose.header = global_path.header;
                        pose.pose.position.x = it.x();
                        pose.pose.position.y = it.y();
                        pose.pose.position.z = it.z();
                        pose.pose.orientation.w = 1.0;
                        global_path.poses.push_back(pose);

                        geometry_msgs::Point point;
                        point.x = it.x();
                        point.y = it.y();
                        point.z = it.z();
                        path_line_markers_.points.push_back(point);
                        path_points_markers_.points.push_back(point);
                    }

                    publishMarker(path_line_markers_, line_markers_pub_);
                    publishMarker(path_points_markers_, point_markers_pub_);
                    global_path_pub_.publish(global_path);

                    path_line_markers_.points.clear();
                    path_points_markers_.points.clear();
                    global_path.poses.clear();

                    ROS_INFO("Path calculated succesfully");
                }
            }else{
                ROS_INFO("Could not calculate path between request points");
            }
        }  
        if(_req.tries.data > 2){
            auto av_time = std::accumulate(times.begin(), times.end(), 0.0) / times.size(); 
            std::cout << "Average Time: "      << av_time  << " milisecs" << std::endl;
            std::cout << "Average Frequency: " << 1000/av_time << std::endl;
        }
        return true;
    }
    void configureAlgorithm(const std::string &algorithm_name, const std::string &_heuristic){

        float ws_x, ws_y, ws_z;

        lnh_.param("world_size_x", ws_x, (float)100.0); // In meters
        lnh_.param("world_size_y", ws_y, (float)100.0); // In meters
        lnh_.param("world_size_z", ws_z, (float)100.0); // In meters
        lnh_.param("resolution", resolution_, (float)0.2);
        lnh_.param("inflate_map", inflate_, (bool)true);

        world_size_.x() = std::floor(ws_x / resolution_);
        world_size_.y() = std::floor(ws_y / resolution_);
        world_size_.z() = std::floor(ws_z / resolution_);
        
        if( algorithm_name == "astar" ){
            ROS_INFO("Using A*");
            algorithm_.reset(new Planners::AStar());
        }else if( algorithm_name == "costastar" ){
            ROS_INFO("Using Cost Aware A*");
            algorithm_.reset(new Planners::AStarM1());
        }else if( algorithm_name == "astarsafetycost" ){
            ROS_INFO("Using A* Safety Cost");
            algorithm_.reset(new Planners::AStarM2());    
        }else if ( algorithm_name == "thetastar" ){
            ROS_INFO("Using Theta*");
            algorithm_.reset(new Planners::ThetaStar());
        }else if ( algorithm_name == "costhetastar" ){
            ROS_INFO("Using Cost Aware Theta* ");
            algorithm_.reset(new Planners::ThetaStarM1());
        }else if ( algorithm_name == "thetastarsafetycost" ){
            ROS_INFO("Using Theta* Safety Cost");
            algorithm_.reset(new Planners::ThetaStarM2());
        }else if ( algorithm_name == "thetastaragr" ){
            ROS_INFO("Using Air-Ground Robot Aware Theta*");
            algorithm_.reset(new Planners::ThetaStarAGR());
        }else{
            ROS_WARN("Wrong algorithm name parameter. Using ASTAR by default");
            algorithm_.reset(new Planners::AStar());
        }

        configureHeuristic(_heuristic);

        ROS_INFO("Using discrete world size: [%.2f, %.2f, %.2f]", world_size_.x(), world_size_.y(), world_size_.z());
        ROS_INFO("Using resolution: [%f]", resolution_);

        if(inflate_){
            double inflation_size;
            lnh_.param("inflation_size", inflation_size, 0.5);
            inflation_steps_ = std::round(inflation_size / resolution_);
            ROS_INFO("Inflation size %.2f, using inflation step %d", inflation_size, inflation_steps_);
        }
        algorithm_->setInflationConfig(inflate_, inflation_steps_);

        sdf_map_.reset(new SDFMap);
        sdf_map_->initMap(lnh_);
        edt_environment_.reset(new EDTEnvironment);
        edt_environment_->setMap(sdf_map_);

        algorithm_->setEnvironment(edt_environment_);
        
        std::string frame_id;
        lnh_.param("frame_id", frame_id, std::string("map"));		
        configMarkers(algorithm_name, frame_id, 0.3);

        //Algorithm specific parameters. Its important to set line of sight after configuring world size(it depends on the resolution)
        float cost_weight;
        lnh_.param("cost_weight", cost_weight, (float)0.0);
        algorithm_->setCostFactor(cost_weight);

        lnh_.param("overlay_markers", overlay_markers_, (bool)false);
    }
    void configureHeuristic(const std::string &_heuristic){
        
        if( _heuristic == "euclidean" ){
            algorithm_->setHeuristic(Planners::Heuristic::euclidean);
            ROS_INFO("Using Euclidean Heuristics");
        }else if( _heuristic == "euclidean_optimized" ){
            algorithm_->setHeuristic(Planners::Heuristic::euclideanOptimized);
            ROS_INFO("Using Optimized Euclidean Heuristics");
        }else if( _heuristic == "manhattan" ){
            algorithm_->setHeuristic(Planners::Heuristic::manhattan);
            ROS_INFO("Using Manhattan Heuristics");
        }else if( _heuristic == "octogonal" ){
            algorithm_->setHeuristic(Planners::Heuristic::octagonal);
            ROS_INFO("Using Octogonal Heuristics");
        }else if( _heuristic == "dijkstra" ){
            algorithm_->setHeuristic(Planners::Heuristic::dijkstra);     
            ROS_INFO("Using Dijkstra Heuristics");
        }else{
            algorithm_->setHeuristic(Planners::Heuristic::euclidean);
            ROS_WARN("Wrong Heuristic param. Using Euclidean Heuristics by default");
        }
    }

    void configMarkers(const std::string &_ns, const std::string &_frame, const double &_scale){

        path_line_markers_.ns = _ns;
        path_line_markers_.header.frame_id = _frame;
        path_line_markers_.header.stamp = ros::Time::now();
        path_line_markers_.id = rand();
        path_line_markers_.lifetime = ros::Duration(500);
        path_line_markers_.type = visualization_msgs::Marker::LINE_STRIP;
        path_line_markers_.action = visualization_msgs::Marker::ADD;
        path_line_markers_.pose.orientation.w = 1;

        path_line_markers_.color.r = 0.0;
        path_line_markers_.color.g = 1.0;
        path_line_markers_.color.b = 0.0;

        path_line_markers_.color.a = 1.0;
        path_line_markers_.scale.x = _scale;

        path_points_markers_.ns = _ns;
        path_points_markers_.header.frame_id = _frame;
        path_points_markers_.header.stamp = ros::Time::now();
        path_points_markers_.id = rand();
        path_points_markers_.lifetime = ros::Duration(500);
        path_points_markers_.type = visualization_msgs::Marker::POINTS;
        path_points_markers_.action = visualization_msgs::Marker::ADD;
        path_points_markers_.pose.orientation.w = 1;
        path_points_markers_.color.r = 0.0;
        path_points_markers_.color.g = 1.0;
        path_points_markers_.color.b = 1.0;
        path_points_markers_.color.a = 1.0;
        path_points_markers_.scale.x = _scale;
        path_points_markers_.scale.y = _scale;
        path_points_markers_.scale.z = _scale;

    }
    void publishMarker(visualization_msgs::Marker &_marker, const ros::Publisher &_pub){
        
        //Clear previous marker
        if( !overlay_markers_ ){
            _marker.action = visualization_msgs::Marker::DELETEALL;
            _pub.publish(_marker);
        }else{
            path_points_markers_.id           = rand();
            path_points_markers_.header.stamp = ros::Time::now();
            setRandomColor(path_points_markers_.color);

            path_line_markers_.id             = rand();
            path_line_markers_.header.stamp   = ros::Time::now();
            setRandomColor(path_line_markers_.color);
        }
        _marker.action = visualization_msgs::Marker::ADD;
        _pub.publish(_marker);
    }
    void setRandomColor(std_msgs::ColorRGBA &_color, unsigned int _n_div = 20){
        //Using golden angle approximation
        const double golden_angle = 180 * (3 - sqrt(5));
        double hue = color_id_ * golden_angle + 60;
        color_id_++;
        if(color_id_ == _n_div)
            color_id_ = 1;

        auto random_color = Planners::Misc::HSVtoRGB(hue, 100, 100);

        _color.r = static_cast<int>(random_color.x());
        _color.g = static_cast<int>(random_color.y());
        _color.b = static_cast<int>(random_color.z());
    }


    ros::NodeHandle lnh_{"~"};
    ros::ServiceServer request_path_server_, change_planner_server_;
    ros::Subscriber pointcloud_sub_, occupancy_grid_sub_;
    //TODO Fix point markers
    ros::Publisher line_markers_pub_, point_markers_pub_, global_path_pub_;

    SDFMap::Ptr sdf_map_;

    std::unique_ptr<Planners::AlgorithmBase> algorithm_;
        
    visualization_msgs::Marker path_line_markers_, path_points_markers_;
    
    //Parameters
    Eigen::Vector3d world_size_; // Discrete
    float resolution_;

    bool inflate_{false};
    unsigned int inflation_steps_{0};
    std::string data_folder_;
    bool overlay_markers_{0};
    unsigned int color_id_{0};
    nav_msgs::OccupancyGrid occupancy_grid_;
    pcl::PointCloud<pcl::PointXYZ> cloud_;
    //0: no map yet
    //1: using occupancy
    //2: using cloud
    int input_map_{0};
    std::string heuristic_;

};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "heuristic_planner_ros_node");

    HeuristicPlannerROS heuristic_planner_ros;
    ros::spin();

return 0;
}