#include <iostream>

#include "Planners/AStar.hpp"
#include "Planners/AStarM2.hpp"
#include "Planners/AStarM1.hpp"
#include "Planners/ThetaStar.hpp"
#include "Planners/ThetaStarM1.hpp"
#include "Planners/ThetaStarM2.hpp"
#include "Planners/ThetaStarAGR.hpp"
#include "Planners/LazyThetaStar.hpp"
#include "Planners/LazyThetaStarM1.hpp"
#include "Planners/LazyThetaStarM1Mod.hpp"
#include "Planners/LazyThetaStarM2.hpp"
#include "utils/ros/ROSInterfaces.hpp"
#include "utils/SaveDataVariantToFile.hpp"
#include "utils/misc.hpp"
#include "utils/geometry_utils.hpp"
#include "utils/metrics.hpp"

// Map
// #include "Grid3D/grid3d.hpp"
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
        
        // pointcloud_sub_     = lnh_.subscribe<pcl::PointCloud<pcl::PointXYZ>>("/points", 1, &HeuristicPlannerROS::pointCloudCallback, this);
        // occupancy_grid_sub_ = lnh_.subscribe<nav_msgs::OccupancyGrid>("/grid", 1, &HeuristicPlannerROS::occupancyGridCallback, this);
        
        request_path_server_   = lnh_.advertiseService("request_path",  &HeuristicPlannerROS::requestPathService, this);
        change_planner_server_ = lnh_.advertiseService("set_algorithm", &HeuristicPlannerROS::setAlgorithm, this);
        
        line_markers_pub_  = lnh_.advertise<visualization_msgs::Marker>("path_line_markers", 1);
        point_markers_pub_ = lnh_.advertise<visualization_msgs::Marker>("path_points_markers", 1);
        global_path_pub_ = lnh_.advertise<nav_msgs::Path>("global_path", 1);
        
    }
    EDTEnvironment::Ptr edt_environment_;

private:

    // void occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr &_grid){
    //     ROS_INFO("Loading OccupancyGrid map...");
    //     Planners::utils::configureWorldFromOccupancyWithCosts(*_grid, *algorithm_);
    //     algorithm_->publishOccupationMarkersMap();
    //     occupancy_grid_sub_.shutdown();
    //     ROS_INFO("Occupancy Grid Loaded");
    //     occupancy_grid_ = *_grid;
    //     input_map_ = 1;
    // }


    // void pointCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &_points)
    // {

    //     ROS_INFO("Loading map...");
    //     Planners::utils::configureWorldFromPointCloud(_points, *algorithm_, resolution_);
    //     algorithm_->publishOccupationMarkersMap();
    //     Planners::utils::configureWorldCosts(*m_grid3d_, *algorithm_);
    //     ROS_INFO("Published occupation marker map");
    //     cloud_ = *_points;
    //     input_map_ = 2;
    //     pointcloud_sub_.shutdown();
    // }   
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
        const auto discrete_goal = Eigen::Vector3i(static_cast<int>(std::round(_req.goal.x / resolution_)),
                                                   static_cast<int>(std::round(_req.goal.y / resolution_)),
                                                   static_cast<int>(std::round(_req.goal.z / resolution_)));
        const auto discrete_start = Eigen::Vector3i(static_cast<int>(std::round(_req.start.x / resolution_)),
                                                    static_cast<int>(std::round(_req.start.y / resolution_)),
                                                    static_cast<int>(std::round(_req.start.z / resolution_)));

        if( algorithm_->detectCollision(discrete_start) ){
            std::cout << discrete_start << ": Start not valid" << std::endl;
            return false;
        }

        if( algorithm_->detectCollision(discrete_goal) ){
            std::cout << discrete_goal << ": Goal not valid" << std::endl;
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

            auto path_data = algorithm_->findPath(discrete_start, discrete_goal);

            if( std::get<bool>(path_data["solved"]) ){
                Planners::utils::CoordinateList path;
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
                    path = std::get<Planners::utils::CoordinateList>(path_data["path"]);

                }catch(std::bad_variant_access const& ex){
                    std::cerr << "Bad variant error: " << ex.what() << std::endl;
                }

                if(save_data_){
                
                    const auto [av_curvature, curv_sigma, curv_min, curv_max] = Planners::utils::metrics::calculatePathCurvature(path);

                    const auto [av_angles, angles_sigma, angles_min, angles_max, changes, angles] = Planners::utils::metrics::calculatePathAnglesMetrics(path, 2);

                    const auto adjacent_path    = Planners::utils::geometry::getAdjacentPath(path, *algorithm_->getInnerWorld());
                    // TODO(Chanjoon) After converting to EDT map and implement getClosestObstaclesToPathPoints, uncomment this save data part
                    // const auto result_distances = getClosestObstaclesToPathPoints(adjacent_path);
                    // const auto [mean_dist, dist_stddev, min_dist, max_dist] = Planners::utils::metrics::calculateDistancesMetrics(result_distances );

                    path_data["av_curv"]        = av_curvature;
                    path_data["std_dev_curv"]   = curv_sigma;
                    path_data["min_curv"]       = curv_min;
                    path_data["max_curv"]       = curv_max;

                    path_data["av_angles"]      = av_angles;
                    path_data["std_dev_angles"] = angles_sigma;
                    path_data["min_angle"]      = angles_min;
                    path_data["max_angle"]      = angles_max;         
                    path_data["angle_changes"]  = changes;

                    // path_data["mean_dist"]      = mean_dist;
                    // path_data["std_dev"]        = dist_stddev;
                    // path_data["min_dist"]       = min_dist;
                    // path_data["max_dist"]       = max_dist;

                    _rep.n_points.data                   = adjacent_path.size();
                    // _rep.mean_distance_to_obstacle.data  = mean_dist;
                    // _rep.mean_std_dev_to_obstacle.data   = dist_stddev;
                    // _rep.min_distance_to_obstacle.data   = min_dist;
                    // _rep.max_distance_to_obstacle.data   = max_dist;

                    // Planners::utils::DataVariantSaver saver;

                    // if(saver.savePathDataToFile(path_data, data_folder_ + "/planning.txt") && 
                    //    saver.savePathDistancesToFile(adjacent_path, result_distances, data_folder_ + "/path_metrics.txt") &&
                    //    saver.saveAnglesToFile(angles, data_folder_ + "/angles.txt") ){
                    //     ROS_INFO("Path data metrics saved");
                    // }else{
                    //     ROS_ERROR("Couldn't save path data metrics. Path and results does not have same size");
                    // }
                }

                if(_req.tries.data < 2 || i == ( _req.tries.data - 1) ){

                    for(const auto &it: std::get<Planners::utils::CoordinateList>(path_data["path"])){
                        geometry_msgs::PoseStamped pose;
                        pose.header = global_path.header;
                        pose.pose.position = Planners::utils::continousPoint(it, resolution_);
                        pose.pose.orientation.w = 1;
                        global_path.poses.push_back(pose);

                        path_line_markers_.points.push_back(Planners::utils::continousPoint(it, resolution_));
                        path_points_markers_.points.push_back(Planners::utils::continousPoint(it, resolution_));
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
        
        lnh_.param("use3d", use3d_, (bool)true);

        // ThetaStarAGR-specific parameters
        float ground_height_threshold = 5.0f;
        int ground_to_air_transition_cost = 0;
        float air_movement_factor = 2;
        int flying_cost = 10;

        if (algorithm_name == "thetastaragr") {
            lnh_.param("ground_height_threshold", ground_height_threshold, (float)5.0);
            lnh_.param("ground_to_air_transition_cost", ground_to_air_transition_cost, (int)0);
            lnh_.param("air_movement_factor", air_movement_factor, (float)2.0);
            lnh_.param("flying_cost", flying_cost, (int)10);
        }

        if( algorithm_name == "astar" ){
            ROS_INFO("Using A*");
            algorithm_.reset(new Planners::AStar(use3d_));
        }else if( algorithm_name == "costastar" ){
            ROS_INFO("Using Cost Aware A*");
            algorithm_.reset(new Planners::AStarM1(use3d_));
        }else if( algorithm_name == "astarsafetycost" ){
            ROS_INFO("Using A* Safety Cost");
            algorithm_.reset(new Planners::AStarM2(use3d_));    
        }else if ( algorithm_name == "thetastar" ){
            ROS_INFO("Using Theta*");
            algorithm_.reset(new Planners::ThetaStar(use3d_));
        }else if ( algorithm_name == "costhetastar" ){
            ROS_INFO("Using Cost Aware Theta* ");
            algorithm_.reset(new Planners::ThetaStarM1(use3d_));
        }else if ( algorithm_name == "thetastarsafetycost" ){
            ROS_INFO("Using Theta* Safety Cost");
            algorithm_.reset(new Planners::ThetaStarM2(use3d_));
        }else if ( algorithm_name == "thetastaragr" ){
            ROS_INFO("Using Air-Ground Robot Aware Theta*");
            algorithm_.reset(new Planners::ThetaStarAGR(use3d_, ground_height_threshold, ground_to_air_transition_cost, air_movement_factor, flying_cost));
        }else if( algorithm_name == "lazythetastar" ){
            ROS_INFO("Using LazyTheta*");
            algorithm_.reset(new Planners::LazyThetaStar(use3d_));
        }else if( algorithm_name == "costlazythetastar"){
            ROS_INFO("Using Cost Aware LazyTheta*");
            algorithm_.reset(new Planners::LazyThetaStarM1(use3d_));
        }else if( algorithm_name == "costlazythetastarmodified"){
            ROS_INFO("Using Cost Aware LazyTheta*");
            algorithm_.reset(new Planners::LazyThetaStarM1Mod(use3d_));
        }else if( algorithm_name == "lazythetastarsafetycost"){
            ROS_INFO("Using LazyTheta* Safety Cost");
            algorithm_.reset(new Planners::LazyThetaStarM2(use3d_));
        }else{
            ROS_WARN("Wrong algorithm name parameter. Using ASTAR by default");
            algorithm_.reset(new Planners::AStar(use3d_));
        }

        // algorithm_->setWorldSize(world_size_, resolution_);

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

        // m_grid3d_.reset(new Grid3d); //TODO Costs not implement yet
        // double cost_scaling_factor, robot_radius;
        // lnh_.param("cost_scaling_factor", cost_scaling_factor, 0.8);		
		// lnh_.param("robot_radius", robot_radius, 0.4);		
        
        // m_grid3d_->setCostParams(cost_scaling_factor, robot_radius);

        sdf_map_.reset(new SDFMap);
        sdf_map_->initMap(lnh_);
        edt_environment_.reset(new EDTEnvironment);
        edt_environment_->setMap(sdf_map_);

        algorithm_->setEnvironment(edt_environment_);
        
        std::string frame_id;
        lnh_.param("frame_id", frame_id, std::string("map"));		
        configMarkers(algorithm_name, frame_id, 0.3);

        lnh_.param("save_data_file", save_data_, (bool)true);		
        lnh_.param("data_folder", data_folder_, std::string("planing_data.txt"));		
        if(save_data_)
            ROS_INFO_STREAM("Saving path planning data results to " << data_folder_);

        //Algorithm specific parameters. Its important to set line of sight after configuring world size(it depends on the resolution)
        float sight_dist, cost_weight;
        lnh_.param("max_line_of_sight_distance", sight_dist, (float)1000.0); // In meters
        lnh_.param("cost_weight", cost_weight, (float)0.0);
        algorithm_->setMaxLineOfSight(sight_dist);
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
    //TODO(Chanjoon) Implement this function for EDT map
    // std::vector<std::pair<Eigen::Vector3i, double>> getClosestObstaclesToPathPoints(const Planners::utils::CoordinateList &_path){
        
    //     std::vector<std::pair<Eigen::Vector3i, double>> result;
    //     if ( use3d_ ){
    //         //TODO grid3d distances does not take into account the inflation added internally by the algorithm

    //         for(const auto &it: _path)
    //             result.push_back( m_grid3d_->getClosestObstacle(it) );
    //         }

    //     else{//TODO IMplement for 2d
    //         result.push_back(std::make_pair<Eigen::Vector3i, double>(Eigen::Vector3i{0,0,0}, 0.0));
    //     }
    //     return result;
    // }
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

    std::unique_ptr<Grid3d> m_grid3d_;
    SDFMap::Ptr sdf_map_;

    std::unique_ptr<Planners::AlgorithmBase> algorithm_;
        
    visualization_msgs::Marker path_line_markers_, path_points_markers_;
    
    //Parameters
    Eigen::Vector3d world_size_; // Discrete
    float resolution_;

    bool save_data_;
    bool use3d_{true};

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