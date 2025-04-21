#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "Planners/AStar.hpp"
#include "Planners/ThetaStar.hpp"
#include "Planners/ThetaStarAGR.hpp"
#include "Planners/ThetaStarAGRpp.hpp"
#include "utils/misc.hpp"
#include "utils/geometry_utils.hpp"

#include "heuristic_planners/srv/get_path.hpp"
#include "heuristic_planners/srv/set_algorithm.hpp"


class HeuristicPlannerROS : public rclcpp::Node
{

public:
    HeuristicPlannerROS() : Node("heuristic_planner_ros_node")
    {
        this->declare_parameter("algorithm", "astar");
        this->declare_parameter("heuristic", "euclidean");
        this->declare_parameter("resolution", 0.2f);
        this->declare_parameter("inflate_map", true);
        this->declare_parameter("frame_id", "map");
        this->declare_parameter("marker_scale", 0.1);
        this->declare_parameter("overlay_markers", false);
        this->declare_parameter("ground_height", 0.0);
    }

    void init() {
        std::string algorithm_name;
        this->get_parameter("algorithm", algorithm_name);
        this->get_parameter("heuristic", heuristic_);
        this->get_parameter("resolution", resolution_);
        this->get_parameter("inflate_map", inflate_);
        this->get_parameter("frame_id", frame_id);
        this->get_parameter("marker_scale", marker_scale);
        this->get_parameter("overlay_markers", overlay_markers_);
        this->get_parameter("ground_height", ground_height_);
        
        configureAlgorithm(algorithm_name, heuristic_);

        request_path_server_   = this->create_service<heuristic_planners::srv::GetPath>(
            "request_path",  std::bind(&HeuristicPlannerROS::requestPathService, this, std::placeholders::_1, std::placeholders::_2));
        change_planner_server_ = this->create_service<heuristic_planners::srv::SetAlgorithm>(
            "set_algorithm", std::bind(&HeuristicPlannerROS::setAlgorithm, this, std::placeholders::_1, std::placeholders::_2));

        rviz_trigger_callback_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal", rclcpp::QoS(1).best_effort(), std::bind(&HeuristicPlannerROS::rvizTriggerCallback, this, std::placeholders::_1));
        
        line_markers_pub_  = this->create_publisher<visualization_msgs::msg::Marker>("path_line_markers", 1);
        point_markers_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("path_points_markers", 1);
        global_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("global_path", 1);
    }

private:

    bool setAlgorithm(const std::shared_ptr<heuristic_planners::srv::SetAlgorithm::Request> _req,
        std::shared_ptr<heuristic_planners::srv::SetAlgorithm::Response> _rep) {
        
        configureAlgorithm(_req->algorithm.data, _req->heuristic.data);
        _rep->result.data = true;
        return true;
    }
    void rvizTriggerCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        algorithm_->reset();
        double init_x_, init_y_, init_z_;
        this->declare_parameter("init_x", 0.0);
        this->declare_parameter("init_y", 0.0);
        this->declare_parameter("init_z", 0.0);
        this->get_parameter("init_x", init_x_);
        this->get_parameter("init_y", init_y_);
        this->get_parameter("init_z", init_z_);
        
        Eigen::Vector3d start_pos(init_x_, init_y_, init_z_);
        Eigen::Vector3d goal_pos(msg->pose.position.x, msg->pose.position.y, ground_height_+0.01);
        
        Eigen::Vector3d origin, size;
        grid_map_->getRegion(origin, size);
        auto isWithinBounds = [&origin, &size](const Eigen::Vector3d &pos) {
            return (pos.x() >= origin.x() && pos.y() >= origin.y() && pos.z() >= origin.z() &&
               pos.x() <= size.x() && pos.y() <= size.y() && pos.z() <= size.z());
        };
        
        if (algorithm_->detectCollision(start_pos) || !isWithinBounds(start_pos)) {
            std::cout << start_pos << ": Start not valid" << std::endl;
            return;
        }
        if (algorithm_->detectCollision(goal_pos) || !isWithinBounds(goal_pos)) {
            std::cout << goal_pos << ": Goal not valid" << std::endl;
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Planning path from [%f, %f, %f] to [%f, %f, %f]", 
             start_pos.x(), start_pos.y(), start_pos.z(),
             goal_pos.x(), goal_pos.y(), goal_pos.z());

        auto path_data = algorithm_->findPath(start_pos, goal_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
        
        if (std::get<bool>(path_data["solved"])) {
            std::vector<Eigen::Vector3d> path = algorithm_->getPath();
            algorithm_->reset();
            
            // Publish path as markers
            path_line_markers_.points.clear();
            path_points_markers_.points.clear();
            
            nav_msgs::msg::Path global_path;
            global_path.header.frame_id = "world";
            global_path.header.stamp = this->get_clock()->now();
            
            for (const auto &it: path) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = global_path.header;
                pose.pose.position.x = it.x();
                pose.pose.position.y = it.y();
                pose.pose.position.z = it.z();
                pose.pose.orientation.w = 1.0;
                global_path.poses.push_back(pose);
                
                geometry_msgs::msg::Point point;
                point.x = it.x();
                point.y = it.y();
                point.z = it.z();
                path_line_markers_.points.push_back(point);
                path_points_markers_.points.push_back(point);
            }
            
            publishMarker(path_line_markers_, line_markers_pub_);
            publishMarker(path_points_markers_, point_markers_pub_);
            global_path_pub_->publish(global_path);
            
            RCLCPP_INFO(this->get_logger(), "Path published successfully");
        } else {
            RCLCPP_WARN(this->get_logger(), "Could not find a path between the specified points");
        }
    }
    
    bool requestPathService(
        const std::shared_ptr<heuristic_planners::srv::GetPath::Request> _req,
        std::shared_ptr<heuristic_planners::srv::GetPath::Response> _rep) {
        algorithm_->reset();

        if( !_req->algorithm.data.empty() ){
            if( !_req->heuristic.data.empty() ){
                configureAlgorithm(_req->algorithm.data, _req->heuristic.data);
            }else{
                configureAlgorithm(_req->algorithm.data, heuristic_);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Path requested, computing path");
        //delete previous markers
        publishMarker(path_line_markers_, line_markers_pub_);
        publishMarker(path_points_markers_, point_markers_pub_);

        //Astar coordinate list is std::vector<Eigen::Vector3i>
        auto start_pos = Eigen::Vector3d((_req->start.x), (_req->start.y), (_req->start.z));
        auto goal_pos = Eigen::Vector3d((_req->goal.x), (_req->goal.y), (_req->goal.z));

        if( algorithm_->detectCollision(start_pos) ){
            std::cout << start_pos << ": Start not valid" << std::endl;
            return false;
        }

        if( algorithm_->detectCollision(goal_pos) ){
            std::cout << goal_pos << ": Goal not valid" << std::endl;
            return false;
        }

        Eigen::Vector3d origin, size;
        grid_map_->getRegion(origin, size);
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
        times.reserve(_req->tries.data);
        int real_tries = _req->tries.data;
        if(real_tries == 0) real_tries = 1;

        nav_msgs::msg::Path global_path;
        global_path.header.frame_id = "world";
        global_path.header.stamp = this->get_clock()->now();

        for(int i = 0; i < real_tries; ++i){
            auto path_data = algorithm_->findPath(start_pos, goal_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

            if( std::get<bool>(path_data["solved"]) ){
                std::vector<Eigen::Vector3d> path;
                try{
                    _rep->time_spent.data           = std::get<double>(path_data["time_spent"] );
                    _rep->time_spent.data /= 1000;
                    times.push_back(_rep->time_spent.data);

                    if(_req->tries.data < 2 || i == ( _req->tries.data - 1) ){
                        _rep->path_length.data          = std::get<double>(path_data["path_length"] );
                        _rep->explored_nodes.data       = std::get<size_t>(path_data["explored_nodes"] );
                        _rep->line_of_sight_checks.data = std::get<unsigned int>(path_data["line_of_sight_checks"] );

                        _rep->total_cost1.data           = std::get<unsigned int>(path_data["total_cost1"] );
                        _rep->total_cost2.data           = std::get<unsigned int>(path_data["total_cost2"] );
                        _rep->h_cost.data               = std::get<unsigned int>(path_data["h_cost"]);
                        _rep->g_cost1.data               = std::get<unsigned int>(path_data["g_cost1"]);
                        _rep->g_cost2.data               = std::get<unsigned int>(path_data["g_cost2"]);
                        _rep->c_cost.data               = std::get<unsigned int>(path_data["c_cost"]);
                    }
                    path = algorithm_->getPath();
                    algorithm_->reset();


                }catch(std::bad_variant_access const& ex){
                    std::cerr << "Bad variant error: " << ex.what() << std::endl;
                }

                if(_req->tries.data < 2 || i == ( _req->tries.data - 1) ){

                    for(const auto &it: path){
                        geometry_msgs::msg::PoseStamped pose;
                        pose.header = global_path.header;
                        pose.pose.position.x = it.x();
                        pose.pose.position.y = it.y();
                        pose.pose.position.z = it.z();
                        pose.pose.orientation.w = 1.0;
                        global_path.poses.push_back(pose);

                        geometry_msgs::msg::Point point;
                        point.x = it.x();
                        point.y = it.y();
                        point.z = it.z();
                        path_line_markers_.points.push_back(point);
                        path_points_markers_.points.push_back(point);
                    }

                    publishMarker(path_line_markers_, line_markers_pub_);
                    publishMarker(path_points_markers_, point_markers_pub_);
                    global_path_pub_->publish(global_path);

                    path_line_markers_.points.clear();
                    path_points_markers_.points.clear();
                    global_path.poses.clear();

                    RCLCPP_INFO(this->get_logger(), "Path calculated succesfully");
                }
            }else{
                RCLCPP_INFO(this->get_logger(), "Could not calculate path between request points");
            }
        }  
        if(_req->tries.data > 2){
            auto av_time = std::accumulate(times.begin(), times.end(), 0.0) / times.size(); 
            std::cout << "Average Time: "      << av_time  << " milisecs" << std::endl;
            std::cout << "Average Frequency: " << 1000/av_time << std::endl;
        }
        return true;
    }
    void configureAlgorithm(const std::string &algorithm_name, const std::string &_heuristic){


        if( algorithm_name == "astar" ){
            RCLCPP_INFO(this->get_logger(), "Using A*");
            algorithm_ = std::make_unique<Planners::AStar>(shared_from_this());
            algorithm_->setParam();
        }else if ( algorithm_name == "thetastar" ){
            RCLCPP_INFO(this->get_logger(), "Using Theta*");
            algorithm_ = std::make_unique<Planners::ThetaStar>(shared_from_this());
            algorithm_->setParam();
        }else if ( algorithm_name == "thetastaragr" ){
            RCLCPP_INFO(this->get_logger(), "Using Air-Ground Robot Aware Theta*");
            algorithm_ = std::make_unique<Planners::ThetaStarAGR>(shared_from_this());
            algorithm_->setParam();
        } else if ( algorithm_name == "thetastaragrpp" ) {
            RCLCPP_INFO(this->get_logger(), "Using MH Theta*");
            algorithm_ = std::make_unique<Planners::ThetaStarAGRpp>(shared_from_this());
            algorithm_->setParam();
        } else{
            RCLCPP_WARN(this->get_logger(), "Wrong algorithm name parameter. Using A* by default");
            algorithm_ = std::make_unique<Planners::AStar>(shared_from_this());
            algorithm_->setParam();
        }

        grid_map_.reset(new GridMap);
        grid_map_->initMap(shared_from_this());
        algorithm_->setGridMap(grid_map_);

        algorithm_->init();
    
        configMarkers(algorithm_name, frame_id, marker_scale);

    }

    void configMarkers(const std::string &_ns, const std::string &_frame, const double &_scale){

        path_line_markers_.ns = _ns;
        path_line_markers_.header.frame_id = _frame;
        path_line_markers_.header.stamp = this->get_clock()->now();
        path_line_markers_.id = rand();
        path_line_markers_.lifetime = rclcpp::Duration::from_seconds(500);
        path_line_markers_.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_line_markers_.action = visualization_msgs::msg::Marker::ADD;
        path_line_markers_.pose.orientation.w = 1;

        path_line_markers_.color.r = 0.0;
        path_line_markers_.color.g = 1.0;
        path_line_markers_.color.b = 0.0;

        path_line_markers_.color.a = 1.0;
        path_line_markers_.scale.x = _scale;

        path_points_markers_.ns = _ns;
        path_points_markers_.header.frame_id = _frame;
        path_points_markers_.header.stamp = this->get_clock()->now();
        path_points_markers_.id = rand();
        path_points_markers_.lifetime = rclcpp::Duration::from_seconds(500);
        path_points_markers_.type = visualization_msgs::msg::Marker::POINTS;
        path_points_markers_.action = visualization_msgs::msg::Marker::ADD;
        path_points_markers_.pose.orientation.w = 1;
        path_points_markers_.color.r = 0.0;
        path_points_markers_.color.g = 1.0;
        path_points_markers_.color.b = 1.0;
        path_points_markers_.color.a = 1.0;
        path_points_markers_.scale.x = _scale;
        path_points_markers_.scale.y = _scale;
        path_points_markers_.scale.z = _scale;

    }
    void publishMarker(visualization_msgs::msg::Marker &_marker, const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &_pub){
        
        //Clear previous marker
        if( !overlay_markers_ ){
            _marker.action = visualization_msgs::msg::Marker::DELETEALL;
            _pub->publish(_marker);
        }else{
            path_points_markers_.id           = rand();
            path_points_markers_.header.stamp = this->get_clock()->now();
            setRandomColor(path_points_markers_.color);

            path_line_markers_.id             = rand();
            path_line_markers_.header.stamp   = this->get_clock()->now();
            setRandomColor(path_line_markers_.color);
        }
        _marker.action = visualization_msgs::msg::Marker::ADD;
        _pub->publish(_marker);
    }
    void setRandomColor(std_msgs::msg::ColorRGBA &_color, unsigned int _n_div = 20){
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


    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> line_markers_pub_;
    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> point_markers_pub_;
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> global_path_pub_;
  
    std::shared_ptr<rclcpp::Service<heuristic_planners::srv::GetPath>> request_path_server_;
    std::shared_ptr<rclcpp::Service<heuristic_planners::srv::SetAlgorithm>> change_planner_server_;
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>> rviz_trigger_callback_;

    GridMap::Ptr grid_map_;

    std::unique_ptr<Planners::AlgorithmBase> algorithm_;
        
    visualization_msgs::msg::Marker path_line_markers_, path_points_markers_;
    
    //Parameters
    float resolution_;
    std::string frame_id;
    double marker_scale;

    bool inflate_{false};
    bool overlay_markers_{0};
    unsigned int color_id_{0};
    std::string heuristic_;
    double ground_height_;

};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<HeuristicPlannerROS>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}