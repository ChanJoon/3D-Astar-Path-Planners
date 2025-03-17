#include "Planners/AStar.hpp"

namespace Planners{
    
AStar::AStar(bool _use_3d = true, std::string _name = "astar" ): AlgorithmBase(_use_3d, _name){
    configAlgorithm();
}
    
AStar::AStar(bool _use_3d = true): AlgorithmBase(_use_3d, "astar")
{
    configAlgorithm();
}
void AStar::configAlgorithm(){

    closedSet_.reserve(50000);
    openSet_.reserve(50000);
    //If compiled with ros and visualization
    explored_nodes_marker_pub_ = lnh_.advertise<visualization_msgs::Marker>("explored_nodes",   1);
    openset_marker_pub_        = lnh_.advertise<visualization_msgs::Marker>("openset_nodes",    1);
    closedset_marker_pub_      = lnh_.advertise<visualization_msgs::Marker>("closed_set_nodes", 1);
    best_node_marker_pub_      = lnh_.advertise<visualization_msgs::Marker>("best_node_marker", 1);
    aux_text_marker_pub_       = lnh_.advertise<visualization_msgs::Marker>("aux_text_marker",  1);
	occupancy_marker_pub_ = lnh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("occupancy_markers", 1, true);

    std::string frame_id;
    lnh_.param("frame_id", frame_id, std::string("map"));	
    lnh_.param("resolution", resolution_, (float)0.2);
	occupancy_marker_.header.frame_id = frame_id; // "world";

    explored_node_marker_.header.frame_id = frame_id; //"world";
	explored_node_marker_.header.stamp = ros::Time();
	explored_node_marker_.ns = "debug";
	explored_node_marker_.id = 66;
	explored_node_marker_.type = visualization_msgs::Marker::CUBE_LIST;
	explored_node_marker_.action = visualization_msgs::Marker::ADD;
	explored_node_marker_.pose.orientation.w = 1.0;
	explored_node_marker_.scale.x = 1.0 * resolution_;
	explored_node_marker_.scale.y = 1.0 * resolution_;
	explored_node_marker_.scale.z = 1.0 * resolution_;
	explored_node_marker_.color.a = 0.7;
	explored_node_marker_.color.r = 0.0;
	explored_node_marker_.color.g = 1.0;
	explored_node_marker_.color.b = 0.0;

    openset_markers_    = explored_node_marker_;
    openset_markers_.color.b = 1.0;
    openset_markers_.color.g = 0.0;
	openset_markers_.id      = 67;

    closed_set_markers_ = explored_node_marker_;
    closed_set_markers_.color.g = 0.0;
    closed_set_markers_.color.r = 1.0;
	explored_node_marker_.id = 68;

    best_node_marker_ = explored_node_marker_;
    best_node_marker_.color.g = 0.7;
    best_node_marker_.color.b = 0.7;
    best_node_marker_.id     = 69;
	best_node_marker_.type = visualization_msgs::Marker::SPHERE;

    aux_text_marker_   = explored_node_marker_;
	aux_text_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	aux_text_marker_.id = 70;
	aux_text_marker_.color.a = 0.7;
	aux_text_marker_.color.g = 0.0;
    aux_text_marker_.text = "";
	aux_text_marker_.scale.z = 3.0 * resolution_;
    last_publish_tamp_ = ros::Time::now();

}
void AStar::publishOccupationMarkersMap()
{
	occupancy_marker_.clear();
    for(const auto &it: discrete_world_.getElements()){
        if(!it.occupied) continue;
        pcl::PointXYZ point;

		point.x = it.coordinates.x() * resolution_;
		point.y = it.coordinates.y() * resolution_;
		point.z = it.coordinates.z() * resolution_;
		occupancy_marker_.push_back(point);
    }

	occupancy_marker_pub_.publish(occupancy_marker_);
}

inline unsigned int AStar::computeG(const Node* _current, Node* _suc,  unsigned int _n_i, unsigned int _dirs){
    unsigned int cost = _current->G;

    if(_dirs  == 8){
        cost += (_n_i < 4 ? dist_scale_factor_ : dd_2D_); //This is more efficient
    }else{
        cost += (_n_i < 6 ? dist_scale_factor_ : (_n_i < 18 ? dd_2D_ : dd_3D_)); //This is more efficient
    }
    
    _suc->C = _suc->cost;
    
    return cost;
}

#pragma GCC diagnostic pop

void AStar::exploreNeighbours(Node* _current, const Eigen::Vector3d &_target, node_by_position &_index_by_pos){
    
    for (unsigned int i = 0; i < direction.size(); ++i) {
            
        Eigen::Vector3d newCoordinates = _current->coordinates + direction[i];
        Node *successor = discrete_world_.getNodePtr(newCoordinates);
        //Skip the neighbour if it is not valid, occupied, or already in teh
        //closed list
        if ( successor == nullptr ||
             successor->isInClosedList || 
             successor->occupied ) 
            continue;
 
        unsigned int totalCost = computeG(_current, successor, i, direction.size());
            
        if ( !successor->isInOpenList ) { 
            successor->parent = _current;
            successor->G = totalCost;
            successor->H = heuristic(successor->coordinates, _target);
            successor->gplush = successor->G + successor->H;
            successor->isInOpenList = true;
            _index_by_pos.insert(successor);
        }
        else if (totalCost < successor->G) {
            successor->parent = _current;
            successor->G = totalCost;
            successor->gplush = successor->G + successor->H;
            auto found = _index_by_pos.find(successor->world_index);
            _index_by_pos.erase(found);
            _index_by_pos.insert(successor);
        }
    }
}
PathData AStar::findPath(const Eigen::Vector3d &_source, const Eigen::Vector3d &_target)
{
    Node *current = nullptr;

    bool solved{false};

    discrete_world_.getNodePtr(_source)->parent = new Node(_source);
    discrete_world_.setOpenValue(_source, true);
    //Timer to record the execution time, not 
    //really important
    utils::Clock main_timer;
    main_timer.tic();

    line_of_sight_checks_ = 0;
    
    node_by_cost&     indexByCost          = openSet_.get<IndexByCost>();
    node_by_position& indexByWorldPosition = openSet_.get<IndexByWorldPosition>();

    indexByCost.insert(discrete_world_.getNodePtr(_source));
    
    while (!indexByCost.empty()) {
        //Get the element at the start of the open set ordered by cost
        auto it = indexByCost.begin();
        current = *it;
        indexByCost.erase(indexByCost.begin());
        
        if (current->coordinates == _target) { solved = true; break; }
        
        closedSet_.push_back(current);
        //This flags are used to avoid search in the containers, 
        //for speed reasons.
        current->isInOpenList = false;
        current->isInClosedList = true;

        exploreNeighbours(current, _target, indexByWorldPosition);     
    }
    main_timer.toc();
    
    PathData result_data = createResultDataObject(current, main_timer, closedSet_.size(), 
                                                 solved, _source, line_of_sight_checks_);
    closedSet_.clear();
    openSet_.clear();
    delete discrete_world_.getNodePtr(_source)->parent;
    
    discrete_world_.resetWorld();
    return result_data;
}

}
