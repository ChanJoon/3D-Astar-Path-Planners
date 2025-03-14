 #ifndef __GRID3D_HPP__
#define __GRID3D_HPP__

/**
 * @file prob_map.cpp
 * @brief This file includes the ROS node implementation.
 * @author Francisco J. Perez Grau and Fernando Caballero
 */

#include <sys/time.h>
#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float32.h>
#include <stdio.h> 
// PCL
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>

#include "utils/utils.hpp"
#include "bonxai/probabilistic_map.hpp"

#include <sstream> //for std::ostringstream
#include <fstream>
#include <iomanip> //for std::setw, std::hex, and std::setfill
#include <openssl/evp.h> //for all other OpenSSL function calls
#include <openssl/sha.h> //for SHA512_DIGEST_LENGTH
// #include "utils/ros/ROSInterfaces.hpp"

class Grid3d
{
private:
	
	// Ros parameters
	ros::NodeHandle m_nh;
	bool m_publishPc;
	std::string m_mapPath, m_nodeName;
	std::string m_globalFrameId;
	float m_sensorDev, m_gridSlice;
	double m_publishPointCloudRate, m_publishGridSliceRate;
	
	// Octomap parameters
	float m_maxX, m_maxY, m_maxZ;
	float m_resolution, m_oneDivRes;
	octomap::OcTree *m_octomap;
	
	// 3D probabilistic grid cell
	Planners::utils::gridCell *m_grid;
	int m_gridSize, m_gridSizeX, m_gridSizeY, m_gridSizeZ;
	int m_gridStepY, m_gridStepZ;
	
	// 3D point clound representation of the map
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_cloud;
	pcl::KdTreeFLANN<pcl::PointXYZI> m_kdtree;
	
	// Visualization of the map as pointcloud
	sensor_msgs::PointCloud2 m_pcMsg;
	ros::Publisher m_pcPub;
	ros::Timer mapTimer;
			
	// Visualization of a grid slice as 2D grid map msg
	nav_msgs::OccupancyGrid m_gridSliceMsg;
	ros::Publisher m_gridSlicePub;
	ros::Subscriber global_cloud_sub_;
	ros::Timer gridTimer;

	//Parameters added to allow a new exp function to test different gridmaps
	double cost_scaling_factor, robot_radius;
	bool use_costmap_function;
	bool use_marsim;
	bool global_cloud_received_ = false;

	// Bonxai
	Bonxai::ProbabilisticMap* m_bonxai_map;

	double m_offsetX, m_offsetY, m_offsetZ;
	
public:
	Grid3d(): m_cloud(new pcl::PointCloud<pcl::PointXYZI>)
	{
		// Load paraeters
		double value;
		ros::NodeHandle lnh("~");
		lnh.param("name", m_nodeName, std::string("grid3d"));
		if(!lnh.getParam("global_frame_id", m_globalFrameId))
			m_globalFrameId = "map";	
		if(!lnh.getParam("map_path", m_mapPath))
			m_mapPath = "map.ot";
		if(!lnh.getParam("publish_point_cloud", m_publishPc))
			m_publishPc = true;
		if(!lnh.getParam("publish_point_cloud_rate", m_publishPointCloudRate))
			m_publishPointCloudRate = 0.2;	
		if(!lnh.getParam("publish_grid_slice", value))
			value = 1.0;
		if(!lnh.getParam("publish_grid_slice_rate", m_publishGridSliceRate))
			m_publishGridSliceRate = 0.2;
		m_gridSlice = (float)value;
		if(!lnh.getParam("sensor_dev", value))
			value = 0.2;
		m_sensorDev = (float)value;

		lnh.param("cost_scaling_factor", cost_scaling_factor, 0.8); //0.8		
		lnh.param("robot_radius", robot_radius, 0.4);		//0.4
		lnh.param("use_costmap_function", use_costmap_function, (bool)true);		
		lnh.param("use_marsim", use_marsim, (bool)false);

		// Load octomap 
		m_octomap = NULL;
		m_grid = NULL;
		m_bonxai_map = NULL;

		global_cloud_sub_ = m_nh.subscribe("/global_cloud", 1, &Grid3d::globalCloudCallback, this);

		if(!use_marsim) {
			if(loadMap(m_mapPath))
			{
				// Compute the point-cloud associated to the ocotmap
				computePointCloud();
				computeGrid();
				
				// Build the msg with a slice of the grid if needed
				if(m_gridSlice >= 0 && m_gridSlice <= m_maxZ)
				{
					buildGridSliceMsg(m_gridSlice);
					m_gridSlicePub = m_nh.advertise<nav_msgs::OccupancyGrid>(m_nodeName+"/grid_slice", 1, true);
					gridTimer      = m_nh.createTimer(ros::Duration(1.0/m_publishGridSliceRate), &Grid3d::publishGridSliceTimer, this);	
				}
				
				// Setup point-cloud publisher
				if(m_publishPc)
				{
					m_pcPub  = m_nh.advertise<sensor_msgs::PointCloud2>(m_nodeName+"/map_point_cloud", 1, true);
					mapTimer = m_nh.createTimer(ros::Duration(1.0/m_publishPointCloudRate), &Grid3d::publishMapPointCloudTimer, this);
				}
			}
		}
	}

	~Grid3d(void)
	{
		if(m_octomap != NULL)
			delete m_octomap;
		if(m_grid != NULL)
			delete []m_grid;
	}
	float getGridSizeX() { return m_gridSizeX; }
	float getGridSizeY() { return m_gridSizeY; }
	float getGridSizeZ() { return m_gridSizeZ; }
	float getOffsetX() { return m_offsetX; }
	float getOffsetY() { return m_offsetY; }
	float getOffsetZ() { return m_offsetZ; }
	float getResolution() { return m_resolution; }
	bool setCostParams(const double &_cost_scaling_factor, const double &_robot_radius){

		cost_scaling_factor = std::fabs(_cost_scaling_factor);
		robot_radius 		= std::fabs(_robot_radius);

		return true;
	}
	
	void setGridSliceHeight(const double _height){
		if(_height < m_maxZ && _height > 0.0 ){
			m_gridSlice = _height;
			buildGridSliceMsg(m_gridSlice);
		}
	}
	float computeCloudWeight(std::vector<pcl::PointXYZI> &points)
	{
		float weight = 0.;
		int n = 0;

		for(long unsigned int i=0; i<points.size(); i++)
		{
			const pcl::PointXYZI& p = points[i];
			if(p.x >= 0.0 && p.y >= 0.0 && p.z >= 0.0 && p.x < m_maxX && p.y < m_maxY && p.z < m_maxZ)
			{
				int index = point2grid(p.x, p.y, p.z);
				weight += m_grid[index].prob;
				n++;
			}
		}

		if(n > 10)
			return weight/n;
		else
			return 0;
	}
	
	void publishMapPointCloud(void)
	{
		m_pcMsg.header.stamp = ros::Time::now();
		m_pcPub.publish(m_pcMsg);
	}
	
	void publishGridSlice(void)
	{
		m_gridSliceMsg.header.stamp = ros::Time::now();
		m_gridSlicePub.publish(m_gridSliceMsg);
	}
	
	bool isIntoMap(float x, float y, float z)
	{
		return (x >= 0.0 && y >= 0.0 && z >= 0.0 && x < m_maxX && y < m_maxY && z < m_maxZ);
	}
	// int getCellCost(const float &_x, const float &_y, const float &_z){
	// JAC Precision
	float getCellCost(const float &_x, const float &_y, const float &_z){
		
		if( !isIntoMap(_x, _y, _z) )
			return 0;

		int index = point2grid(_x, _y, _z);
		
		return m_grid[index].prob;
	}
	
	std::pair<Planners::utils::Vec3i, double>  getClosestObstacle(const Planners::utils::Vec3i& _coords){

		pcl::PointXYZI searchPoint;

		searchPoint.x = _coords.x * m_resolution;
		searchPoint.y = _coords.y * m_resolution;
		searchPoint.z = _coords.z * m_resolution;

		int k = 1;
		m_kdtree.setInputCloud(m_cloud);
		std::vector<int> pointIdxNKNSearch(k);
		std::vector<float> pointNKNSquaredDistance(k);

		if(m_kdtree.nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){

			Planners::utils::Vec3i result;

			result.x = std::round(m_cloud->points[pointIdxNKNSearch[0]].x / m_resolution);
			result.y = std::round(m_cloud->points[pointIdxNKNSearch[0]].y / m_resolution);
			result.z = std::round(m_cloud->points[pointIdxNKNSearch[0]].z / m_resolution);

			return std::make_pair(result, std::sqrt(pointNKNSquaredDistance[k-1]));	
		}else{
			return std::make_pair(Planners::utils::Vec3i{}, std::numeric_limits<double>::max());
		}	
	}

protected:

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
	void publishMapPointCloudTimer(const ros::TimerEvent& event)
	{
		publishMapPointCloud();
	}
	
	void publishGridSliceTimer(const ros::TimerEvent& event)
	{
		publishGridSlice();
	}
#pragma GCC diagnostic pop
	void globalCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
	{
		if(use_marsim) {
			ROS_INFO("Received global cloud");

			// Convert ROS PointCloud2 to PCL point cloud
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::fromROSMsg(*msg, *cloud);

			// Downsample if too many points
			if (cloud->points.size() > 100000)
			{
					ROS_INFO("Too many points (%lu), downsampling", cloud->points.size());
					pcl::VoxelGrid<pcl::PointXYZ> sor;
					sor.setInputCloud(cloud);
					sor.setLeafSize(0.1f, 0.1f, 0.1f);
					sor.filter(*cloud);
					ROS_INFO("Downsampled to %lu points", cloud->points.size());
			}

			// Get map parameters
			double minX = std::numeric_limits<double>::max();
			double minY = std::numeric_limits<double>::max();
			double minZ = std::numeric_limits<double>::max();
			double maxX = std::numeric_limits<double>::lowest();
			double maxY = std::numeric_limits<double>::lowest();
			double maxZ = std::numeric_limits<double>::lowest();

			for (const auto& point : cloud->points)
			{
					minX = std::min(minX, (double)point.x);
					minY = std::min(minY, (double)point.y);
					minZ = std::min(minZ, (double)point.z);
					maxX = std::max(maxX, (double)point.x);
					maxY = std::max(maxY, (double)point.y);
					maxZ = std::max(maxZ, (double)point.z);
			}
			
			m_offsetX = minX;
			m_offsetY = minY;
			m_offsetZ = minZ;

			m_maxX = maxX - minX;
			m_maxY = maxY - minY;
			m_maxZ = maxZ - minZ;
			m_resolution = 0.1; // TODO(ChanJoon)
			m_oneDivRes = 1.0 / m_resolution;

			// Create Bonxai map with resolution
			if (m_bonxai_map != NULL)
					delete m_bonxai_map;
			m_bonxai_map = new Bonxai::ProbabilisticMap(m_resolution);

			// Prepare points for Bonxai insertion
			std::vector<Eigen::Vector3d> points;
			points.reserve(cloud->points.size());
			for (const auto& point : cloud->points)
			{
					points.emplace_back(
							point.x - minX,
							point.y - minY,
							point.z - minZ
					);
			}

			// Insert point cloud into Bonxai map
			const Eigen::Vector3d origin(0, 0, 0);
			m_bonxai_map->insertPointCloud(points, origin, std::numeric_limits<double>::infinity());

			ROS_INFO("Map size:\n");
			ROS_INFO("\tx: %.2f to %.2f", minX, maxX);
			ROS_INFO("\ty: %.2f to %.2f", minY, maxY);
			ROS_INFO("\tz: %.2f to %.2f", minZ, maxZ);
			ROS_INFO("\tRes: %.2f", m_resolution);

			// Compute point cloud and grid
			computePointCloud();
			computeGrid();

			// Build the msg with a slice of the grid if needed
			if (m_gridSlice >= 0 && m_gridSlice <= m_maxZ)
			{
					buildGridSliceMsg(m_gridSlice);
					m_gridSlicePub = m_nh.advertise<nav_msgs::OccupancyGrid>(m_nodeName + "/grid_slice", 1, true);
					gridTimer = m_nh.createTimer(ros::Duration(1.0 / m_publishGridSliceRate), &Grid3d::publishGridSliceTimer, this);
			}

			// Setup point-cloud publisher
			if (m_publishPc)
			{
					m_pcPub = m_nh.advertise<sensor_msgs::PointCloud2>(m_nodeName + "/map_point_cloud", 1, true);
					mapTimer = m_nh.createTimer(ros::Duration(1.0 / m_publishPointCloudRate), &Grid3d::publishMapPointCloudTimer, this);
			}

			global_cloud_received_ = true;
			global_cloud_sub_.shutdown(); // Only process once
		}
	}

	bool loadOctomap(std::string &path)
	{
		// release previously loaded data
		if(m_octomap != NULL)
			delete m_octomap;
		if(m_grid != NULL)
			delete []m_grid;
		
		// Load octomap
		octomap::AbstractOcTree *tree;
		if(path.length() > 3 && (path.compare(path.length()-3, 3, ".bt") == 0))
		{
			octomap::OcTree* binaryTree = new octomap::OcTree(0.1);
			if (binaryTree->readBinary(path) && binaryTree->size() > 1)
				tree = binaryTree;
			else 
				return false;
		} 
		else if(path.length() > 3 && (path.compare(path.length()-3, 3, ".ot") == 0))
		{
			tree = octomap::AbstractOcTree::read(path);
			if(!tree)
				return false;
		}
		else if (path.length() > 4 && (path.compare(path.length() - 4, 4, ".pcd") == 0))
		{
			// Load PCL file
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
			if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) == -1)
			{
				std::cerr << "Error: Couldn't read .pcl file " << path << std::endl;
				return false;
			}

			std::cout << "Loaded pc with " << cloud->points.size() << " points" << std::endl;
			if (cloud->points.size() > 100000)
			{
				std::cout << "Too many points, downsampling" << std::endl;
				pcl::VoxelGrid<pcl::PointXYZ> sor;
				sor.setInputCloud(cloud);
				sor.setLeafSize(0.1f, 0.1f, 0.1f);
				sor.filter(*cloud);
				std::cout << "Downsampled to " << cloud->points.size() << " points" << std::endl;
			}
	
			// Convert PCL to OctoMap
			octomap::OcTree *octree = new octomap::OcTree(0.1); // Set resolution
			for (const auto &point : cloud->points)
			{
				octree->updateNode(octomap::point3d(point.x, point.y, point.z), true);
			}
	
			// Perform tree update
			octree->updateInnerOccupancy();
			tree = octree;
		}
		else
			return false;
		
		/*
		// Load octomap
		octomap::AbstractOcTree *tree = octomap::AbstractOcTree::read(path);
		if(!tree)
			return false;*/
		m_octomap = dynamic_cast<octomap::OcTree*>(tree);
		std::cout << "Octomap loaded" << std::endl;

		// Check loading and alloc momery for the grid
		if(m_octomap == NULL)
		{
			std::cout << "Error: NULL octomap!!" << std::endl;
			return false;
		}
		
		// Get map parameters
		double minX, minY, minZ, maxX, maxY, maxZ, res;
		m_octomap->getMetricMin(minX, minY, minZ);
		m_octomap->getMetricMax(maxX, maxY, maxZ);
		res = m_octomap->getResolution();
		m_maxX = (float)(maxX-minX);
		m_maxY = (float)(maxY-minY);
		m_maxZ = (float)(maxZ-minZ);
		m_resolution = (float)res;
		m_oneDivRes = 1.0/m_resolution;
		ROS_INFO("Map size:\n");
		ROS_INFO("\tx: %.2f to %.2f", minX, maxX);
		ROS_INFO("\ty: %.2f to %.2f", minZ, maxZ);
		ROS_INFO("\tz: %.2f to %.2f", minZ, maxZ);
		ROS_INFO("\tRes: %.2f" , m_resolution );
		
		return true;
	}
	bool loadMap(std::string &path)
	{
		std::cout << "Loading map from " << path << std::endl;
		if(m_bonxai_map != NULL)
			delete m_bonxai_map;
		if(m_grid != NULL)
			delete []m_grid;

		if (path.length() > 4 && (path.compare(path.length() - 4, 4, ".pcd") == 0))
		{
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
				if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) == -1)
				{
						std::cerr << "Error: Couldn't read PCD file " << path << std::endl;
						return false;
				}
				std::cout << "Loaded pc with " << cloud->points.size() << " points" << std::endl;
				if (cloud->points.size() > 100000)
				{
					std::cout << "Too many points, downsampling" << std::endl;
					pcl::VoxelGrid<pcl::PointXYZ> sor;
					sor.setInputCloud(cloud);
					sor.setLeafSize(0.1f, 0.1f, 0.1f);
					sor.filter(*cloud);
					std::cout << "Downsampled to " << cloud->points.size() << " points" << std::endl;
				}

				std::vector<Eigen::Vector3d> points;
				points.reserve(cloud->points.size());

				// Get map parameters
				double minX = std::numeric_limits<double>::max();
				double minY = std::numeric_limits<double>::max();
				double minZ = std::numeric_limits<double>::max();
				double maxX = std::numeric_limits<double>::lowest();
				double maxY = std::numeric_limits<double>::lowest();
				double maxZ = std::numeric_limits<double>::lowest();

				for (const auto &point : cloud->points)
				{
					minX = std::min(minX, (double)point.x);
					minY = std::min(minY, (double)point.y);
					minZ = std::min(minZ, (double)point.z);
					maxX = std::max(maxX, (double)point.x);
					maxY = std::max(maxY, (double)point.y);
					maxZ = std::max(maxZ, (double)point.z);
				}
				
				m_maxX = maxX - minX;
				m_maxY = maxY - minY;
				m_maxZ = maxZ - minZ;
				m_resolution = 0.1; // TODO(ChanJoon)
				m_oneDivRes = 1.0/m_resolution;

				// Create Bonxai map with resolution
				m_bonxai_map = new Bonxai::ProbabilisticMap(m_resolution);

				for (const auto &point : cloud->points)
				{
					points.emplace_back(
						point.x - minX,
						point.y - minY,
						point.z - minZ
					);
				}
				// Insert point cloud into Bonxai map
				const Eigen::Vector3d origin(0,0,0);
				m_bonxai_map->insertPointCloud(points, origin, std::numeric_limits<double>::infinity());

				ROS_INFO("Map size:\n");
				ROS_INFO("\tx: %.2f to %.2f", minX, maxX);
				ROS_INFO("\ty: %.2f to %.2f", minY, maxY);
				ROS_INFO("\tz: %.2f to %.2f", minZ, maxZ);
				ROS_INFO("\tRes: %.2f", m_resolution);

				return true;
		}
		return false;
	}
	void computePointCloud(void)
	{
		// Convert Bonxai occupied cells to point cloud
		std::vector<Bonxai::CoordT> occupied_cells;
		m_bonxai_map->getOccupiedVoxels(occupied_cells);

		m_cloud->width = occupied_cells.size();
		m_cloud->height = 1;
		m_cloud->points.resize(m_cloud->width * m_cloud->height);

		#pragma omp parallel for
		for(size_t i = 0; i < occupied_cells.size(); i++)
		{
				auto pos = m_bonxai_map->grid().coordToPos(occupied_cells[i]);
				m_cloud->points[i].x = pos.x;
				m_cloud->points[i].y = pos.y;
				m_cloud->points[i].z = pos.z;
		}

		pcl::toROSMsg(*m_cloud, m_pcMsg);
		m_pcMsg.header.frame_id = m_globalFrameId;
	}
	
	// void computePointCloud(void)
	// {
	// 	// Get map parameters
	// 	double minX, minY, minZ;
	// 	m_octomap->getMetricMin(minX, minY, minZ);
		
	// 	// Load the octomap in PCL for easy nearest neighborhood computation
	// 	// The point-cloud is shifted to have (0,0,0) as min values
	// 	int i = 0;
	// 	m_cloud->width = m_octomap->size();
	// 	m_cloud->height = 1;
	// 	m_cloud->points.resize(static_cast<long>(m_cloud->width) * m_cloud->height);
	// 	for(octomap::OcTree::leaf_iterator it = m_octomap->begin_leafs(), end = m_octomap->end_leafs(); it != end; ++it)
	// 	{
	// 		if(it != NULL && m_octomap->isNodeOccupied(*it))
	// 		{
	// 			m_cloud->points[i].x = it.getX()-minX;
	// 			m_cloud->points[i].y = it.getY()-minY;
	// 			m_cloud->points[i].z = it.getZ()-minZ;
				
	// 			i++;
	// 		}
	// 	}
	// 	m_cloud->width = i;
	// 	m_cloud->points.resize(i);
		
	// 	// Create the point cloud msg for publication
	// 	pcl::toROSMsg(*m_cloud, m_pcMsg);
	// 	m_pcMsg.header.frame_id = m_globalFrameId;
	// }

	void computeGrid(void)
	{
		//Publish percent variable
		// Alloc the 3D grid
		m_gridSizeX = static_cast<int>(m_maxX*m_oneDivRes);
		m_gridSizeY = static_cast<int>(m_maxY*m_oneDivRes); 
		m_gridSizeZ = static_cast<int>(m_maxZ*m_oneDivRes);
		m_gridSize = m_gridSizeX*m_gridSizeY*m_gridSizeZ;
		m_gridStepY = m_gridSizeX;
		m_gridStepZ = m_gridSizeX*m_gridSizeY;
		m_grid = new Planners::utils::gridCell[m_gridSize];

		// Setup kdtree
		m_kdtree.setInputCloud(m_cloud);

		// Compute the distance to the closest point of the grid
		float gaussConst1 = 1./(m_sensorDev*sqrt(2*M_PI));
		float gaussConst2 = 1./(2*m_sensorDev*m_sensorDev);
		double count=0;
		double size = static_cast<double>(m_gridSize);

		pcl::PointXYZI searchPoint;
		std::vector<int> pointIdxNKNSearch(1);
		std::vector<float> pointNKNSquaredDistance(1);
		float dist, prob;
		#pragma omp parallel for collapse(3) private(searchPoint, pointIdxNKNSearch, pointNKNSquaredDistance, dist, prob)
		for(int iz=0; iz<m_gridSizeZ; iz++)
		{
			for(int iy=0; iy<m_gridSizeY; iy++)
			{
				for(int ix=0; ix<m_gridSizeX; ix++)
				{

					searchPoint.x = ix*m_resolution;
					searchPoint.y = iy*m_resolution;
					searchPoint.z = iz*m_resolution;
					int index = ix + iy*m_gridStepY + iz*m_gridStepZ;

					#pragma omp atomic
					count++;
					#pragma omp critical
					{
						double percent = count/size *100.0;
						ROS_INFO_THROTTLE(0.5,"Progress: %lf %%", percent);
					}
					if(m_kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
					{
						dist = pointNKNSquaredDistance[0];
						m_grid[index].dist = dist;
						if(!use_costmap_function){
							m_grid[index].prob = gaussConst1*exp(-dist*dist*gaussConst2);
						}else{
							prob =  100*exp(-cost_scaling_factor*std::fabs((dist - robot_radius)));
							// ROS_INFO("[%f, %f, %f] Dist: %f Probability: %f", searchPoint.x, searchPoint.y, searchPoint.z, dist, prob);
							//JAC: Include the computation of prob considering the distance to the nearest voronoi edge.
							m_grid[index].prob = prob;
						}
					}
					else
					{
						m_grid[index].dist = -1.0;
						m_grid[index].prob =  0.0;
					}
				}
			}
		}
	}

	// void computeGrid(void)
	// {
	// 	// Alloc the 3D grid
	// 	m_gridSizeX = (int)(m_maxX*m_oneDivRes);
	// 	m_gridSizeY = (int)(m_maxY*m_oneDivRes); 
	// 	m_gridSizeZ = (int)(m_maxZ*m_oneDivRes);
	// 	m_gridSize = m_gridSizeX*m_gridSizeY*m_gridSizeZ;
	// 	m_gridStepY = m_gridSizeX;
	// 	m_gridStepZ = m_gridSizeX*m_gridSizeY;
	// 	m_grid = new Planners::utils::gridCell[m_gridSize];

	// 	// Constants for probability calculation
	// 	float gaussConst1 = 1./(m_sensorDev*sqrt(2*M_PI));
	// 	float gaussConst2 = 1./(2*m_sensorDev*m_sensorDev);

	// 	// Progress tracking
	// 	double count = 0;
	// 	const double size = m_gridSizeX*m_gridSizeY*m_gridSizeZ;

	// 	// Get occupied cells from Bonxai
	// 	std::vector<Bonxai::CoordT> occupied_cells;
	// 	m_bonxai_map->getOccupiedVoxels(occupied_cells);

	// 	// Process each grid cell
	// 	for(int iz = 0; iz < m_gridSizeZ; iz++)
	// 	{
	// 		for(int iy = 0; iy < m_gridSizeY; iy++)
	// 		{
	// 			for(int ix = 0; ix < m_gridSizeX; ix++)
	// 			{
	// 				// Calculate current position
	// 				Eigen::Vector3d point(
	// 					ix * m_resolution,
	// 					iy * m_resolution,
	// 					iz * m_resolution
	// 				);

	// 				int index = ix + iy*m_gridStepY + iz*m_gridStepZ;

	// 				// Find closest occupied cell
	// 				// double min_dist = std::numeric_limits<double>::max();
	// 				// for(const auto& occ_coord : occupied_cells) {
	// 				// 	auto pos = m_bonxai_map->grid().coordToPos(occ_coord);
	// 				// 	double dist = (point - Eigen::Vector3d(pos.x, pos.y, pos.z)).norm();
	// 				// 	min_dist = std::min(min_dist, dist);
	// 				// }
	// 				double min_dist = m_bonxai_map->getNearestOccupiedDistance(point);

	// 				// Set grid cell values
	// 				if(min_dist < std::numeric_limits<double>::max())
	// 				{
	// 					m_grid[index].dist = min_dist;
	// 					if(!use_costmap_function) {
	// 							m_grid[index].prob = gaussConst1*exp(-min_dist*min_dist*gaussConst2);
	// 					} else {
	// 							m_grid[index].prob = 100*exp(-cost_scaling_factor*std::fabs((min_dist - robot_radius)));
	// 					}
	// 				}
	// 				else
	// 				{
	// 					m_grid[index].dist = -1.0;
	// 					m_grid[index].prob = 0.0;
	// 				}

	// 				// Update progress
	// 				count++;
	// 				if(fmod(count, 10000) < 1) {
	// 					ROS_INFO_THROTTLE(0.5, "Progress: %.2f %%", count/size * 100.0);
	// 				}
	// 			}
	// 		}
	// 	}
	// }
	
	void buildGridSliceMsg(float z)
	{
		static int seq = 0;
		
		// Setup grid msg
		m_gridSliceMsg.header.frame_id = m_globalFrameId;
		m_gridSliceMsg.header.stamp = ros::Time::now();
		m_gridSliceMsg.header.seq = seq++;
		m_gridSliceMsg.info.map_load_time = ros::Time::now();
		m_gridSliceMsg.info.resolution = m_resolution;
		m_gridSliceMsg.info.width = m_gridSizeX;
		m_gridSliceMsg.info.height = m_gridSizeY;
		m_gridSliceMsg.info.origin.position.x = 0.0;
		m_gridSliceMsg.info.origin.position.y = 0.0;
		m_gridSliceMsg.info.origin.position.z = z;
		m_gridSliceMsg.info.origin.orientation.x = 0.0;
		m_gridSliceMsg.info.origin.orientation.y = 0.0;
		m_gridSliceMsg.info.origin.orientation.z = 0.0;
		m_gridSliceMsg.info.origin.orientation.w = 1.0;
		m_gridSliceMsg.data.resize(static_cast<long>(m_gridSizeX)*m_gridSizeY);

		// Extract max probability
		int offset = (int)(z*m_oneDivRes)*m_gridSizeX*m_gridSizeY;
		int end = offset + m_gridSizeX*m_gridSizeY;
		float maxProb = -1.0;
		for(int i=offset; i<end; i++)
			if(m_grid[i].prob > maxProb)
				maxProb = m_grid[i].prob;

		// Copy data into grid msg and scale the probability to [0,100]
		if(maxProb < 0.000001)
			maxProb = 0.000001;
		maxProb = 100.0/maxProb;
		for(int i=0; i<m_gridSizeX*m_gridSizeY; i++)
			m_gridSliceMsg.data[i] = (int8_t)(m_grid[i+offset].prob*maxProb);
	}
	
	inline int point2grid(const float &x, const float &y, const float &z)
	{
		return (int)(x*m_oneDivRes) + (int)(y*m_oneDivRes)*m_gridStepY + (int)(z*m_oneDivRes)*m_gridStepZ;
	}

};


#endif
