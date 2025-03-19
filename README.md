# 3D A* Path Planners

## Quick-Start

1. **Install ROS [Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) or [Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)**

2. Downlaod Heuristic Path Planners
- Clone the repo, install dependencies with rosdep and compile it with catkin

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone git@github.com:ChanJoon/3D-Astar-Path-Planners.git
cd ..
rosdep update && rosdep install --from-paths src/ -y -r
catkin_make
```

3. Launch a demo a request a path

- 3D Theta*:

In one terminal:
```bash
roslaunch heuristic_planners planner.launch algorithm_name:=thetastar
```

Open another terminal and call an example start and goal request: 
```bash
rosservice call /planner_ros_node/request_path "start:
  x: 0.0
  y: 0.0
  z: 0.0
goal:
  x: 40.0
  y: 40.0
  z: 4.0" 
```

## Test scripts

The package also include pythons scripts to easily automatize test of ROS launch files with ranges of values of some parameters. 

### Dependencies

- Python 3.6 or higher
- matplotlib: ```pip3 install matplotlib```
- numpy: ```pip3 install numpy```
- ROS Melodic/Noetic

Right now there are several scripts 

* test_algorithms.py: Test multiple algorithms with multiple parameters
* test_algorithms_compare.py : Test multiple algorithms with same parameters for comparison purposes and plot them in same plot
* compare_trajectories_rviz.py : Test N algorithms overlaying the trajectories in RVIZ


The general usage is the following:
 
```bash
usage: test_algorithms.py [-h] --launch {planner2d_example.launch,planner.launch}
                          [{planner2d_example.launch,planner.launch} ...] --algorithm
                          {astar,costastar,thetastar,lazythetastar,costlazythetastar}
                          [{astar,costastar,thetastar,lazythetastar,costlazythetastar} ...] --start-coords
                          START_COORDS START_COORDS START_COORDS --goal-coords GOAL_COORDS GOAL_COORDS
                          GOAL_COORDS [--cost-range COST_RANGE COST_RANGE COST_RANGE]
                          [--lof-value LOF_VALUE LOF_VALUE LOF_VALUE]

optional arguments:
  -h, --help            show this help message and exit
  --launch {planner2d_example.launch,planner.launch} [{planner2d_example.launch,planner.launch} ...]
                        name of the launch file
  --map-name            The name of the map with .bt or .pgm extension. This map should be under the resources/3dmaps or resources/2dmaps. 2d/3d option should
                        be configured in the launch
  --algorithm {astar,costastar,thetastar,lazythetastar,costlazythetastar} [{astar,costastar,thetastar,lazythetastar,costlazythetastar} ...] It can be a list 
                        name of the algorithm
  --start-coords START_COORDS START_COORDS START_COORDS
                        start coordinates (x,y,z). Set z to 0 when testing with 2D
  --goal-coords GOAL_COORDS GOAL_COORDS GOAL_COORDS
                        goal coordinates (x,y,z). Set z to 0 when testing with 2D
  --cost-range COST_RANGE COST_RANGE COST_RANGE
                        cost range to evaluate. (min, max, step). To test only one value set COST COST+1 1
  --lof-value LOF_VALUE LOF_VALUE LOF_VALUE
                        Line of sight range to evaluate. (min, max, step). To test only one value set LOF_VALUE
                        LOF_VALUE+1 1
```

```bash
roscd heuristic_planners 
./scripts/test_algorithms.py --launch planner.launch --map-name mbzirc_challenge3.bt --algorithm costastar --start-coords 20.0 20.0 3.0 --goal-coords 40.0 40.0 3.0 --cost-range 1 5 1 --lof-value 1 3 1 --plots total_cost path_length mean_distance_to_obstacle
```

Another example: 

Run the launch ```planner.launch``` with algorithms costlazythetastar and lazythetastarsafetycost with the map mbzirc_challenge3.bt with start coords [20,20,3] and goal coords [40, 40, 3] with cost range between 1 and 20 with steps of 1 and line of sight range between 1 and 5 with an step of 1. 

As it is configured right now, the script will create a multiple graphic visualization of the data with the resulting data for every algorithm with every line of sight i.e. DATA VS COST. They will be saved in the same folder the script is executed. 

```bash
./test_algorithms.py --launch planner.launch --algorithm costlazythetastar lazythetastarsafetycost --map-name mbzirc_challenge3.bt --start-coords 20 20 3 --goal-coords 40 40 3 --cost-range 1 20 1 --lof-value 1 5 1 --plots total_cost1 total_cost2 path_length mean_distance_to_obstacle
```

To plot in the same figure multiple algorithms:

```bash
/scripts/test_algorithms_compare.py --launch planner_example.launch --map-name mbzirc_challenge3.bt --algorithm costastar astarsafetycost --start-coords 20.0 20.0 4.0 --goal-coords 40.0 40.0 3.0 --cost-range 1 5 1 --lof-value 1 3 1 --plots total_cost1 total_cost2 g_cost1 g_cost2 c_cost h_cost
```
### Comparing algorithms in RViz

For that purpose you can use the script ```compare_trajectories_rviz.py``` :

```bash
roscd heuristic_planners/scripts
./compare_trajectories_rviz.py --launch planner.launch --algorithm astar costastar astarsafetycost thetastar costhetastar thetastarsafetycost lazythetastar lazythetastarsafetycost costlazythetastar --map-name mbzirc_challenge3.bt --start-coords 20 20 3 --goal-coords 40 40 3 --cost-value 1 --lof-value 1
```

For the script to finish you should press some key with in the running terminal.

## ACKNOWLEDGEMENT

This work is build upon [robotics-upo/Heuristic_path_planners](https://github.com/robotics-upo/Heuristic_path_planners).