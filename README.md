# 3D A* Path Planners


> ⚠️ This project is currently **UNDER DEVELOPMENT**

## Quick-Start

1. **Install ROS [Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) or [Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)**

2. **Downlaod Heuristic Path Planners**
  Clone the repo, install dependencies with rosdep and compile it with catkin

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Clone the UAV simulator and this planner
git clone git@github.com:ChanJoon/uav_simulator.git
git clone git@github.com:ChanJoon/3D-Astar-Path-Planners.git

cd ..
rosdep update && rosdep install --from-paths src/ -y -r
catkin_make # catkin build
```

3. **Launch a demo a request a path**
- 3D A*:

```bash
roslaunch heuristic_planners planner.launch algorithm_name:=astar
```
- 3D Theta*:

```bash
roslaunch heuristic_planners planner.launch algorithm_name:=thetastar
```
- 3D LazyTheta*:

```bash
roslaunch heuristic_planners planner.launch algorithm_name:=lazythetastar
```

and send a goal point using RViz 2D Nav Goal.

### ✨ Key Differences from Original Version
This project is based on the original [Heuristic Path Planners](https://github.com/robotics-upo/Heuristic_path_planners) but introduces important improvements:

✅ Supports 3D grid maps, not just preprocessed static maps (checkout their [.bt files](https://github.com/robotics-upo/Heuristic_path_planners/tree/master/resources/3dmaps))

✅ Works directly with live PointCloud2 topics

✅ Includes updated simulation support via uav_simulator

---

## ACKNOWLEDGEMENT

This work is build upon [robotics-upo/Heuristic_path_planners](https://github.com/robotics-upo/Heuristic_path_planners).