<img src="Pics/Path Planning Pic.png" align="center" width="700"/>


# Robotics Systems Science, Path Planing Algorithms

# Introduction
This project aims to study the performance of four path planning and obstacle avoidance methods within various closed environments. The comparison is performed against success rate, accuracy and cost effectiveness of the planned path. Four methods are studied:
   * [Dijkstra Shortest Path Algorithm](#Dijkstra Shortest Path Algorithm)
   * [A* Algorithm](#A* Algorithm)
   * [Probabilistic Road Map Algorithm](#Probabilistic Road Map Algorithm)
   * [Rapidly-exploring Random Tree](#Rapidly-exploring Random Tree)
   * [Dynamic Window Approach](#Dynamic Window Approach)

The ["PythonRobotics"](https://github.com/AtsushiSakai/PythonRobotics) project from - [AtsushiSakai](https://github.com/AtsushiSakai/) ([@Atsushi_twi](https://twitter.com/Atsushi_twi)) has been of great help in developing of path planning algorithms.

# Results
<img src="Pics/Final Result.jpg" align="center" width="700"/>

Dijkstra returning the shortest possible path and A* slightly longer; whereas,
sample-based algorithms such as RRT and PRM return longer paths. RRT converges faster by increasing the rate at which the goal is sampled; however, this would
run the risk of getting stuck in a corner or a turn where the goal is just behind the obstacle, similar to the U-shaped maze. Compared to RRT, PRM was able to find a more economic path.

<img src="Pics/Path Planning Pic.png" align="center" width="700"/>

Dynamic Window Approach (DWA) was also implanted capable of navigating collision free avoiding obstacles not existing on the initial occupancy grid.

# Documentation

This read-me discuss the result of course project on path planning algorithms. The codes are stored in separate folders.The top view 2D map is fed to the algorithms as a black and white picture (.png) file.


# How to use

- Clone this repo or download the scripts individually.

- Make sure required libraries are installed and execute python script using python 2.7.


# Authors

- Behrooz Bajestani,moradi.bajestani@husky.neu.edu
- Bishwarup Neogy, neogy.b@husky.neu.edu
