# Hybrid A* Path Planner

This is `ros`-integrated package for Hybrid A* Path Planner. 

### Prerequisites

- [Gazebo-8](https://medium.com/@abhiksingla10/setting-up-ros-kinetic-and-gazebo-8-or-9-70f2231af21a?fbclid=IwAR3pPyfB7X_1MiqBNpAEK2-7IqwZ3YtpzuMwxEa8AL5qKq4hSNiTnZbGrQs) 
- [Costmap_2d](https://github.com/strawlab/navigation/tree/master/costmap_2d)

### Installing

1.`git clone` the following repository into your catkin workspace

2.`catkin_make` the package

### Running

#### Launching OSRF world
`roslaunch car_demo demo.launch`

#### Next we need to convert the laserscan to costmap_2D
`roslaunch costmap_2d gazebo.launch`

#### Launching hybrid astar planner
`rosrun hybrid_astar hybridAstar_node`

#### Publishing velocity 
`rosrun hybrid_astar velocity_publisher`

#### Launching tracking method
`python catkin_ws/src/tracking_control/src/scripts/tracking_Methods/pure_pursuit.py`

#### Launching controllers
`python catkin_ws/src/tracking_control/src/scripts/controllers/PID_velocity_controller.py`

### Videos

[Video](https://www.youtube.com/watch?v=vPbcxREJunU)

### Resources
[Practical Search Techniques in Path Planning for Autonomous Driving](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf)






