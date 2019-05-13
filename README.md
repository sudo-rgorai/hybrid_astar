## Hybrid A* Path Planner ROS Package

This is `ros`-integrated package for Hybrid A* Path Planner.The underlying method is similar to A* search algorithm, applied to the 3D kinematic state space of the vehicle, but with a modified state-update rule that captures the continuous state of the vehicle in the discrete nodes of A* thus guaranteeing kinematic feasibility of the path.

#### Prerequisites
- This is not the official installation process but a medium article thats well formed<br/>
 [gazebo-8](https://medium.com/@abhiksingla10/setting-up-ros-kinetic-and-gazebo-8-or-9-70f2231af21a?fbclid=IwAR3pPyfB7X_1MiqBNpAEK2-7IqwZ3YtpzuMwxEa8AL5qKq4hSNiTnZbGrQs) <br/>
- This is car_demo package used for simulation during the development of this package<br/>
  [car_demo](https://drive.google.com/drive/folders/1t3Mamr8fq8slctGyB_iglyOQkyteMKBQ?usp=sharing)<br/>
  The official package:<br/>
  [car_demo](https://github.com/osrf/car_demo)<br/>
- This is the official `ros` package for costmap_2d. The associated launch file that work with this gazebo simulation is part of the Hybrid A* package. <br/>
  [costmap_2d](https://github.com/strawlab/navigation/tree/master/costmap_2d)<br/>

#### Building

1.`git clone` the following repository into your catkin workspace\
2.`catkin_make` the package

#### `ROS` topics

- odometry topic - `/base_pose_ground_truth`
- goal topic - `/move_base_simple/goal`  
- costmap topic - `/costmap_node/costmap/costmap`
- path topic - `astroid path`

#### Running

**Launching OSRF world**<br/>
`roslaunch car_demo demo.launch`

**Launching costmap_2d package**<br/>
`roslaunch costmap_2d gazebo.launch`

**Launching hybrid astar planner**<br/>
`rosrun hybrid_astar hybridAstar_node`

**Launching velocity publisher**<br/> 
`rosrun hybrid_astar velocity_publisher`<br/>

The previous three steps can done using `hybrid_astar.launch`<br/>
Next publish the goal on `/move_base_simple/goal` using `2D Nav Goal` in `rviz` <br/>
The path is published on `/astroid_path`. Select it on rviz to visualize the path published.<br/>

**Launching tracking method**<br/>
`python catkin_ws/src/tracking_control/src/scripts/tracking_Methods/pure_pursuit.py`

**Launching controllers**<br/>
`python catkin_ws/src/tracking_control/src/scripts/controllers/PID_velocity_controller.py`

#### Images
Gazebo Simulation<br/>
![image](https://drive.google.com/uc?export=view&id=15-kqJRyS5_auLp9y4gNx95SUvupi-068)

Path Planned by Hybrid A*<br/>
![image](https://drive.google.com/uc?export=view&id=15JqaoWBB1ZlF8xwrw4Ds7CTJv9RFrCWB)

Rviz Simulation<br/>
![image](https://drive.google.com/uc?export=view&id=1Xs8r86dIlGmqcnmqkDH6teeXNFR0b4Bb)

#### Video for Gazebo simulation

[Video](https://www.youtube.com/watch?v=vPbcxREJunU)

#### Resources
[Practical Search Techniques in Path Planning for Autonomous Driving](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf)






