attention: manual not tested yet!

## How to clone
* clone in your ROS workspace: ``` ~/catkin_ws/scr/ ```
* ``` ~/catkin_ws$: . ~/catkin_ws/devel/setup.bash ```

## How to build
* build the project: ``` ~/catkin_ws$: catkin_make ```

## How to run
for each step open new terminal and paste ``` source ./devel/setup.bash ```
* ``` roscore ```
* ``` rosrun lane-regression talker ```
* ``` rosrun lane-regression listener.py ```