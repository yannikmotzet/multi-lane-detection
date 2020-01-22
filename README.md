attention: manual not tested yet! (on test system it is working)

## How to clone
* clone in your ROS workspace: ``` ~/catkin_ws/scr/ ```
* ``` ~/catkin_ws$: . ~/catkin_ws/devel/setup.bash ```

## How to build
* build the project: ``` ~/catkin_ws$: catkin_make ```

## How to run
start roscore master:
* ``` ~/catkin_ws$: roscore ```

for each step open new terminal and paste ``` source ./devel/setup.bash ```
* ``` ~/catkin_ws/src/lane-regression$: rosrun lane-regression talker_cluster.py ``` (talker simuates interface from LaneDetection)
* ``` ~/catkin_ws$: rosrun lane-regression laneregressionr.py ```  (main node)
* ``` ~/catkin_ws$: rosrun lane-regression listener_laneassist_dummy.py ``` (listener simulates interface to LaneAssist)
* 

## Result example
![result](result.jpg)