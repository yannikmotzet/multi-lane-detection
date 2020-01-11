attention: manual not tested yet!

## How to clone
* clone in your ROS workspace: ``` ~/catkin_ws/scr/ ```
* ``` ~/catkin_ws$: . ~/catkin_ws/devel/setup.bash ```

## How to build
* build the project: ``` ~/catkin_ws$: catkin_make ```

## How to run
start roscore master:
* ``` ~/catkin_ws$: roscore ```

for each step open new terminal and paste ``` source ./devel/setup.bash ```
* ``` ~/catkin_ws$: rosrun lane-regression talker_cluster.py ``` (talker simuates interface to LaneDetection)
* ``` ~/catkin_ws$: rosrun lane-regression laneregressionr.py ```  (main node)
* ``` ~/catkin_ws$: rosrun lane-regression listener_laneassist_dummy.py ``` (listener simulates interface to LaneAssist)