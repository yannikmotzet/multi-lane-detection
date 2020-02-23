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


## How LaneRegressions works

preliminary work:
* in LaneDetection an algorithm for Perspective Transformation was implemented (to get the top-down view)
* the cluster points of a few frames were written to a file

LaneDetection dummy publisher:
* retrieve test points from a file and publishes successively the frames with the clusterpoints on a topic

LaneRegression:
* lanregression node subscribes to topic
* look for related cluster (dashed line consists of several clusters)
* for each line cluster run Ramer–Douglas–Peucker algorithm  (calculates less points)
* order new points
* calculate x(t) and y(t) functions (third degree polynomial)
* build a message which contains information about functions and publish it on a topic

LaneAssist dummy subscriber:
* subscribe to topic and parse information

## Result example
![result](result.jpg)