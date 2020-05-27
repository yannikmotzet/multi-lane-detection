<<<<<<< HEAD
# Lane Keeping Assist

### Components
* Lane Detection (current editor: Fabian Hauck)
* Lane Regression (current editor: [Yannik Motzet (TIT17)](mailto:yannik.motzet@outlook.com))
* Lane Assist (current editor: Inci Sahin)

## Getting started
### How to clone
* clone in your ROS workspace: ``` ~/catkin_ws/scr$: https://gitlab.com/zfinnolab/lane-detection-assist/lane-keeping-assist-2020 ```
=======
## About Project
author: Yannik Motzet (TIT17)
dummy lanedetection retrieves points from files and publishes data
laneregression determines functions for each line, calculates an ideal line and calculates an offset, publishes offset
dummy laneassist takes the offset and calculates an steering angle, sends steering angle via UDP message to steering control unit

this project will be further developed in the repo lane_keeping_assist


## How to clone
* clone in your ROS workspace: ``` ~/catkin_ws/scr$: git clone https://gitlab.com/zfinnolab/lane-detection-assist/laneregression.git ```

>>>>>>> ccdca0726feb59eeeb5468b4acecec3e4ce70499


### How to build
* ``` ~/catkin_ws$: source /opt/ros/melodic/setup.bash ```
* ``` ~/catkin_ws$: source devel/setup.bash ```
* build the project: ``` ~/catkin_ws$: catkin_make ```

### How to run
* ```~/catkin_ws/$: source devel/setup.bash ```

run all with roslaunch
* ```~/catkin_ws/$: roslaunch lane-keeping-assist detection_regression_assist ```

or start single nodes manually:
* start roscore: ```~/catkin_ws/$: roscore ```
for each step open new terminal and first paste ``` source devel/setup.bash ``` in ``` /catkin_ws$ ```
* ``` ~/catkin_ws$: rosrun lane-keeping-assist lanedetection ``` 
* ``` ~/catkin_ws$: rosrun lane-keeping-assist laneregression.py ```
* ``` ~/catkin_ws$: rosrun lane-keeping-assist laneassist.py ```


## Other things
### Get sample video of Truck Maker
* [Spurerkennungssimulation.avi](https://drive.google.com/open?id=1Fd3jdyYO9kUJk1QslhRScMUqpeO-pjwe)
* Save in: ``` ~/catkin_ws/scr/lane-keeping-assist-2020 ```

### How to: IPGMovie --> webcam
* use Capture Card (Mira Box)
* use [v4l2loopback](https://github.com/umlaeute/v4l2loopback) + [OBS](https://obsproject.com/de) (install v4l2loopback, install OBS, run ```sudo modprobe v4l2loopback```, in OBS: tools -> V4L2 Video output)
* use [v4l2loopback](https://github.com/umlaeute/v4l2loopback) + shell script (install  v4l2loopback, run ```./tools/screen_capture.sh```)


## How it works
### How LaneRegressions works (not updated yet)

preliminary work:
* in LaneDetection an algorithm for Persepctive Transformation was implemented (to get the top-down view)
* the cluster points of a few frames were written to a file

LaneDetection dummy publisher:
* retrieve test points from the file and publishes successively the frames with the clusterpoints on a topic

LaneRegression:
* subscribe to topic
* look for related cluster (dashed line consists of several clusters)
* for each line cluster run Ramer–Douglas–Peucker algorithm  (calculates less points)
* order new points
* calculate x(t) and y(t) functions (third degree polynomials)
* build a message which contains information about functions and publish it on a topic

LaneAssist dummy subscriber:
* subscribe to topic and parse information

#### Result example
Here is an example image of one frame.
* black lines: raw cluster
* blue points: start and end points of raw cluster 
* red points: points calculated by Ramer–Douglas–Peucker algorithm
* green lines: illustration of calculated functions
![result](result.jpg)

Result without Perspective transformation:
![result](result_without_perspect_transf.png)
