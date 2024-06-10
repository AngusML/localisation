# localisation

FYI: The files in the top-level directory are ones I’ve edited. It is also found inside src directory.

NOTE:
For this project, I'm using ROS humble and ubuntu 22.04.4 LTS (jammy).
Terminal commands to check version are:
ubuntu:
1. lsb_release -a
ros:
1. rosversion -d
OR
2. echo $ROS_DISTRO


This README.md is for client to follow the steps for ease of setup (if needed) and assessment of the work I have done.
places where i have edited code are commented 'A ADDITIONS' OR 'A NOTES'

INFO:
1. using tag 0, 1, 2, 3, and 4 of tag36h11 family
2. height and width of each tag is 0.05m by 50mm


INITIAL DIRECTORY SETUP
===
mkdir -p ~/ros2_ws/src

PACKAGES INSTALLATION:
=============

[intel realsense D435i]
1. follow startup guide booklet (that comes in the box).
2. cd ~/colcon_ws/src
3. git clone https://github.com/IntelRealSense/realsense-ros.git

   NOTE: file named 'realsense-ros' will appear in /src directory after this step

   OPTIONAL: git clone https://github.com/pal-robotics/realsense_gazebo_plugin.git


[aruco_ros]
1. cd ~/colcon_ws/src
2. git clone https://github.com/pal-robotics/aruco_ros.git

   NOTE: file named 'aruco_ros' will appear in /src directory

   OPTIONAL: git clone https://github.com/AprilRobotics/apriltag.git


STEPS TO DISPLAY TAG/S IN RVIZ:
=============
  COMMANDS:
  1. ros2 launch realsense2_camera rs_launch.py
  2. ros2 launch aruco_ros single.launch.py
  3. rviz2
    /n ON RVIZ SIMULATOR:
    /n a) click 'Add'
    /n b) click 'By topic'
    /n c) click 'image' under dropdown '/result' which is under dropdown '/aruco_single'
  4. ./marker_publisher
     NOTE: run this executable in the build directory OR hit run in vscode (for 'marker_publisher' executable). This executes the code in marker_publish.cpp

RESULT OF THESE STEPS: tag detection & visualisation should be visible of what camera lens is pointing at will be visible and displayed at bottom left window. Tag ID's respective estimated position should be visible and printed in the terminal.




LOCATION OF TAGS:
=============
  ON TERMINAL:
  1. I have implemented a function called getTransform() from a library
  2. I have created a function to called translation() to extract translation part of 4x4 transform that contains translation and rotation
  3. using 'std::cout()' and 'ROS_INFO_STREAM()' I have isolated to a callback function (that publishes markers) and printed out the x y z position/location of tag/s in real-world 3D space.


GOING FORWARD:
=============
In the lines that prints the location of tags, going forward we can potential add the following approaches that can be used for UR3 drawing:
Offsetting - xyz position of tags on paper will compute camera's fixed position. Then setting an integer or double value in mm (as per the unnit of the code) of camera's fixed position relative to UR3 base position and using forward kinematics to find end effectors position. This allows the code to integrate to the path planning of UR3 to draw on paper.

NOTE: CHANGE getTranslation() function TO RETURN THE POSITION OF EACH ID (PAIR IT WITH EACH ID) (aka. each ‘i’ iterator)

update: getTranslation() is now not used






MARKER_PUBLISH.CPP BREAKDOWN
===
[setup()]
initialises all relevant publishers subscribers
parameter, pointer and variable declarations.

[image_callback()]

core sub-functions of image_callback() function
PART 1:
	parameterised void aruco::MarkDetector::detect()

PART 2:
	parameterised bool ArucoMarkerPPublisher::getTransform()
	parameterised tf2::fromMsg()
	parameterised tf2::Transform aruco_ros::arucoMarker2Tf2()

PART 3:
rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::publish<std_msgs::msg::UInt32MultiArray>(const 
marker_list_pub_->publish(marker_list_msg_)
std_msgs::msg::UInt32MultiArray &msg)
parameterised void aruco::Marker::draw()
parameterised static void aruco::CvDrawingUtils::draw3DAxis()

PART 4:
image_pub_.publish(out_msg.toImageMsg())

   NOTE: a few minor functions are left out
