# localisation


INFO:
using tag 0, 1, 2, 3, and 4 of tag36h11 family
height and width of each tag is 0.05m by 50mm



INSTRUCTIONS:
=============
INITIAL DIRECTORY SETUP
mkdir -p ~/ros2_ws/src



PACKAGES INSTALLATION
[intel realsense D435i]
follow startup guide booklet (that comes in the box).
cd ~/colcon_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git
NOTE: file named 'realsense-ros' will appear in /src directory after this step
OPTIONAL: git clone https://github.com/pal-robotics/realsense_gazebo_plugin.git

[aruco_ros]
cd ~/colcon_ws/src
git clone https://github.com/pal-robotics/aruco_ros.git
NOTE: file named 'aruco_ros' will appear in /src directory
OPTIONAL: git clone https://github.com/AprilRobotics/apriltag.git



STEPS TO DISPLAY TAG/S IN RVIZ:
  COMMANDS:
  1. ros2 launch realsense2_camera rs_launch.py
  2. ros2 launch aruco_ros single.launch.py
  3. rviz2
    ON RVIZ SIMULATOR:
    click 'Add'
    click 'By topic'
    click 'image' under dropdown '/result' which is under dropdown '/aruco_single'
  4. ./marker_publish

NOTE: Image of what camera lens is pointing at will be displayed at bottom left window.

===

