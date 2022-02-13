# Lane Keep Assist #
Group 4 - Navistar

* Cristian Valadez
* Vivek Bonangi
* Amy Kofoed
* Kiran Thorat
* Cory Packard

## How to run ##

* Clone repository
* Replace the files turtlebot3_burger.gazebo.xacro and turtlebot3_burger.urdf.xacro found in your local /opt/ros/noetic/share/turtlebot3_description/urdf/ with the files included in folder /xacro_files
    * These files were edited to include the Raspberry PI camera model. They were also edited to raise the camera up slightly to get a better view of lanes
* Create a softlink of this folder to your /sae_ws/ros_ws/src directory. "ln -s source destination"
* run cmake using command "cm"
* run "source devel/setup.bash" from the ros_ws directory or close terminal and open again
* Run the launch file: "roslaunch lane_keeping lane_keep.launch"