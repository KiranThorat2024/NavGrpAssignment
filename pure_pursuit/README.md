# Pure Pursuit #
Group 4 - Navistar

* Cristian Valadez
* Vivek Bonangi
* Amy Kofoed
* Kiran Thorat
* Cory Packard

## How to run ##

* Clone repository
* Create a softlink of this folder to your /sae_ws/ros_ws/src directory. "ln -s source destination"
* run cmake using command "cm"
* run "source devel/setup.bash" from the ros_ws directory or close terminal and open again
* Run the launch file: "roslaunch pure_pursuit purepursuit_control.launch"
    * Or you can run "roslaunch pure_pursuit spawn_racecar_porto.launch" and then run "rosrun pure_pursuit pure_pursuit.py". This method allows for setting up of camera on gazebo before running script and getting better console prints for debugging
