# How to run #
* Clone repository
* Create a softlink of this folder to your /sae_ws/ros_ws/src directory. "ln -s source destination"
* run cmake using command "cm"
* **TODO: Instructions on how to Run the launch file**
* **TEMPORARY INSTRUCTION, delete once launch file is updated**
    * Make sure the launch file f1_tenth.launch is modified to uncomment the argument world_name with value="racecar_wall". This way gazebo will bring up the racecar_wall world
    * "roslaunch race f1_tenth.launch"
    * "rosrun automatic_emergency_braking sae_aeb.py"
