# Lane Keep Assist #
Group 4 - Navistar

* Cristian Valadez
* Vivek Bonangi
* Amy Kofoed
* Kiran Thorat
* Cory Packard

## Some notes ##

* Regarding the video of turtlebot3 navigating my living room, obviously this map will not work unless you are in my living room.
* In the video, the pillow and backpack were not in the original map, they were added obstructions. That is why in the video the robot struggles and reverses when it encounters the new obstructions. I did tune the costmap_common_params_burger.yaml slightly. Changed inflation radius to 0.125 and cost scaling factor to 5.0. This file is found in /opt/ros/noetic/share/turtlebot3_navigation/param/.
