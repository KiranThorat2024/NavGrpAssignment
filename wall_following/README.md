# Wall Following #
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
* To run using the right wall follow mode
    * roslaunch wall_following wall_following.launch right_wall_mode:=1
* To run using the centerline follow mode
    * roslaunch wall_following wall_following.launch right_wall_mode:=0 

## Implementation ##
The main difficulties we had was not crashing into walls during sharp turns. To remedy this, we added velocity constraints dependent on the error value. In the extreme cases where the error was very large due to a sharp turn, we made the velocity very small so that the vehicle is able to turn in time.

The PID controller was initially tuned roughly using the Ziegler Nichols method. Then we tuned it manually until we found the desired response that did not cause a crash during sharp turns. 

Another minor addition we added was racecar_barca.launch which is slight modified version of racecar.launch. The slight modification was to the default yaw value so that vehicle is in a better yaw angle to start with for our testing.