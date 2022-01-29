# Automatic Emergency Braking #
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
* To run using the distance to collision controller
    * roslaunch automatic_emergency_braking aeb_distance_controller.launch
* To run using the time to collision controller
    * roslaunch automatic_emergency_braking aeb_ttc_controller.launch
    
## Why TTC is better than Euclidean ##
Using time-to-collision algorithm is better than a distance-to-collision because the ttc algorithm will correctly take into account the relative speeds of the two objects that are about to collide. For example, if a vehicle is about to collide with a wall with a threshold set to 5m, if the vehicle was driving at 10mph, the vehicle may stop in time. If the vehilce was going 100mph, it will not stop in time. By using the ttc algorithm, the vehicle driving at 100mph will calculate a time-to-collision much farther away from the wall than if it was driving at 10mph. This will give the vehicle more distance to brake due to its higher speed.
