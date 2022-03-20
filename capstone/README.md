# Capstone Project
Group 4 - Navistar

* Cristian Valadez
* Vivek Bonangi
* Amy Kofoed
* Kiran Thorat

# How to run
Make sure you add the following to your .bashrc file so that Gazebo knows where to find the model that was created for the blue line.
```bash
export GAZEBO_MODEL_PATH=~/path_to_package/capstone/worlds/models/
```


# Tasks
## Task1: (A) Wall follow
For this task, the robot does a simple right wall follow. Some logic was added to detect an imminent head on collision. If detected, robot will turn the other way slowly until collision is not triggered anymore.

## Task2: (A) Line following with object detection
For this task, the robot follows the tape on the floor. Object detection was not implemented due to time.

## Task3: (B) Pure pursuit using waypoints
Pure pursuit algorithm using defined waypoints was used for this task.
