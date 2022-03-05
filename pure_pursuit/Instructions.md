### Group Project: Pure Pursuit (Carrot Planner)

**<u>Objective</u>** 

The goal of this exercise is to make the f1tenth car to follow waypoints using pure pursuit. To do this, you will create a (.csv) file containing the list of waypoints and then write a pure-pursuit control algorithm  which will make the car follow all the points. 

The skeleton code is provided in `bootcamp_assignments/pure_pursuit`  

1. `waypoint_logger` - This file records waypoints in a .csv file.  
2. `pure_pursuit_control.py` - Skeleton code for pure pursuit.  

**<u>To Do</u>** :

1. **Create a list of way-points by manually driving the car around in the Gazebo world (pick any course world)** 

    Use the `waypoint_logger` node to record waypoints. 

    ```bash
    $ rosrun waypoint_logger waypoint_logger.py
    ```

    Run the waypoint logger and begin teleoping. Press `ctrl + c ` to stop the logging.

    Make sure that you're saving the way-points in a correct manner. Go into the python script `waypoint_logger.py`  to change the filename to your desired filename and also make sure that the .csv file is getting saved at the correct location once the file is logged with all the way-points. 

       

2.  **Write the pure pursuit controller to follow the way-points.**  

   1. Parse the saved csv file in `pure_pursuit.py`. Convert the waypoints to a numpy array.
   2. Declare the publishers and subscribers.
   3. Develop the logic for finding out the *'instantaneous carrot position'* based on the list of way-points and current position of the car. 
      1. Use the theory discussed in class to formulate the logic to find out the interpolated points
      2. Some of the helpful functions you'd need in order to proceed are - 
         1. `find_distance(x,y)` , `find_index_based_distance(idx1,idx2)` , `find_nearest_waypoint()` This list is just to help you to think in the right direction. You can follow your own approach to complete the algorithm. 
      3. Set a random `LOOKAHEAD_DISTANCE` and make sure that all the interpolated points are always `LOOKAHEAD_DISTANCE` away from your car. This is a test to make sure that your interpolation logic is working well. 
      4. Once you are able to sample the interpolation points correctly, write a pure pursuit control code for your car to follow those *'instantaneous carrot positions'*.
   
   

**<u>Submission</u>**

All your assignment code submission is expected to be pushed to your Git-hub repository.  Submit link to the repository on canvas along with a video of your code working in the simulator environment.

**The READ-ME files should contain instructions on how to run your code. **

It is also recommended that you place comments in your code at places that they may be required where you explain what exactly that piece of code does.