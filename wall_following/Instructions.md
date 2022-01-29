### <u>Wall Following</u>

In the last assignment, you implemented velocity control. Now you will implement steering control to keep the vehicle parallel to a curving wall and a certain distance away from it. Your controller should be as smooth as possible and support a high speeds of the car. You will implement two versions, 

1. Vehicle should follow the right wall
2. Controller to follow the centerline of the walls

##### <u>Instructions:</u>

Complete `sae_wall_following.py` in the `wall_following` folder. The simulator must bring up the `track_barca` world. 

**TO-DOs:**

1. Write the publishers and subscribers necessary with the appropriate callback function

2. Use the `get_index` function to return the indexes of the required ranges of angles of the LIDAR beam. You can do this by finding the value of number of indices per degree of scan in `ranges` array and then multiplying it by the required angle.

3. Pick two rays, directly to the right of the vehicle, and one further ahead, as shown in the picture below:

   ![wall_follow](../_pictures/wall_follow.png)

   Use the `get_index` function from earlier to get the index of these selected angles and find the range values, **a** and **b**. Since theta is known, use geometry to find alpha, which is the angle of the vehicle from the centerline.  Finally, find the actual distance of the vehicle from the centerline. The error your controller should depend on is the difference between this value and desired distance from the wall. 

   <u>Note</u>: This method, while good enough at low speeds, will fail at higher velocities. This is because of the distance traveled by the vehicle in the time taken to execute the code through one loop. The best way to calculate that distance is to use the frequency of the publisher. Standardize the rate of the publisher by using `rospy.Rate(frequncy)` and `rospy.spin()`. The distance traveled by the vehicle in this time is now `velocity*frequency`. Include this term into the actual-distance-from-the-wall term through geometry.  Further implement a PD controller to improve your results. 

4. Implement the controller and publish the message to the publisher
5. Tune the controller
6. Finally, create a launch file to bring up the simulator and launch your code.
7. In your README, explain your implementation clearly. 

<u>Note:</u> The TO-DOs in the skeleton code are meant to be a guide, but the function layout does not have to be strictly followed. If you have chosen to implement the wall following logic differently, describe it in detail in the submitted README. 

The goal should be to make the car complete a lap of the track as fast as possible.

**Submission**

All your assignment code submission is expected to be pushed to your GitHub fork.  

Once you have done this, upload a link to your Github repository i.e. ```https://github.com/<your_username>/bootcamp-assignments.git``` to Canvas along with a video of your code working in the simulator environment.

**DO NOT FORGET TO INCLUDE A README.** **The README files should contain instructions on how to run your code. Specifically, you should have the commands the TA should enter into the terminal to run your code.**

It is also recommended that you place comments in your code at places that they may be required where you explain what exactly that piece of code does.