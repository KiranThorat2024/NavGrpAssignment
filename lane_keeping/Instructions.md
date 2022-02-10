### Autonomous Lane Keeping

This in assignment you will use some of the skills in OpenCV to implement and autonomous lane keeping controller. 

**Instructions:**

Complete the `simple_lane_keep.py` to implement the lane keeping controller. Bring up the simulator with the `lane_keep` world. 

This homework will feature fewer explicit instructions on implementing the controllers.

**TO-DOs:**

1. Write the publishers and subscribers necessary with the appropriate callback function.
2. Within the callback function, convert the data to cv2 type and implement a number of functions on it to extract the lane lines.
3. As shown in class, one of the first things to implement would be to extract the canny lines and the region of interest that the camera frame would look at.
4. Extract the Hough Lines from the canny image
5. Use the information  of the Hough lines to extract the coordinates of the average centerline of the frame within the frame of the camera. The difference between this angle and the desired angle of the robot will form the  error for the controller
6. Finally, implement the controller and publish the vehicle velocity and steering angle.
7. Implement a launch file to bring up the robot in the `lane_keep` world. When you first bring it up, the robot will not lie in-between the lane lines. Pass arguments into the `racecar.launch` file to spawn the robot anywhere on the lane, between the yellow lines. 

<u>Note:</u> You may have to change the position of the camera on the robot by editing the correct `.xacro` files in order to see the lane better in the camera frame. 

**<u>BONUS:</u>** Congratulations, you have now implemented lane keeping! There is however a big issue with the method of lane keeping we have implemented. We haven't taken into account the transformations from camera frame to the world frame of the robot! Can you account for this?

**Submission:**

All your assignment code submission is expected to be pushed to your GitHub fork.  

Once you have done this, upload a link to your Github repository i.e. ```https://github.com/<your_username>/bootcamp-assignments.git``` to Canvas along with a video of your code working in the simulator environment.

**DO NOT FORGET TO INCLUDE A README.** **The README files should contain instructions on how to run your code. Specifically, you should have the commands the TA should enter into the terminal to run your code.**

It is also recommended that you place comments in your code at places that they may be required where you explain what exactly that piece of code does.



