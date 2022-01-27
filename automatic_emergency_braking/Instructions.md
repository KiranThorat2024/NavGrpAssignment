### <u>Automatic Emergency Braking</u>

Implement a P-controller to stop the vehicle before it collides with a wall. The threshold can depend on:

1. The Euclidean distance between vehicle and wall (distance error), or
2. The Time-to-collision (TTC) between vehicle and wall (velocity error)

##### <u>Instructions:</u>
Complete `sae_aeb.py` with relevant logic. 

Launch the gazebo simulator and bring up the vehicle with the wall world. Uncomment the `racecar_wall` world file in `f1tenth.launch`.

Make your python code executable:

```bash
$ chmod u+x sae_aeb_yourname.py
```

The task is complete when the vehicle stops at a certain distance from the obstacle in front of it.

**<u>TO-DOs:</u>**

1. Use the ```get_index``` function to return the indexes of the required ranges of angles of the LIDAR beam. You can do this by finding the value of number of indices per degree of scan in `ranges` array and then multiplying it by the required angle.
2. Once you complete this function, you can use it to get a distance at a particular angle in the scan frame of reference (i.e. the hokuyo LIDAR as origin). 
3. Implement the two kinds of errors above to convert this range in front of the vehicle to an error that will be 0 if the vehicle is in the threshold of an emergency braking scenario. Implement a proportional controller to use the error signal to regulate velocity in a smooth manner and gently stop the vehicle before collision
4. Call the distance function and controller in the Callback function for the publisher.  You can only call one controller at a time. 
5. Tune the controller
6. Finally, create a launch file to bring up the simulator and launch your code.
7. In your README, explain which control method is preferable and why. 

<u>Note:</u> The TO-DOs in the skeleton code are meant to be a guide, but the function layout does not have to be strictly followed. As long as the code performs the two kinds of AEB using P-control, the criteria will be considered met. 

#### <u>Submission</u>
All your assignment code submission is expected to be pushed to your GitHub fork.  

**DO NOT FORGET TO INCLUDE A README.** **The README files should contain instructions on how to run your code. Specifically, you should have the commands the TA should enter into the terminal to run your code.**

It is also recommended that you place comments in your code at places that they may be required where you explain what exactly that piece of code does.

Once you complete the required tasks, push the changes into GitHub. To recap, navigate to your ```git_ws``` and run the following commands:

```bash
$ cd ~/sae_ws/git_ws/bootcamp-assignments
$ git status
```

This should show the current status of your repository. If you have made any changes, stage them using:

```bash
$ git add .
```

This will stage all your local changes, now you can commit those changes using:

```bash
$ git commit -m "your comments on the changes made"
```

Finally push those changes to your remote git server using:

```bash
$ git push origin master
```

You should now be able to see your changes in your fork online. 

Once you have done this, upload a link to your Github repository to canvas along with a video of your code working in the simulator environment.



