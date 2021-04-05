# warmup_project

### Zhou Xing 
##### zhouxing@uchicago.edu

Last Edit: Apr 4, 2021

## Driving in a Square

### Introduction
For this part, [./scripts/drive_square.py](./scripts/drive_square.py) controls a turtlebot3 model to run through a square path with side length 1. We use a timing strategy to make sure that the robot moves forward for a fixed duration for each side of the square, or changes direction with a fixed angular speed and duration as well. 

### Code Explanation

We wrote up the `drive_square.py` with OOP, specifically, to a `DriveSquare` object. We encapsulate two actions -- _move forward_, and _change direction_ -- into two methods `self.move_forward` and `self.turn_to`. And finally, use a `self.move` method to accomplish a pipeline of these two actions. 

For accurate tuning, we allow customization of the loop rate in each action, with a `self.loop_rate` attribute in `DriveSquare`'s `init` function. 

For both `self.move_forward` and `self.turn_to`, we are using a timing strategy, i.e. letting the robot to move a fix speed for a fixed duration. We also allow customization of these two parameters. We also force the robot to stop at the end of each move, either it's moving forward, or turning head.

For the `self.move` method, we simply let the robot to do `move_forward` and then `turn_to` for four times. 

### Demo gif

(Please wait for about 5 seconds at the beginning of the gif.)
<img src="gifs/drive_square.gif" width=800>

### Challenges

1. System setting: It took me the whole weekend to get my Gazebo work. It turns out that at the end reset my machine is the best solution lol.
2. Parameter Tuning: I spent one hour to make sure my robot turn by a more accurate 90 degree. I also found that the robot is very sensitive to a slight change (<0.1) of angular speed. 


### Future Work

Tune more and find more interesting things in controling the robot. I notice that though I set the robot to move forward, it tends to drift. Also, I used `self.speed_pub.publish(Twist())` to make the robot stop at the end of each move, but it seems that in some situation this would invoke the robot to turn as well. Not sure about this and will check it out later.

### Takeaways

1. **Leave more time for the assignment**. Because you have no idea what bug the system would bring you. (I spent about 2 hours to finish the coding, but 2 days for the system set up.)
2. **The parameter tuning can be tricky**. I finally settled with a `0.502` for the angular speed for each turning, and a duration of `3`. It seems that for a more accurate performance of robot, sometimes I have to try many times for a magic number. 
