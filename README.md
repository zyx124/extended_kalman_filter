# extended_kalman_filter
An example of implementation of EKF on ROS Kinetic which runs on Ubuntu 16.04.

In the 2D space, the robot has three parameter to describe its state, that is (x, y, theta). The model for the robot is:

```
x(k+1) = x(k) + t * v_t * cos(theta)
y(k+1) = y(k) + t * v_t * sin(theta)
theta(k+1) = theta(k) + t * v_ang
```

in which t is a step of time, v_t is translate velocity and v_ang is the angular velocity. The model is with the noise, whose covariance is 0.0025 and 0.005 for translation and rotation respectively.

When the robot is close to the landmarks, the sensor get the location of landmarks and then calculate the bearing and range in the following equation:

```
range = sqrt((xr-xl)^2 + (yr-yl)^2)
bearing = atan2(yl-yr, xl-xr) - thetar
```

in which (xr, yr, thetar) represent the state of the robot and (xl, yl) indicate the location of landmarks. There are also noise for the sensor measurements.

To run it, first build it in the workspace and then source the setup.bash (if you use zsh, source setup.zsh).Then in the terminal:

```
roslaunch state_estimator estimator.launch
```

You can see in the GUI there are two robot are in the middle without moving.

Then open a new terminal:

```
rosrun state_estimator robot.py
```

The landmarks appear and the robots start to move randomly. 

![moving](https://github.com/zyx124/extended_kalman_filter/blob/master/state_estimator/state_estimator.png?raw=true)

You can also open the nodes separately but the estimator must be opened **before** the robot moves.You can modify the est.py file to see changes in the behavior of the robot.



