# CarND-Path-Planning-Project

### Steps：

Step1. Jeck, acceleration and speed control mechanisms.
Step2. Collision avoidance mechanism.
Step3. Finite state.
Step4. Trajectory planning--Cubic Spline Interpolation Algorithm.

#### Step 1--Jerk, acceleration and speed control mechanisms:

In order to reduce the discomfort caused by sudden acceleration and deceleration; We have limited the acceleration and deceleration of the ego car.

**line 57**: Set the initial speed to zero .
**line 228**: Set the acceleration to 5m/s^2. ref_vel + 0.224;

We did the same for deceleration.
**line 225**： Set the deceleration to 5m/s^2. ref_vel - 0.224;

There are three main factors for jerk limits:
Acceleration, Maximum Speed, and Curvature of Vehicle Trajectory;
**line 227**: I set the maximum speed limit. ref_vel < 49.5;
The problem of jeck limited caused by sudden speed changes has been described above by gradual acceleration and deceleration.
Below will be described how to solve the smooth trajectory planning problem in the trajectory generation model.

#### Step 2--Collision avoidance mechanism:

The idea of the Collision avoidance mechanism is keeping distance from the car in front of the current lane.
**line 107 - 125**
The sensor fusion data will giving all the other cars's parametres on the road.
Therefore, through the D'value of each car, we know whether the car is in the current line or not. **line 112**

If there is a car in front of the current lane, we need to judge whether the car will collide.
The method is judge whether the distance between the future position of the car ahead and the ego car is within 30m.**line 121**

#### Step 3--Finite state mechanism:

**line 136 - 229**
In this project, I used three states to decide the next action of the vehicle.

-- change_left_lane = false
-- change_right_lane = false
-- keep_lane = true

In this project, I did't use the loss function. Only according to whether the ahead car is too close as a trigger condition.

At the same time, I set a safety condition.-- Comparing the positions of the vehicles on the target lane and the ego cars, if the distances are all greater than 30m, lane change safety can be determined.

if safety , we change the current lane to the goal lane. **line 220-226**

#### Step-4 Cubic Spline Interpolation Algorithm:

**line 237 - 339**
Trajectory generation code is inspired from project code walk through in course.
It can be summarized as the following three steps：

1. Generate initial anchor point.
2. Generating the spline function according to anchor points.
3. Generating a trajectory according to the spline function.

When generating anchor points, in order to make our trajectory smoother, we need to make full use of the information of the previous path.
Therefore, we need to consider the remaining waypoints of the previous path. If less than 2, we need to calculate the tangent of the current car steering angle to generate 2 points; Otherwise, the last 2 points of the previous path are directly used as the starting point for generating the next path.**line 246 - 274**

At the same time, a target point is set every 30m, and a total of 3 points are set.**line 275 - 285**

Then use the spline library to create the spline function. **line 298**

Finally, the next path trajectory is generated according to the spline function. **line 310 - 334**
