# Code Description

## Circular Path Open Loop Controller
1. The code accepts the angular rate of rotation (w) and a radius of the circle (r)
2. The linear velocity of the turtle is calculated as v = w*r
3. The linear velocity and the angular velocity of the turtle are specified in the form of a twist and published
4. The code additionally tracks the original orientation of the turtle and determines when it completes 1 full circle

![Circular Trajectory followed by th Turtle](/Images/Circle.png)

## Square Path Open Loop Controller
1. The code accepts the angular rate of rotation (w) and the global linear velocity (v)
2. The square is divided into 4 segments which are cycled through based on x,y location of the
   - Move Right [Vx = v]
   - Move Up	[Vy = v]
   - Move Left	[Vx = -v]
   - Move Down	[Vy = -v]
3. The turtle's current orientation is mapped from (-pi/2, pi/2) to (0, 2*pi) and a rotation matrix R is calculated 
4. The body velocity of the turtle is calculated as Vb = R*[Vx, Vy]'
5. The linear velocity and the angular velocity of the turtle are specified in the form of a twist and published

![Square Trajectory followed by th Turtle](/Images/Square.png)

# Launching files
Launch circular trajectory open loop controller

```console
roslaunch assignment2_ws circular.launch
```

Launch square trajectory open loop controller

```console
roslaunch assignment2_ws square.launch
```

