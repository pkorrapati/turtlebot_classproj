# Results

## Circle, Slow [w=0.1]
![Image, Circular Trajectory, Slow Speed,   w=0.1](./images/Circle_Speed_Slow.png)

[<img src="https://img.youtube.com/vi/6AAMIuGA4Gg/hqdefault.jpg"
/>](https://youtu.be/6AAMIuGA4Gg)

## Circle, Normal [w=0.3]
![Image, Circular Trajectory, Normal Speed, w=0.3](./images/Circle_Speed_Normal.png)

[<img src="https://img.youtube.com/vi/xuvNKPSpl8k/hqdefault.jpg"
/>](https://youtu.be/xuvNKPSpl8k)

## Circle, Fast [w=0.5]
![Image, Circular Trajectory, Fast Speed,   w=0.5](./images/Circle_Speed_Fast.png)

[<img src="https://img.youtube.com/vi/B_uCSd7c4_c/hqdefault.jpg"
/>](https://youtu.be/B_uCSd7c4_c)

## Square, Slow [w=0.1]
![Image, Square Trajectory, Slow Speed,   w=0.1](./images/Square_Speed_Slow.png)

[<img src="https://img.youtube.com/vi/HPHVHBC1qM8/hqdefault.jpg"
/>](https://youtu.be/HPHVHBC1qM8)

## Square, Normal [w=0.3]
![Image, Square Trajectory, Normal Speed, w=0.3](./images/Square_Speed_Normal.png)

[<img src="https://img.youtube.com/vi/vcKnU5jD3Zs/hqdefault.jpg"
/>](https://youtu.be/vcKnU5jD3Zs)

## Square, Fast [w=0.5]
![Image, Square Trajectory, Fast Speed,   w=0.5](./images/Square_Speed_Fast.png)

[<img src="https://img.youtube.com/vi/VpMnhz8u0oI/hqdefault.jpg"
/>](https://youtu.be/VpMnhz8u0oI)

# Prerequisites
Depends on matplotlib python library

```console
sudo apt install python3-pip
python3 -m pip install matplotlib
```
# Launching files
## Circular Trajectory
Launch circular trajectory

```console
roslaunch assignment2b_turtlebot3 move.launch code:='circle'
```

Launch circular trajectory (Slow)

```console
roslaunch assignment2b_turtlebot3 move.launch code:='circle' w:=0.1
```

Launch circular trajectory (Fast)

```console
roslaunch assignment2b_turtlebot3 move.launch code:='circle' w:=0.5
```
## Square Trajectory
Launch square trajectory

```console
roslaunch assignment2b_turtlebot3 move.launch code:='square'
```

Launch square trajectory (Slow)

```console
roslaunch assignment2b_turtlebot3 move.launch code:='square' w:=0.1 v:=0.1
```

Launch square trajectory (Fast)

```console
roslaunch assignment2b_turtlebot3 move.launch code:='square' w:=0.5 v:=0.5
```


