# COM3528: Investigating Maze Navigation Approaches for MiRo Robots

## Usage

Prepare the project using the following commands:

```sh
catkin build
src
```

Before launching any of the control methods, open the simulated Gazebo environment using the `sim.launch` launch file.

```sh
roscore
roslaunch team08 sim.launch
```

### Sonar

```sh
roslaunch team08 ctrl_sonar.launch
```

### Colour Detection

```sh
roslaunch team08 ctrl_color.launch
```

### Stereo Imaging

```sh
roslaunch team08 stereo_camera.launch
```

While runnning, view the stereo imaging of the `stereo_image_proc` package by running the following command:

```sh
rosrun image_view stereo_view stereo:=/miro/sensors/stereo image:=image_rect_color _approximate_sync:=True
```

While running, view the point clouds produced by running RViz and adding the following topics:
* `stereo_image_proc` (ROS built-in): `/miro/sensors/stereo/points2`
* `stereo_camera_depth` (custom): `/miro/sensors/stereo/depth`

### Odometry

```sh
roslaunch team08 ctrl_odometry.launch
```
