# TagSLAM MRSS root repository

This is a TagSLAM fork for the Montreal Robotics Summerschool (MRSS).

The main objective of this repository is to use TagSLAM to estimate the pose of a Go1 robot and
the poses of April Tags in the environment using a RealSense camera and some odometry. This code uses ROS 1 and broadcasts all transforms
to the ```tf``` topic.
## Installation Instructions

### TagSLAM
Follow the installation instructions in the original [TagSLAM README](https://github.com/sachaMorin/tagslam_root). Keep in 
mind TagSLAM relies on a lot of submodules. Make sure to clone them. Install TagSLAM on your computer (not the robot).

### Realsense Camera
Then install the [Realsense ROS 1 Wrapper](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy). 

### Go1 Twist Controller
The Go1 already has a ```cmd_vel``` topic, but it appears to be unresponsive.

For high-level Go1 twist controls with ROS, we need the [go1-math-motion](https://github.com/dbaldwin/go1-math-motion) repo. Run the following in ```tagslam_root/src```:
```shell
git clone https://github.com/dbaldwin/go1-math-motion.git
git clone -b v3.8.0 https://github.com/unitreerobotics/unitree_legged_sdk
cd ~/tagslam_root
catkin_make
```
This will allow us to spin up an interface between the ```cmd_vel``` topic and the SDK, allowing twist controls.

Finally, make sure to build.
```shell
cd ~/tagslam_root
catkin_make
```

## Usage
In any terminal where you need to use TagSLAM, you should run
```shell
source ~/tagslam_root/devel/setup.bash
```

Then you can call the main launch file as follows
```shell
roslaunch tagslam mrss_laptop.launch rviz:=1
```
`rviz:=1` enables visualization. 

```mrss_laptop``` will launch appropriate April Tag detection and SLAM nodes and broadcast transforms to the ```tf``` topic.

## Transforms
TagSLAM broadcasts transforms for individual April Tags as well as the bodies they are attached to. Have a look at 
the [Concepts](https://berndpfrommer.github.io/tagslam_web/concepts/) page of TagSLAM. The most relevant transforms are
- ```cam0``` to ```map```: camera pose.
- ```rig``` to ```map```: rig/robot.
- ```boardX``` to ```map```: pose of the Xth board. We define a "board" as a foam tile with 4 april tags. The pose is with respect to the 
center of the tile.
- ```obstacleX``` to ```map```: pose of the center of the Xth obstacle. 

The boards are defined as static objects with known pose, while the rig, cameras and obstacles are dynamic.

The ```map``` frame is defined with respect to the static boards. **This means the camera needs to see a board at least
once to start localizing.**

## Launch Files and Configs
There are a few relevant files.
- ```src/tagslam/launch/mrss_laptop.launch```: The main launch file.
- ```src/tagslam/example/tagslam_mrss.yaml```: The main config file. This is where you can define all bodies and April Tags. This is also where you can remap the Odometry topic and change most TagSLAM settings.
- ```src/tagslam/example/camera_jackal.yaml```: Camera intrinsics and topics. See instructions [here](https://berndpfrommer.github.io/tagslam_web/intrinsic_calibration/).
- ```src/tagslam/example/camera_poses.yaml```: Camera extrinstics, i.e. ```cam0``` to ```rig```.
- ```src/tagslam/launch/apriltag_detector_node.launch```: April Tag detector node. This has to be changed if we move from raw to compressed images.
 
##  Using the demo ROS bag
You can download a demo bag [here](https://drive.google.com/file/d/1dtGW4ajMcIU0OFmfhAPx2s4sGFXasphg/view?usp=sharing).
You can play it using standard ROS commands or our launch file
```shell
roslaunch tagslam play_bag.launch bag_path:=$ABSOLUTE_PATH_TO_BAG rviz:=1
```

## Camera Calibration
You may need to calibrate the camera following the instructions [here](https://berndpfrommer.github.io/tagslam_web/intrinsic_calibration/). The following command may be useful

```shell
rosrun camera_calibration cameracalibrator.py --size 6x4 --square 0.108 image:=/camera/color/image_raw --no-service-check
```


## go1_odom Package
We did not manage to extract odometry from the current ROS topics on the Go1. It's probably available via lcm. As a temporary fix,
the go1_odom package reverse engineers the `tf` topic to publish some odom messages with pose only.

## Printing April Tags
We use the 36h11 family of April tags. A script to generate all tag pdfs can be found in the `tag_pdfs` directory.
- Tags from 0 to 99 are used for static objects, like boards.
- Tags from 100 to 200 are used for dynamic objects, like obstacles.

## URDFs
Completely optional, but 3D models are nice. The current xml hardcodes the home directory so loading meshes in new
installs will fail. To solve this, run the following

```shell
rosrun tagslam_viz make_tags.py --file ~/tagslam_root/src/tagslam/example/tagslam_mrss.yaml --mesh_dir ~/tagslam_root/src/tagslam_viz/tags/36_11
```
and update the tag links (be careful not to delete camera and goal links) in the `src/tagslam/example/urdf/camera.urdf` file. This is also where you can define additional 3D models.



## To-Dos
Some non-blocking to-dos:
- There's no link between the odom and map trees. Maybe figure out one to have the pose of all the robot components int the map frame?
- Fix the path problem in URDFs
- Make more obstacles.
- Image compressed.