roadside_unit
====
This project is part of my master thesis: "Implementation of a System to Infrastructure System for Autonomous Vehicles"

Set up GitHub
------
https://docs.github.com/en/get-started
```
git config --global user.name "PedroSoler10"
git config --global user.email "soler.pedrojavier@gmail.com"
```
SSH authentication:

https://docs.github.com/en/authentication/connecting-to-github-with-ssh/about-ssh

https://github.com/settings/keys

```
ssh-keygen -t ed25519 -C "soler.pedrojavier@gmail.com"
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/max_laptop
cat ~/.ssh/max_laptop.pub
```

Clone repository:
```
git clone git@github.com:PedroSoler10/roadside_unit.git
```

Create a catkin_workspace
----
http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
```
mkdir -p catkin_workspace/src
cd catkin_workspace
catkin_make
```

Create a ROS package
----
http://wiki.ros.org/ROS/Tutorials/CreatingPackage


Publish camera images with usb_cam
----
http://wiki.ros.org/usb_cam

Camera calibration
----
https://www.youtube.com/watch?v=S-UHiFsn-GI&list=PL2zRqk16wsdoCCLpou-dGo7QQNks1Ppzo


Calibrate the camera with camera_calibration
----
http://wiki.ros.org/camera_calibration
```
rosdep install camera_calibration
rostopic list
  /camera/camera_info
  /camera/image_raw
rosrun camera_calibration cameracalibrator.py --size 9x7 --square 0.020 image:=/usb_cam/image_raw camera:=/usb_cam
```
usb_cam already uses a default calibration data: file:///home/pedro/.ros/camera_info/head_camera.yaml

The file created by camera_calibration, ost.yaml, is really similar to the default one of usb_cam

Architecture
----
![rosgraph](https://github.com/PedroSoler10/v2i_system/assets/74536059/22992f37-c4aa-4dd4-9ff1-adcd110a5c4b)


usb_cam-test.launch
----
```
roslaunch ./launch/usb_cam-test.launch video_device:=/dev/video2 image_view:=true image_view_topic:=/tennis_ball_image
```

perception_pkg
----
```
rosrun perception_pkg tennis_ball_tracking.py
```
```
rosrun perception_pkg scenario_classification.py
```

debugging_pkg
----
```
rosrun debugging_pkg visualizer.py
```
YOLO Ultralytics
----

https://github.com/ultralytics

https://docs.ultralytics.com/quickstart/

darkent_ros
----
https://github.com/leggedrobotics/darknet_ros/tree/master

    cd catkin_workspace/src
    git clone --recursive git@github.com:leggedrobotics/darknet_ros.git
    cd ../
    catkin_make -DCMAKE_BUILD_TYPE=Release
    gedit src/darknet_ros/darknet_ros/config/ros.yaml
Change the predefined topic names to the proyect ones:

  /camera/rgb/image_raw --> /usb_cam/image_raw
    
    source devel/setup.bash
    roslaunch darknet_ros darknet_ros.launch

Connecting two devices via WLAN and ROS
----
Server device (where roscore will be running):
```
export ROS_MASTER_URI=http://10.42.0.85:11311
export ROS_HOSTNAME=10.42.0.85
```
Client device:
```
export ROS_MASTER_URI=http://10.42.0.85:11311
export ROS_HOSTNAME=10.42.0.1
```
The IPs shown correspond to the HP (85) and Max (1) laptops when connected to Max's hotspot.



Manual
----
Just the first time:
```
cd
git clone git@github.com:PedroSoler10/roadside_unit.git
cd roadside_unit
chmod +x src/perception_pkg/src/tennis_ball_tracking.py
chmod +x src/perception_pkg/src/scenario_classification.py
chmod +x src/debugging_pkg/src/visualizer.py
gedit src/darknet_ros/darknet_ros/config/ros.yaml
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
roslaunch ./launch/usb_cam-test.launch video_device:=/dev/video2 image_view:=true image_view_topic:=/tennis_ball_image
roslaunch darknet_ros darknet_ros.launch
rosrun perception_pkg tennis_ball_tracking.py
rosrun perception_pkg scenario_classification.py
rosrun debugging_pkg visualizer.py
```
Every time the code is modified
```
catkin_make
source devel/setup.bash
roslaunch ./launch/usb_cam-test.launch video_device:=/dev/video2 image_view:=true image_view_topic:=/tennis_ball_image
roslaunch darknet_ros darknet_ros.launch
rosrun perception_pkg tennis_ball_tracking.py
rosrun perception_pkg scenario_classification.py
rosrun debugging_pkg visualizer.py
```

Issues
-----
opencv instead of opencv4
----
CMake Error at /opt/ros/melodic/share/cv_bridge/cmake/cv_bridgeConfig.cmake:113 (message):
  Project 'cv_bridge' specifies '/usr/include/opencv' as an include dir,
  which is not found.  It does neither exist as an absolute directory nor in
  '${{prefix}}//usr/include/opencv'.  Check the issue tracker
  'https://github.com/ros-perception/vision_opencv/issues' and consider
  creating a ticket if the problem has not been reported yet.
https://github.com/ros-perception/vision_opencv/issues/389

Had the same issue, the above fixed my compiler needs.
Be ware of compatibility with yours, you may need to delete the link afterwards to not break other cmake runs that use/expect other versions of opencv.

"Problem" is the static / hardcoded libraries in cv_bridge cmake file.

```
cd /usr/include
sudo ln -s opencv4/ opencv
```

ssh
----
Both devices should be connected to a common network like the laptop's hotspot.

TX2:

For the first time:

```
sudo apt-get install openssh-server
sudo systemctl status ssh
sudo systemctl start ssh
ip addr show
```
Configure the WiFi connection to the laptop hotspot to:

Automatically connect to this network when it is available

All users may connect to this network

Laptop:

Turn on the hotspot and log in via ssh with the following command and passwort:
```
ssh tx2@10.42.0.47
tx2
```
