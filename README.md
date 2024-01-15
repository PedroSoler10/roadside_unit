v2i_system
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
git clone git@github.com:PedroSoler10/camera_infrastructure.git
```

Create a catkin_workspace
----
http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

Create a ROS package
----
http://wiki.ros.org/ROS/Tutorials/CreatingPackage


Publish camera images with usb_cam
----
http://wiki.ros.org/usb_cam
usb_cam-test,launch modified
roslaunch ./launch/usb_cam-test.launch image_view:=true video_device:=/dev/video2

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

Tennis ball detector
----
