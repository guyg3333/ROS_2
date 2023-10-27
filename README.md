# Streaming Video over Ros2 node Pub Sub 

This repo show an example of how to stream video with Gstreamer from Live source 
in this Project there are two ros2 node video_sub and video_pub 

### to run the example

#### Install the Repo 
1. Install Gstreamer on Ubuntu or Debian : 
```
apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl
```
2. Install ROS2 : [see full tutorial on setup ROS2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
```
sudo apt install ros-humble-base
sudo apt install ros-dev-tools
```
3. Install BOOST 
```
sudo apt install libboost-all-dev
```
4. clone the repo 
5. enter to repo folder 
6. run commands :
```
rosdep install -i --from-path src --rosdistro humble -y
colcon build
source ./install/setup.bash
``` 

#### Run the example 
1. connect USB camera to device ( currently the example works with live inputs )
2. run the Publisher node 
```
source ./install/setup.bash
ros2 run video_pub video_pub
```
3. open new terminal and run the Subscriber node :
```
source ./install/setup.bash
ros2 run video_sub video_sub 
```


### System 

OS : ubuntu 22.04
ROS relese : humble

<img title="a title" alt="Alt text" src="./Image.png">

