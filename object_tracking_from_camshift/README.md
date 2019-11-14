## object_tracking_from_camshift
this code is powered by opencv, and also we merge it with ROS.

### Get Started
```python3
cd ~/catkin_ws/src
git clone https://github.com/ZhuChaozheng/object_tracking_from_camshift
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
rosrun object_tracking_from_camshift camshift_ros.py
