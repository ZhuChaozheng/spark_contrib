# face_recognizer
this function is powered by opencv. Also, you should start it with Opencv3. Attention: OpenCV3 must be configured or it will not be executed successfully. (if you are using ROS kinetic, it is configured by opencv3 in the default version.)
## Get Started
```Python3
cd ~/catkin_ws/src
git clone https://github.com/ZhuChaozheng/face_recognizer
cd ~/catkin_ws
catkin_make
roslaunch face_recognizer face_recognizer.launch 
