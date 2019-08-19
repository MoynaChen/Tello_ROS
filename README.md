# tello_driver

**Thanks for the author@anqixu/https://github.com/anqixu/TelloPy.git
ROS driver wrapper for DJI/Ryze Tello drone

#This package can control the Tello drone via keyboard, in addition we can see video through Tello's camera.

*keyboard control
> cd ~/catkin_ws/src
> roscore 
> #open another Terminal 
> rosrun tello_driver tello_control.py
> #open a new Terminal 
> rosrun tello_driver keys_publish.py

*video
> roslaunch tello_driver tello_node.launch
> #open a new Terminal
> rosrun rqt_image_view rqt_image_view /tello/image_raw/compressed