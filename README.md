# ROS driver for the Arbotix Commander 2 joystick

This driver interfaces the Arbotix Commander 2 to ROS. Plug in the Xbee dongle paired to the Commander and let the node know which serial port it is on, and the node will publish sensor_msgs::Joy messages.

This driver was written for the Arbotix Commander version 2 from Trossen Robotics, but wil probably work with the original from Vanadium Labs as well.
