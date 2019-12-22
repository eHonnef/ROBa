# ROBa

Project for the robotics course. This project aims to implement base nodes (listeners/publishers) for arduino mega sensors (SRF08 sonars and MinIMU-9) and the Asus xtionPro live camera. The nodes will run on Odroid XU4 with Ubuntu 18.04 and ROS melodic.  
There are standalone libraries for the `SRF08 sonar` and `MinIMU-9` sensors (`LSM6` and `LIS3MDL`). You just need to download and include in your main file.

## Image processing

Using Asus xtionPRO live for the RGB image and depth image.  
If you are running on windows remove the path from `openni2.initialize("/usr/lib/")` ---> `openni2.initialize()`.

### Libraries needed

To run the code you need to install de `requirements.txt` file using `pip install -r requirements.txt`.  
Of course you need to install an external library `openni2` that you [can find here](https://structure.io/openni).  
Let me know if I missed some requirements.

## Arduino schematics

![circuit](./circuit.svg)
P.S: IC = MinIMU-9.

## ROS

Ok. This one will be tricky:

- First make sure to install the `ROS-melodic-desktop` (version that I'm using on Ubuntu 18.04), [you can find a tutorial here](http://wiki.ros.org/melodic/Installation/Ubuntu).
- Install the `python3` libraries from the previous topic.
  - Try to run one of the examples from `./image_processing`, if you run you're good to go on the python side (I hope).
- Install the [openni2_launch](http://wiki.ros.org/openni2_launch) package.
- Install the Arduino IDE and necessary libraries, [tutorial](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup).
  - If you want to know more about [Arduino + ROS workpace](http://wiki.ros.org/rosserial_arduino/Tutorials/CMake)
- Make both python node executable with `chmod +x node.py`, usually they are located in `workspace/src/SOME_PACKAGE/scripts`.
- It'll be nice to have `tmux, screen` or something similar.

Now to start the workspace navigate to `./ros_workspace` and run `catkin_make` and then `source devel/setup.bash`.  

Upload the Arduino code `catkin_make arduino_sensors_firmware_arduino_sensors-upload`.  

Start the ROS-nodes:

- Start `roscore`.
- Camera topics `roslaunch openni2_launch openni2.launch`.
- Arduino node `rosrun rosserial_python serial_node.py /dev/ttyACM0`.
- Camera nodes `rosrun camera_rgb_img node.py` and `rosrun camera_rgb_img node.py`.
