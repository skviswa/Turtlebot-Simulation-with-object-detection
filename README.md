This is a Turtlebot simulation that implements path planning towards a specific goal.
YOLO framework running on Webcam is integrated with this system and it provides information
to the turtlebot whenever there are people appearing on the webcam scene.
The turtlebot stops its movement in such a scenario, and resumes its movement to the specified
goal when the scene is cleared of people, simulating an autonomous vehicle movement with
a pedestrian detection mechanism.

Project by Karthik Viswanathan, Ram Prakash and Shruthi Kulkarni.

We assume you have ROS Kinetic, CUDA (version 8.0 or above), cuDNN (version 5.0 or above) and
OpenCV (version 2.4 or above) and Gazebo installed.

To compile the project, add the darknet ros and usb_cam packages to your catkin workspace.
It is recommened to test this on an isolated catkin workspace.

Do:

mkdir -p test_ws/src
cd test_ws
catkin_make
source devel/setup.bash
Copy the darknet_ros and usb_cam packages to the src directory under test_ws.
The makefile for darknet framework under darknet_ros has GPU,cuDNN and OpenCV enabled by default.
Modify the lines 26,27 of single_image_test.cpp and lines 27,28 of yolo_ross.cpp 
to point to the correct path to the config and weight files in your directories.

Then do 
catkin_make --pkg usb_cam
catkin_make --pkg darknet_ros
source devel/setup.bash

Alternatively, you can follow the procedure from the following reference links we consulted to get this system setup to be integrated with ROS:

https://github.com/pgigioli/darknet_ros
http://wiki.ros.org/usb_cam

in the test_ws, do:
roscore
Open a new terminal, and do:
roslaunch darknet_ros yolo_ros.launch
This should launch the darknet framework on ros 

This produces 2 topics:

/found_object - displays "1" or "0" corresponding to whether or not an object has been detected

/YOLO_bboxes - displays the class label that was detected followed by the bbox coordinates [xmin, ymin, xmax, ymax].

On a new terminal in the test_ws workspace, do
rostopic list 
rostopic echo /found_object
rostopic echo /YOLO_bboxes
to find the output results of the Darknet framework.

Once this procedure is done, we can move

To initialze turtlebot simulator, 
Open new terminals, do
roslaunch turtlebot_gazebo turtlebot_world.launch
roslaunch turtlebot_navigation amcl_demo.launch

This should bring up the turtlebot in a default map environment.
Modify lines 23 and 24 and provide proper x,y co-ordinate for the destination.
We have it at default 0,0 position. Try to limit the range of values to a magnitude
of 3 for either co-ordinate so that the destination is within the map.
Then run 
python move_and_avoid.py
