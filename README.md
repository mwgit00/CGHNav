# CGHNav

*Experiments with the Generalized Hough Transform and LIDAR.*

This repo has a simple simulation of a 2-wheel differential drive robot
with a LIDAR.  It can roll around in a room based on user key presses.

The Generalized Hough Transform is used to match the robot's current LIDAR scan
with a scan taken previously at a waypoint.  A template is generated for a range
of angles about each waypoint.  Typically there are 360 templates; 1 for each
degree in the range 0 to 359.  The robot searches the templates to determine
a best guess for its current orientation and translation from the waypoint.

This code could be the "front end" for a SLAM algorithm.

The code has been tested with OpenCV 4.3.0 on a Windows 10 machine with
Community Edition of Visual Studio 2019.

Click the image to see a YouTube demo video:

[![CGHNav Demo Video](http://img.youtube.com/vi/jNoWSvQfcbc/0.jpg)](http://www.youtube.com/watch?v=jNoWSvQfcbc)
