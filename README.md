RobotVisionControl
==================

This is a series of programs that work together to allow a user to control an ICreate robot via a Kinnect camera. The user connects the kinnect to thier computer, then they run lab2.py, which takes the image stream and picks out a specific colored object defined by the user (put a colorful piece of paper on top of the robot and calibrate this program to find only these colors). Then the user runs a file called kbd_publisher, which gives us some keyboard commands to send to the robot. This allows us to switch between manual command mode and visual command mode. Finally run robot1.py and this will allow the robot to control and compute its heading and target vector. 
