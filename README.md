# Red Light Green Light Robot
The primary problem addressed by our project is robotic follower behavior i.e. maintaining distance from a moving target. It consists of three components: following the target, object detection/avoidance, and stopping when detecting the red/stop symbol.


## COSC 281 Group 8
* Angus Emmett
* Isaac Spokes
* Joseph Hajjar
* William Romano

## How to Run the Code
1. Turn on the Husarion RosBot 2.0, and SSH into it
2. Follow the instructions to launch the core and then launch the sensors
3. Run the `calibrate.py` file to calibrate the sensor to your tag color
4. Open up 2 terminals 
5. In the first, run the `vision.py` file
6. In the second, run the `driver.py` file
7. Next, use the red/green tags to make the robot stop/follow