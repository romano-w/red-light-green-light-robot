# DIVISION OF LABOUR
## TASKS

* Setting up Stucture and Nodes for each task 
	* ```main.py```
	* ```driving.py```
	* ```vision.py``` 

* Vision System 
	* Set up camera feed node (figure it out) 
	* FSM = see_person, no_person, see_face 
	* Person Detection HAAR Cascades - Output (angle from center, distance)
	* Face Detection HAAR Cascades - Output (true/false)
		* Distance calculation with depth data and bounding box
	
* Driving System
	* Boiler plate from assignments
	* FSM = move, lost, avoid
	* PD when not avoiding obstacles
		* No Integral Gain term because the moving target means past errors are less likely to affect the robot's motion.
	* Lost Mode
	* Local Planning
	* Obstacle Avoidance (Cherry on top)
		* Bug 2 algo