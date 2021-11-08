# DIVISION OF LABOUR
## TASKS

* Setting up Stucture and Nodes for each task 
	* ```main.py```
	* ```driving.py```
	* ```vision.py``` 

* Vision System 
	* Set up camera feed node (figure it out) 
	* FSM = see_person, no_person, see_face 
	* Person Detection HAAR Cascades - Output (angle from center, distace)
	* Face Detection HAAR Cascades - Output (true/false)
		* Distance calculation with depth data and bounding box
	
* Driving System
	* Boiler plate from assignments
	* FSM = move, lost, avoid
	* PID when not avoiding obstacles
	* Lost Mode
	* Local Planning
	* Obstacle Avoidance (Cherry on top)
		* Bug 2 algo