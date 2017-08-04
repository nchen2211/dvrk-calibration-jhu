DATA COLLECTION
Collecting Torque Offset Data
on the terminal type in:
	python force_joint_space_torque_offset.py PSM3

read this: 
- PSM3 is the name of the arm you are testing
- There is a constant called CONST_JOINT_UDER_TESTING. Change this to either 0 or 1 depending 
  which joint you want to test
- Once you run the code, there are 2 options that you can choose:
	1. "collect large range torque offset". Select this option if you want to collect torque offset as one of the parameters of the Compliance model.
		-  The data generated will be called "torque_offset_output_joint[0 or 1]_[date-time]"
	2. "collect small range torque offset". Select this option if you want to collect torque offset data that will be used to subtract current joint effort after you collect hysteresis data for finding stiffness or backlash parameter. NOTE: don't forget to uncomment line 120 and comment out line 119 on force_joint_space_torque_offset.py
		- The data generated will be called "small_torque_offset_output_joint[0 or 1]_[date-time]"



Collecting Stiffness and Backlash Data
on the terminal type in:
	python force_joint_space_backlash.py PSM3

read this in: 
- PSM3 is the name of the arm you are testing
- There is a constant called CONST_JOINT_UDER_TESTING. Change this to either 0 or 1 depending 
  which joint you want to test
- Once you run the code, follow the instruction.
- The data generated will be called "hysteresis_output_joint[0 or 1]_[date-time]"


GRAPHING

Graph Torque Offset Data
on the terminal type:
	python force_joint_space_graph.py

read this:
- Select the option 1. It says "for plotting torque offset"





Graphing Hysteresis
on the terminal type in:
	python force_joint_space_graph.py

read this:
- To graph hyseresis, you need collect the data from the clamping method first. The filename generated from the clamping method is "hysteresis_output_joint[0 or 1]_[date-time]"
- On the terminal type:
	python force_joint_space_graph.py
- Select option 2. It says "for plotting hysteresis"





Graphing Hysteresis Best-Fit Lines, Stiffness, and Backlash Slope
- To graph them, you need 2 files start with "hysteresis_processed"

read this:
If this is the first time you collected stiffness and/or backlash data:
- To generate a new hysteresis processed data using the new hystersis output data: 
	1. On the terminal type: python force_joint_space_graph.py
	2. Select option 4 "for generating hysteresis processed data"
	3. Notice that the new files of hysteresis_processed were generated and saved under the directory /catkin_ws/src/dvrk-calibration-jhu/JointForceTesting/ForceTestingDataJointSpace


	Graph Hysteresis Best-Fit Lines:
	on the terminal type in:
		python force_joint_space_graph.py

	read this:
	- On the terminal type in:
		python force_joint_space_graph.py
	- Select option 3. It says "for plotting hsyteresis best-fit lines"



	Graph Stiffness:
	on the terminal type in:
		python force_joint_space_graph.py

	read this:
	- On the terminal type in:
		python force_joint_space_graph.py
	- Select option 5. It says "for plotting stiffness and its slope"




	Graph Backlash Slope:
	on the terminal type in:
		python force_joint_space_graph.py

	read this:
	- On the terminal type in:
		python force_joint_space_graph.py
	- Select option 6. It says "for plotting backlash slope"





ACCURACY IMPROVEMENT TEST
Experiment 1: Cartesian Positions Experiment
- Run the dvrk console falcon to turn on the Falcon teleop.
- Position the metal plate in a way that all the points can be reached by the robot without moving the plate and the robot joint setup. The only robot joints that are allowed to move is joint 1,2, and 3 only.
- Open a terminal, get ready to record the data using rosbag
- Everytime the tooltop lands on a point, type in: rosbag record /dvrk/PSM3/state_joint_current /dvrk/PSM3/position_cartesian_current /dvrk/PSM3_compensated/state_joint_current /dvrk/PSM3_compensated/position_cartesian_current
- The .bag data will be stored at /Home
- For .bag data generated on point x, move it to directory /catkin_ws/src/dvrk-calibration-jhu/JointForceTesting/CorrectionData/DistanceMeasurement/Point_x
- Now we need to convert .bag data to .csv separated by each topic. To do that, type in on the terminal:
	for topic in `rostopic list -b [rosbag_filename].bag` ; do rostopic echo -p -b [rosbag_filename].bag $topic >${topic//\//_}.csv ; done
- On the terminal type in:
	python force_joint_space_graph_compensation.py
- Select option 3. It says "for generating cartesian of plate distance"
- 2 output files called "points_cartesian_compensated" and "points_cartesian_encoder.txt" are generated and stored under /catkin_ws/src/dvrk-calibration-jhu/JointForceTesting/CorrectionData/


Experiment 2: Joint Position Displacement Experiment
- Run the dvrk console falcon to turn on the Falcon teleop.
- Pick 1 point, move the tooltip inside the point. Determines an arbitrary max effort (I used 1.0, 1.2, 1.5, and 1.7). Pick a direction (0', 90', 180', or 270'). 
- Run rosbag, on the terminal type in:
	rosbag record /dvrk/PSM3/state_joint_current /dvrk/PSM3_compensated/state_joint_current 
- Apply force slowly to that direction until the effort of that joint reaches the max effort.
- The .bag data will be stored at /Home
- For .bag data generated on point x with max effort y, move it to directory /catkin_ws/src/dvrk-calibration-jhu/JointForceTesting/CorrectionData/JointDisplacementTest/Point_x/[max effort y]
- Now we need to convert .bag data to .csv separated by each topic. To do that, type in on the terminal:
	for topic in `rostopic list -b [rosbag_filename].bag` ; do rostopic echo -p -b [rosbag_filename].bag $topic >${topic//\//_}.csv ; done
- On the terminal type in:
	python force_joint_space_graph_compensation.py
- Select option 1. It says "for generating comparison file".
- Open force_joint_space_graph_compensation.py on a text editor. There are 4 constant values declared and initialized near the top: JOINT, MAX_EFFORT, POINT, ANGLE. Modify them depending on your experiment. 
- On the terminal type in:
	python force_joint_space_graph_compensation.py
- Select option 2. It says "for plotting compensation comparison graph".
















