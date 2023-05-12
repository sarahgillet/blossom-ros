# blossom-ros

## Run roscore:
roscore 

## Running the robot:
python3 commander.py

## Running the example sequence publisher:
python3 test_pub.py

# Running the position tracker
rosrun blossom_ros position_tracker.py

For more convenient starting of a kinect azure camera and the object tracker use:
roslaunch blossom_ros launch_blossom_kinect.launch

launch Blossom commander through
roslaunch blossom_ros launch_blossom.launch

in the launch_blossom.launch file one can change the config file for the specific robot used.

## Sending object positions to Blossom so it can look at them
See ['test_object_person_tracking.py'](https://github.com/sarahgillet/blossom-ros/blob/main/scripts/test_object_person_tracking.py) for example code on how to send object positions and how to change the mode of the trajectory/position tracker. More information on the different modes can be found below.


### Defining new object positions
The coordinate system of 'base_link' is defined as follows: 
- x (red): Forward from the robot's 0 position on the base
- y (green): Sideways out, positive to the left from the robot's point of view
- z (blue): Pointing upward to the robot's head
The 'base_link' is located in the very bottom plate of the robot.

<img src="https://github.com/sarahgillet/blossom-ros/assets/65712056/853bd816-d4fc-4e44-840f-55fcb7be5599" width="300"/>

To define new objects, you can either define your object positions in the coordinate system of the 'base_link' or define a new coordinate system and publish the transform between 'base_link' and your new coodinate system.

## Modes of the position tracker
Currently the position tracker has five allowed different modes: 'follow_person', 'look_between', 'follow_person_base_only', 'look_at_object', 'track_object'.
- 'follow_person': moves base and head to 'look' at the person (input: kinect body tracking)
- 'look_between': Not yet implemented, does the same as 'follow_person'
- 'follow_person_base_only': moves base to orient toward the person (input: kinect body tracking)
- 'look_at_object': moves base and head to orient toward the given position (input: PositionStamped on topic /object_pose_tracker)
- 'track_object': No special implementation yet, does the same as 'look_at_object'

The mode can be changed by sending a std_msgs/String with the mode name to the '/mode' topic.
