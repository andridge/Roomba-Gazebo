# Gazebo

<img width="1552" alt="Screenshot 2023-11-14 at 16 46 33" src="https://github.com/andridge/Roomba-Gazebo/assets/46260701/13751682-2b98-44da-86e6-0f7bf1bdfbf0">

## Readme File

This readme file provides the correct order of commands for executing a specific task.

## Instructions

1. Open a terminal.
2. Navigate to the root of the workspace.
3. Run the following commands in the given order:

```bash
roscore
4. Open another terminal.
catkin_make
source devel/setup.bash

chmod +x '/home/andre/Desktop/TaskB/src/com760_Suhael_Ahmed/src/scripts/controller.py'
chmod +x '/home/andre/Desktop/TaskB/src/com760_Suhael_Ahmed/src/scripts/Bug2.py'
chmod +x '/home/andre/Desktop/TaskB/src/com760_Suhael_Ahmed/src/scripts/Bug1.py'
chmod +x '/home/andre/Desktop/TaskB/src/com760_Suhael_Ahmed/src/scripts/Bug0.py'
chmod +x '/home/andre/Desktop/TaskB/src/com760_Suhael_Ahmed/src/scripts/GoToPoint.py'
chmod +x '/home/andre/Desktop/TaskB/src/com760_Suhael_Ahmed/src/scripts/FollowWall.py'
rostopic list
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/set_link_state
/gazebo/set_model_state
roslaunch com760_Suhael_Ahmed b00856266.launch
roslaunch com760_Suhael_Ahmed b00856266.launch algo:=BUG0
roslaunch com760_Suhael_Ahmed b00856266.launch algo:=BUG1
roslaunch com760_Suhael_Ahmed b00856266.launch algo:=BUG2
Note: Please make sure to replace /home/andre/Desktop/TaskB with the actual path to your workspace directory.

Contact Information

If you have any questions or need further assistance, feel free to contact me:

Name: Andre
Email: munenengarw@gmail.com

Thank you!
