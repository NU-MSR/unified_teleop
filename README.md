# Unified Teleop (NUMSR)
ROS interface for joy messages from various types of control devices to do robot teleoperation with different control schemes.
This package offers an intuitive method of creating customize control schemes, and allows for the use of a variety of input devices by creating your own input mappings.

## Requirements
For whichever control device you intend to use (video game controller, SpaceNav, etc.), the joy package for that device will be required.

## Key Concepts
For this package to be used, it requires three components:
1. `Input Mapping`  - Contains the names of each of the control device's inputs and their respective index numbers in a /joy message (found in the /config folder, refer to example_mapping.yaml for detailed example).
Mappings for the Dualshock 3 and 4 controllers, and SpaceNav joystick have been included with this package, and you can create your own.
2. `Control Scheme` - Contains the names of the parameters of a joystick node, each representing an in-use control function, and the name of the control device's input that will control said functions. These input names must match their counterpart in Input Mapping (the control scheme should be created in your robot teleoperation project package, but an example called example_scheme.yaml can be found in the /config folder).
3. `Control Node`   - The node that takes in the Input Mapping and Control Scheme .yaml files, and receives /joy messages to output the desired teleoperation message output (these can be foudn in the /src folder).

## Control Node Overview
1. `cmd_vel_mirror_joystick`        - When provided a control scheme and input mapping, publishes Twist messages where the values mirror the user's inputs (e.g. the velocity values will mirror how you push the joystick).
2. `point_stamped_incr_joystick`    - When provided a control scheme and input mapping, publishes PointStamped messages where the values increment based on the user's inputs (e.g. the positional values will increase the longer you press on the button).
3. `point_stamped_mirror_joystick`  - When provided a control scheme and input mapping, publishes PointStamped messages where the values mirror the user's inputs (e.g. the positional values will mirror how you push the joystick).
4. `twist_merger`                   - Takes in Twist messages from two different topics, merging the linear variable values from one with the angular variable values from the other to output a combined Twist message (for those looking to merge multiple control schemes or nodes).

## Common Demo
To control the `turtlesim` with the `cmd_vel_mirror_joystick`, the demo `demo_dualshock4_turtlesim.launch` has been included to demonstrate an example on how to utilize the Unified Teleop package for controlling a robot, including the usage of Control Schemes and Input Mappings.

Run `ros2 launch unified_teleop demo_dualshock4_turtlesim.launch.xml` and refer to `example_scheme.yaml` on how to control the `turtlesim`.