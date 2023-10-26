# Unified Teleop (NUMSR)
ROS interface for joy messages from various types of control devices to do robot teleoperation with different control schemes.

## Requirements
For whichever control device you intend to use (video game controller, SpaceNav, etc.), the joy package for that device will be required.

## Key Concepts
For this package to be used, it requires three components:
1. `Input Mapping`  - Contains the names of each of the control device's inputs and their respective index numbers in a /joy message (found in the /config folder, refer to example_mapping.yaml for detailed example).
2. `Control Scheme` - Contains the names of the parameters of a joystick node, each representing an in-use control function, and the name of the control device's input that will control said functions. These input names must match their counterpart in Input Mapping (the control scheme should be created in your robot teleoperation project package, but an example called example_scheme.yaml can be found in the /config folder).
3. `Control Node`   - The node that takes in the Input Mapping and Control Scheme .yaml files, and receives /joy messages to output the desired teleoperation message output (these can be foudn in the /src folder).

## Control Node Overview
1. `cmd_vel_mirror_joystick`        - When provided a control scheme and input mapping, publishes Twist messages where the values mirror the user's inputs (e.g. the velocity values will mirror how you push the joystick).
2. `point_stamped_incr_joystick`    - When provided a control scheme and input mapping, publishes PointStamped messages where the values increment based on the user's inputs (e.g. the positional values will increase the longer you press on the button).
3. `point_stamped_mirror_joystick`  - When provided a control scheme and input mapping, publishes PointStamped messages where the values mirror the user's inputs (e.g. the positional values will mirror how you push the joystick).
4. `twist_merger`                   - Takes in Twist messages from two different topics, merging the linear variable values from one with the angular variable values from the other to output a combined Twist message (for those looking to merge multiple control schemes or nodes).

## Common Demo

To make a single robot "walk the dog" use `ros2 launch omnid_control single_robot_float.launch robot_id:=X` where `X` is the numerical ID of the robot.
By default, this launch file will enable the gimbal window so the robot doesn't move unless the gimbal is within some tolerance of upright. To disable this behavior, use the option `enable_gimbal_window:=false` with caution.

To make three robots float at once with an articulated or otherwise unmodeled payload, use `ros2 launch omnid_control three_robot_float_articulated.launch`.
This will enable virtual impedances in the Z direction to compensate for shifting payload center of mass.

To visualize the live joint state of the omnid during these demos use `ros2 launch omnid_control rviz_single_robot.launch robot_id:=1|2|3`, this will require the omnid_description and omnid_delta_state packages.

If you would like to visualize the current state of the robot without running any of the above demos, run `ros2 launch omnid_control interface_demo.launch robot_id:=3` and then `ros2 launch omnid_control rviz_single_robot.launch robot_id:=1|2|3`.

To make three robots float at once with the provided PVC pipe fork payload, use `ros2 launch omnid_control three_robot_float_rigidbody.launch`. This will
enable the hardcoded center-of-mass model for the PVC pipe fork payload, with no virtual impedances by default.

To make the delta robot move through a sequence of positions, use `ros2 launch omnid_control delta_position_demo.launch robot_id:=X` where `X` is the numerical
ID of the robot. Default tunings are intended for a single delta robot without a payload.

## Single Omnid Teleoperation w/ Seperate Input Device
### CLI Commands
To control a single omnid - the mobile base and delta - with an input device of your choosing (e.g. PS4 controller) use `ros2 launch omnid_control single_robot_teleop_station.launch robot_id:=X control_scheme:=Y` on the base PC and use `ros2 launch omnid_control single_robot_teleop_omnid.launch robot_id:=X` on the PC of the omnid you wish to control, where `X` is the numerical ID of the robot and `Y` is the chosen .yaml file name of the control scheme you wish to use - you can find these in the config folder.
By default, you will be able to use the the controller the move the omnid around and control the position of the delta arms.

If you would like to only control the mobile base of the omnid, use `ros2 launch omnid_control mobile_teleop.launch robot_id:=X control_scheme:=Y` on the base PC and use `ros2 launch omnid_control mobile_interface.launch robot_id:=X` on the PC of the omnid you wish to control.

If you would like to only control the delta of the omnid, use `ros2 launch omnid_control delta_teleop.launch robot_id:=X control_scheme:=Y` on the base PC and use `ros2 launch omnid_control delta_interface.launch robot_id:=X` on the PC of the omnid you wish to control.

### Teleop Control Schemes
There are different control schemes that the user can choose to control the omnid with, as well as create their own custom control schemes.
The different control schemes consist of yaml files can be found in the `config` directory.
To view an example control scheme, refer to `config/example_scheme.yaml` for a documented example that should contain all the information you need to develop your own control scheme.

## Mobile Teleoperation
To drive a robot, use the following command:

`roslaunch omnid_control mobile_interface.launch robot_id:=X`

where X is the robot number (e.g. 1 for omnid1). The robot will now subscribe to /omnidX/cmd_vel for motion commands.

The /cmd/vel topic should be published at least 10 times per second. The mobile_interface node will automatically stop the robot
if a message doesn't appear for 0.1 seconds.

To drive the robot from a keyboard, use [teleop_twist_keyboard](https://github.com/ros-teleop/teleop_twist_keyboard). This command will run the keyboard node and remap its output to a robot, repeating the command at 100Hz and sending a zero-twist command if no key is pressed for 0.6 seconds.

`rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=omnidX/cmd_vel _key_timeout=0.6 _repeat_rate:=100`

Because of the nature of teleop_twist_keyboard, the teleop delay needs to be longer than the OS key repeat delay (0.5s by default on Ubuntu). A delay of 0.6 seconds is rather long, so it may be desirable to reduce the delay to around 0.2s:

`gsettings set org.gnome.desktop.peripherals.keyboard delay 200`

Once this is done, the teleop command can be edited to use a shorter timeout that is just barely longer than the new, shortened key repeat delay.

`rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=omnidX/cmd_vel _key_timeout=0.21 _repeat_rate:=100`

