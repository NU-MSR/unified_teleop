# Unified Teleop (NUMSR)
Unified Teleop is a versatile ROS package designed for seamless robot teleoperation through various control devices. It enables users to craft tailored control schemes and interface with any device supported by a ROS joy package. This package is user-friendly, encouraging community contributions for extended device support.

## Features of Unified_teleop vs Teleop_twist_joy Packages
1. **Configurable Input Schemes for any device:** As long as the input control device of choice has a ROS joy package, users can create configurable Input Schemes to interface with said device (e.g. Xbox360 Controller, Dualshock4 Controller, SpaceNav Mouse).
Users are encouraged to make pull requests for their Input Schemes of new input control devices.
2. **Custom Output Schemes:** Users can create custom Output Schemes to control their robot in their preferred manner (e.g. moving the robot forwards using the left joystick, right joystick, or up button).
3. **Supports intuitive naming schemes:** As opposed to using confusing index numbers, users can utilize clear and meaningful naming schemes for describing control device inputs to simplify the Output Scheme creation process (e.g. enable_control: button_r1).
4. **Provides both Twist and PointStamped messages:** This package includes Teleop Nodes that can publish Twist or PointStamped messages to provide users with additional options for controlling their robots. More options will be added if there is high demmand for specific message types.
5. **Allows for different modes of control:** For PointStamped, there is the choice between Incremental and Mirror control modes to provided users additional options in how they control their robots.
    1. Mirror: The Teleop Node's ouput will mirror its inputs (e.g. the robot arm will more as far as the joystick is moved).
    2. Incremental: The Teleop Node's output will increments as its inputs are toggled (e.g. the robot arm will continue to move whilst the joystick is pushed).
6. **Apply modifiers to the teleop outputs:** Additional modifiers for the Teleop Node outputs have been applied to allow for additional customization in how a user would like to control their robot.
    1. Rate of Change: To smoothen a robot's movement and minimize erratic movements, users can implement a Rate of Change movement limit.
    2. Spherical Boundary Radius: For users who want their robot to move within a Spherical Boundary instead of the standard cuboid one.
    3. Offsets: To Offset the robot's zero position to suit the user's needs.

## Package Requirements
To utilize Unified Teleop with your chosen input device, such as a video game controller or SpaceNav, you must install the corresponding ROS joy package. Ensure these packages are active before operating your device. Here are some common examples:

1. **Game Controllers (e.g., Playstation Dualshock 4):** https://index.ros.org/p/joy/
2. **SpaceNav Controller:** https://index.ros.org/p/spacenav/

## Essential Concepts
To harness the full potential of Unified Teleop, understanding these core concepts is crucial:

1. **Input Scheme:** This is a configuration that maps the physical controls of your device to identifiable input names, along with their index numbers in a `/joy` message. You can find these configurations in the `/config` directory, with pre-configured mappings for Dualshock 3 and 4 controllers, as well as the SpaceNav joystick and an annotated example, included. Users are encouraged to create and contribute mappings for additional devices.
2. **Output Scheme:** This configuration specifies the associations between joystick node parameters and their corresponding functions, linked by the control device inputs named in the Input Scheme. While you should create Output Schemes within your own robot's package, an illustrative example in the `/example` directory is available for reference.
3. **Teleop Node:**  It is the operational core that processes Input and Output Schemes, interprets `/joy` messages, and produces the relevant teleoperation messages. The Teleop Nodes are located in the `/src` directory, ready to be tailored to your project's requirements.

These concepts work together to deliver a customizable and intuitive teleoperation experience, ensuring your commands are translated into precise robotic actions.
Illustrative examples for these concepts are available in the `/example` directory. With an example Input Scheme being in the `/config` directory.

## Overview of Teleop Nodes
1. **Twist Mirror Node (`twist_mirror`):**  Interprets your Output and Input Schemes to produce `Twist` messages that directly reflect the magnitude and direction of your control inputs. For example, the velocity output will correspond proportionally to how far the joystick is pushed.
2. **PointStamped Mirror Node (`point_stamped_mirror`):** Similar to the Twist Mirror Node, it publishes `PointStamped` messages that match the user's control actions, offering a mirrored, one-to-one mapping between joystick movement and positional output.
3. **PointStamped Incremental Node (`point_stamped_incr`):** This node, configured with your Output and Input Schemes, emits `PointStamped` messages where the position values incrementally adjust based on continuous user inputs. For instance, pressing and holding a button causes a gradual increase in the position value.
4. **Twist Merger Node (`twist_merger`):** A specialized node that fuses `Twist` messages from two distinct topics. It combines linear movements from one source with angular movements from another to output a cohesive `Twist` message. This functionality is particularly useful when you need to integrate multiple Output Schemes or control nodes for a unified motion command.

## Common Demos
### Examples
The `/example` directory contains annotated examples showcasing the essential features of the Unified Teleop package. With an example Input Scheme being in the `/config` directory.

### Demo Setup
To illustrate the package's capabilities, we've included the `demo_dualshock4.launch` file. This launch file sets up an environment for you to control the `turtlesim` node using `Twist` messages, showcasing the versatility of the Unified Teleop package with predefined Input and Output Schemes. Make sure to have the turtlesim package installed.

### Running the Demo
1. Start the `turtlesim` node by running:

        ros2 run turtlesim turtlesim_node

2. Launch the Unified Teleop demo with the following command:

        ros2 launch unified_teleop demo_dualshock4.launch.xml

3. Observe the command messages being published to the `turtle1/cmd_vel` topic:

        ros2 topic echo /turtle1/cmd_vel

Refer to the `example_scheme.yaml` file for detailed control instructions tailored for the Dualshock 4 controller within the demo.

### Customization and Experimentation
This demo encourages you to explore various configurations. Try adjusting the Output Scheme for different control styles or create new Input Schemes to use with alternative controllers. This flexibility allows you to customize the teleoperation experience to suit your project's needs or personal preferences. Install the turtlesim package (if not already installed) and use the provided launch file as a starting point for your teleoperation adventures.