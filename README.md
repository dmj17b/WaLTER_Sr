# Installation:
## ROS2 Installation:
Follow documentation for ROS2 Humble to install ROS2 and associated development packages:
https://docs.ros.org/en/humble/Installation.html

## Repository Installation:
Clone git repository 
```git clone https://github.com/dmj17b/WaLTER_Sr```

### Git Submodules:
The ROS2 ODrive package has been forked and included as a git submodule so that it can be edited with any changes we need. The .gitmodules file includes the location and link to this forked repository.

To intialize and set up the git submodule, navigate to the main working directory (/WaLTER_Sr) and run:

```git submodule init```

```git submodule update```

### Sourcing the Underlay:
In order to 'activate' ros2 commands in the terminal, you must source the underlay. For Linux systems, this is typically found in /opt/ros/humble.

```source /opt/ros/humble/setup.bash```

In practice, this command is typically added to the .bashrc file so that the underlay is always sourced when you open a new terminal window.

### Building the Repository
When the package is first installed or any changes are made, the repository must be built by navigating to the main directory and running:

```colcon build```

Since most of the code that is being altered is Python code (not compiled) we can use the ```--symlink-install``` extension to ensure that changes to Python code are included at run-time. This allows us to simply run our python files after making edits, rather than rebuilding the code base.

```colcon build --symlink-install```

### Sourcing the Overlay
Once the repository has been built, you should notice the addition of three new folders (build, install, log). Now that these are here, we can source the OVERLAY. This must be done in every new terminal window when working on the code base. In practice this can also be included in the .bashrc script if no other ros2 repositories are being run on the computer. Otherwise it is best to source the overlay manually to avoid confusion and naming collisions.


# Code Architecture
WaLTER's current architecture involves the following nodes:
- joy_node,
- wheel_ctrl_node,
- fr_hip,
- fr_knee,
- fl_hip,
- fl_knee,
- rr_hip,
- rr_knee,
- rl_hip,
- rl_knee,
- main_ctrl_node,

## Joy Node
The ```joy_node``` is a built-in ROS2 node that detects joystick inputs and publishes them to the ```/joy`` topic.

To ensure that this node is working properly, you can run ```ros2 run joy joy_node```, then in a separate terminal ```ros2 topic echo /joy```

