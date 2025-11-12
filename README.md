# Installation:
MAIN README
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

# BOOM AND DATA LOGGING

## Connecting to the boom
1. Sign into boom control computer (MSI Desktop)
    - User: StrideONR
    - PWD: jecAME109

2. Power on the boom (turn on power supply)
    - Voltage should be set at 36V
    - Current should be set at max (50A)

3. After a minute or so, the boom pi should start advertising its hotspot, "Strideboom" - connect to this wifi network from the desktop control computer
    - I don't think this is password protected, but either way, the boom computer should have the password saved

4. On the boom desktop computer, open Visual Studio Code (blue icon on the left)
    - If VS code is not already attempting to connect, click on the "Remote Explorer" icon on the left side of the VSCode window. This should show a list of devices to connect to.
        - Click on strideboom
    - You may need to reload the window a few times before it asks for a password. The password to ssh into the pi is "password"

## Starting the Test

5. Once the VSCode window is ssh'd into the Raspberry Pi, navigate to "repository/WaLTER_Sr"

6. Source the terminal environment to ensure that all of the ros topics and nodes are understood by the terminal window when using commands
    - ```source install/setup.bash```

7. To launch the boom control code, run one of the following commands:
    - For testing the setup without logging: ```ros2 launch launch/boom_launch.py```
    - To launch the same control with logging: ```ros2 launch launch/boom_log_launch.py```
        - Logging will begin automatically as soon as this script is launched

8. BEFORE you press the start button on the joystick, wiggle the sticks around a bit. By default, the joystick initializes to "full throttle" in every direction

9. To begin controlling the robot, press the start button on the joystick.
    - Left joystick: Wheels
    - Right joystick: Knee
    - D-Pad Up/Down: Hip

10. To shutdown the test and stop logging, press ctrl-c in the terminal. This should shut down all nodes, stop logging, and send the motors into an idle state

## Data Extraction

11. Data Extraction - If you ran the log launch file, a rosbag folder stamped with the time and date of the test should appear in main repository folder
    - To convert this data to csv files, run the following (without <> brackets):
    ```python3 bag_reader.py <name_of_bag_folder>```
    - CSV files should be created and loaded into the rosbag folder

12. CSV and rosbag files are stored locally on the pi. To download the folder, right click on it in the VSCode window and select "download" you can then set a location on the boom desktop computer where you'd like the files downloaded.

