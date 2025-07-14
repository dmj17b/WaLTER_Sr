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
