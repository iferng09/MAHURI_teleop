# MAHURI_teleop

Tele operator node for MAHURI Android app.

## Dependencies:

### Clonning the repository
```bash
cd $HOME/dev_ws/src
git clone https://github.com/iferng09/MAHURI_teleop.git
```
### Building ROS 2 package
Open the terminal and source the ROS:

```bash
source /opt/ros/foxy/setup.bash

# Build the packages
cd $HOME/dev_ws
colcon build --packages-select mahuri_teleop
```
### Running the talker and listener
Open two terminals and source ROS 2 in both:
```bash
source /opt/ros/foxy/setup.bash
```

**Terminal 1:** Run the talker:
``` bash
ros2 run mahuri_teleop talker
```

**Terminal 2:** Run the listener:
```bash
ros2 run mahuri_teleop listener
```
