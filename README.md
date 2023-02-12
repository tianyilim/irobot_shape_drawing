# Coding Submission — Tianyi

## Setup
The following instructions assume Ubuntu 20.04, ROS2 Galactic, and Gazebo 11 has been installed.
1. Clone this repository into a convenient location, for example your home folder.
2. Clone the iRobot Simulator into `src`. Assuming that you cloned this repo into `~/`:
    ```bash
    cd ~/irobot_shape_drawing/src
    git clone git@github.com:iRobotEducation/create3_sim.git -b galactic
    ```
3. Setup dependencies. Do note that ROS2 Galactic is EOL, so add the following when running `rosdep`:
   ```bash
    cd ~/irobot_shape_drawing
    sudo apt update
    rosdep update --include-eol-distros # This allows rosdep to find ROS galactic dependencies
    rosdep install --from-path src -yi
    ```
4. Edit the RViz config file in the Create simulator. This is an easy way to make use of the existing launch files, while displaying output in the way we want.
    ```bash
    cd ~/irobot_shape_drawing
    mv irobot_create_view.rviz src/create3_sim/irobot_create_common/irobot_create_common_bringup/rviz/irobot_create_view.rviz
    ```
5. Build the project:
   ```bash
   cd ~/irobot_shape_drawing
   colcon build --symlink install
   ```
6. In a new terminal source the project.
    ```bash
    cd ~/irobot_shape_drawing
    source install/setup.bash
    ```

## Running
After sourcing the setup (step 6 above), we can run the shape drawing task by running `ros2 launch irobot_command task.launch.py`.

Here is a screenshot of the drawn letters.

On YouTube [link] is an annotated video of the robot moving around.

## Code Structure
