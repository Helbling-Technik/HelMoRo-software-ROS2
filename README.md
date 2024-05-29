# HelMoRo-software-ROS2

# Installation

The packages included support ROS Humble on Ubuntu 20.04 and Ubuntu 22.04.

1. Install [ROS Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) on Helmoro and on your local machine.

    Make sure you source ROS 2 every time you open a new Terminal using

      ```sh
      source /opt/ros/humble/setup.bash
      ```

    Alternatively you can add it to your .bashrc file

      ```sh
      echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
      ```

2. Create a [ROS2 Workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)

      ```sh
      mkdir -p ~/ros2_ws/src
      ```

3. Clone the repository into your catkin workspace. The commands are stated in the following.

      ```sh
      git clone https://github.com/Helbling-Technik/HelMoRo-software-ROS2.git
      ```
4. Install the ROS 2 dependencies

      ```sh
      cd ~/ros2_ws
      sudo apt-get update
      rosdep install --from-path src -yi
      ```

5. Build and source the Helmoro software package using

      ```sh
      colcon build --symlink-install
      source install/local_setup.bash
      ```

6. Run the Simulation

      ```sh
      ros2 launch helmoro_bringup gazebo_sim_launch.py
      ```
