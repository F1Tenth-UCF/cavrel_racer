# Setting Up F1Tenth Gym ROS

This guide walks you through configuring your system to set up the F1Tenth Gym ROS environment [^1]. This repo also content some control algorithms and some maps as well.

-- *Last Tested: 05/12/2025*

## Prerequisites

- **Ubuntu 20.04 (Focal Fossa)** or **Ubuntu 22.04 (Jammy Jellyfish)** environment
- **ROS 2 Foxy Fitzroy** or **ROS 2 Humble Hawksbill**
- **Python 3** *(Comes with Ros 2)*

## System preparation

1. **Update and Upgrade System Packages:**

   ```bash
   sudo apt update && sudo apt upgrade
   ```

2. **Install Pip for Python 3:** *(if not already installed)*

   ```bash
   sudo apt install python3-pip
   ```

## Setting Up F1tenth Gym

1. **Navigate to the Home Folder or Desired Directory:**

   ```bash
   cd $HOME
   ```

2. **Clone the F1Tenth Gym Repository:**

   ```bash
   git clone https://github.com/f1tenth/f1tenth_gym
   cd f1tenth_gym
   ```

3. **Install the Python Dependencies For the F1Tenth Gym:**

   ```bash
   pip3 install -e .
   ```

## Setting Up ROS Environment for F1tenth Gym

1. **Create a Workspace:**

   ```bash
   cd $HOME && mkdir -p sim_ws/src
   ```

2. **Clone the F1Tenth Gym ROS Repository into the Workspace:**

   ```bash
   cd $HOME/sim_ws/src
   git clone https://github.com/f1tenth-ucf/f1tenth_gym_ros
   ```

3. **Confirm/Update the Map Path Parameter:**

    - Open the `sim.yaml` file located in your cloned repository at `config/sim.yaml`.
    - Change the map_path parameter to point to the correct location:
    - The maps are in the `map` folder

      ex:
      ```bash
      map_path: "<your_home_dir>/sim_ws/src/f1tenth_gym_ros/maps/levine"
      ```

4. **Initialize and Update `rosdep`:**

   **If you have not yet initialize rosdep on this machine**
   ```bash
   sudo rosdep init
   ```

   **For ROS2 Foxy:**
   ```bash
   rosdep update --include-eol-distros --rosdistro foxy
   ```

   **For ROS2 Humble:**
   ```bash
   rosdep update
   ```

5. **Navigate to the Top Level of the Simulation Workspace Folder `sim_ws` and Install ROS Dependencies:**

   **For ROS2 Foxy:**
   ```bash
   rosdep install -i --from-path src --rosdistro foxy -y
   ```

   **For ROS2 Humble:**
   ```bash
   rosdep install -i --from-path src --rosdistro humble -y
   ```

6. **Build The Workspace**

   ```bash
   colcon build
   ```

## Run the Simulation Environment

1. **Source the ROS 2 Setup Script if not Already Sourced**

   **For ROS2 Foxy:**
   ```bash
   source /opt/ros/foxy/setup.bash
   ```

   **For ROS2 Humble:**
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Navigate to the Top Level of the Simulation Workspace Folder `sim_ws`and Source the Local Workspace Setup Script:**

   ```bash
   source install/local_setup.bash
   ```

3. **Run the Simulation Environment**

   ```bash
   ros2 launch f1tenth_gym_ros gym_bridge_launch.py
   ```

   **An rviz window should pop up showing the simulation**

# Topics published by the simulation

`/scan`: The ego agent's laser scan

`/ego_racecar/odom`: The ego agent's odometry

`/map`: The map of the environment

A `tf` tree is also maintained.

# Topics subscribed by the simulation

`/drive`: The ego agent's drive command via `AckermannDriveStamped` messages

# Keyboard Teleop

The keyboard teleop node from `teleop_twist_keyboard` is also installed as part of the simulation's dependency. 

To run it, make sure `kb_teleop` is set to `True` in `sim.yaml`. Open another terminal, run:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Then, press `i` to move forward, `u` and `o` to move forward and turn, `,` to move backwards, `m` and `.` to move backwards and turn, and `k` to stop in the terminal window running the teleop node.
