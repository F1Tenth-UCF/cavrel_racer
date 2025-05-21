# CAVREL Racer

This is the RoboRacer (F1TENTH) repo of the CAVREL Team. The repo has the following components
- The F1TENTH Gym Platform `f1tenth_gym`
- The simulation environment leveraging the F1TENTH Gym through a ROS bridge `sim_ws`
- The control software stack for the **CAVREL Racer** `racer_ws`

## Prerequisites

- **Ubuntu 20.04 (Focal Fossa)** or **Ubuntu 22.04 (Jammy Jellyfish)** environment
- **ROS 2 Foxy Fitzroy** or **ROS 2 Humble Hawksbill** (Instructions for ROS Foxy - https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
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
### Download the Repo

1. **Navigate to the Home Folder or Desired Directory:**

   ```bash
   cd $HOME
   ```

2. **Clone the Repository:**

   ```bash
   git clone https://github.com/f1tenth-ucf/cavrel_racer
   ```

## Setting Up F1Tenth Gym ROS Simulation Environment

This guide walks you through configuring your system to set up the F1Tenth Gym ROS environment [^1]. This repo also content some control algorithms and some maps as well.

[^1]: Those set of instructions are based on the [Installation Instruction](https://github.com/f1tenth/f1tenth_gym_ros) for F1Tenth Gym ROS Simulation as of 08/28/2024

-- *Last Tested: 05/13/2025*

### Setting Up F1tenth Gym

1. **Navigate to the Folder that has the repository:**

   ```bash
   cd cavrel_racer
   ```

2. **Navigate to the F1Tenth Gym folder:**

   ```bash
   cd f1tenth_gym
   ```

3. **Install the Python Dependencies For the F1Tenth Gym:**

   ```bash
   pip3 install -e .
   ```

### Setting Up ROS Environment for F1tenth Gym

1. **Go to the repository parent folder and navigate to the f1tenth_gym_ros folder:**

   ```bash
   cd sim_ws/src/f1tenth_gym_ros
   ```

2. **Confirm/Update the Map Path Parameter:**

    - Open the `sim.yaml` file located in the folder at `config/sim.yaml`.
    - Change the map_path parameter to point to the correct location:
    - The maps are in the `map` folder

      ex:
      ```bash
      map_path: "<your_home_dir>/cavrel_racer/sim_ws/src/f1tenth_gym_ros/maps/levine"
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

5. **Navigate to the Simulation's Top Level Folder `sim_ws` and Install ROS Dependencies:**

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

### Run the Simulation Environment

1. **Source the ROS 2 Setup Script if not Already Sourced**
   
   ***Replace `.bash` with `.sh` or `.zsh` depending on your terminal***

   **For ROS2 Foxy:**
   ```bash
   source /opt/ros/foxy/setup.bash
   ```

   **For ROS2 Humble:**
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Navigate to the Simulation's Top Level Folder `sim_ws` and Source the Local Workspace Setup Script:**

   ```bash
   source install/local_setup.bash
   ```

3. **Run the Simulation Environment**

   ```bash
   ros2 launch f1tenth_gym_ros gym_bridge_launch.py
   ```

   **An rviz window should pop up showing the simulation**

### Topics published by the simulation

`/scan`: The ego agent's laser scan

`/ego_racecar/odom`: The ego agent's odometry

`/map`: The map of the environment

A `tf` tree is also maintained.

### Topics subscribed by the simulation

`/drive`: The ego agent's drive command via `AckermannDriveStamped` messages

### Keyboard Teleop

The keyboard teleop node from `teleop_twist_keyboard` is also installed as part of the simulation's dependency. 

To run it, make sure `kb_teleop` is set to `True` in `sim.yaml`. Open another terminal, run:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Then, press `i` to move forward, `u` and `o` to move forward and turn, `,` to move backwards, `m` and `.` to move backwards and turn, and `k` to stop in the terminal window running the teleop node.

## Run CAVREL Racer Control Algorithm

1. Open a new terminal
2. **Source the ROS 2 Setup Script if not Already Sourced**
   
   ***Replace `.bash` with `.sh` or `.zsh` depending on your terminal***

   **For ROS2 Foxy:**
   ```bash
   source /opt/ros/foxy/setup.bash
   ```

   **For ROS2 Humble:**
   ```bash
   source /opt/ros/humble/setup.bash
   ```
3. Navigate to the top level parent folder of the repository `cavrel_racer`
4. Go to the `racer_ws`
5. Build the workspace and source the the packages

   ```bash
   colcon build
   source install/local_setup.bash
   ```

6. Run the desired control node

   ex:

   ```bash
   ros2 run gap_follow gap_follow_node
   ```

**If you are running the simulation and running the gap followind node, you should be able to see the car moves in the simulation**
