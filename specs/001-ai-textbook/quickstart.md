# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

## Prerequisites

Before starting with the textbook, ensure you have the following:

- **Operating System**: Ubuntu 22.04 LTS (recommended) or a compatible environment
- **Hardware**:
  - Minimum: 8GB RAM, 4 cores, 50GB free disk space
  - Recommended: 16GB+ RAM, 8+ cores, dedicated GPU (NVIDIA RTX series recommended)
- **Software**: Git, Docker, Node.js 18+, Python 3.10+

## Environment Setup

### 1. Clone the Repository
```bash
git clone https://github.com/[username]/ai-robo-learning.git
cd ai-robo-learning
```

### 2. Install ROS 2 Humble
Follow the official installation guide for Ubuntu 22.04:
```bash
# Add ROS 2 repository
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble packages
sudo apt update
sudo apt install ros-humble-desktop ros-humble-ros-base
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool
```

### 3. Install Gazebo Harmonic
```bash
sudo apt install ros-humble-gazebo-*
sudo apt install gazebo
```

### 4. Install NVIDIA Isaac Sim (Optional but Recommended)
1. Download Isaac Sim from NVIDIA Developer website
2. Follow installation instructions for your system
3. Ensure GPU drivers are up to date (minimum CUDA 11.8 support)

### 5. Set up the Textbook Environment
```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Install Python dependencies
pip3 install -r requirements.txt

# Install Node.js dependencies for Docusaurus
npm install
```

## Running the Textbook Locally

### 1. Start the Docusaurus Server
```bash
npm start
```
The textbook will be available at `http://localhost:3000`

### 2. Access the Simulation Environment
For chapters involving simulation:
```bash
# Terminal 1: Start ROS 2
source /opt/ros/humble/setup.bash
ros2 launch [package_name] [launch_file].py

# Terminal 2: Start Gazebo
gazebo [world_file].world

# Terminal 3: Run your robot control code
source /opt/ros/humble/setup.bash
python3 [your_script].py
```

## First Lab: Creating Your First Robot Simulation

1. Navigate to Module 1 → Chapter 1 → Lab Exercise in the textbook
2. Follow the setup instructions to create a basic URDF robot model
3. Launch the simulation using the provided launch file
4. Verify the robot appears correctly in Gazebo
5. Run the example control script to make the robot move

## Content Navigation

- **Modules**: Major learning units (4 total)
- **Chapters**: Detailed topics within each module
- **Lessons**: Specific learning segments
- **Labs**: Hands-on exercises with step-by-step instructions
- **Reflections**: Critical thinking prompts for deeper understanding

## Getting Help

- Check the FAQ section in the textbook sidebar
- Report issues via GitHub Issues
- Join the community forum linked in the textbook footer
- For simulation-specific problems, consult the Troubleshooting section

## Next Steps

After completing the setup:
1. Start with Module 1, Chapter 1
2. Complete the foundational lessons before moving to advanced topics
3. Don't skip the lab exercises - they're crucial for understanding
4. Use the progress tracking feature to monitor your learning journey