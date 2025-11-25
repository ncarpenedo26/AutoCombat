Example folder structure of ros2_ws

my_ws/
 ├── src/     <-- your packages go here
 │    ├─ my_robot_description/
 │    ├─ my_robot_gazebo/
 │    └─ my_robot_control/
 ├── build/   <-- created automatically by colcon
 ├── install/ <-- created automatically by colcon
 └── log/     <-- also created automatically

AutoCombat /
    ros2_docker_ws /
        .devcontainer /
            Dockerfile
            devcontainer.json
        ros2_ws
            src/
            (...)
            


Setup:
https://apps.microsoft.com/detail/9pn20msr04dw?hl=en-US&gl=US


https://github.com/Https404PaigeNotFound/ros-humble-docker-demo/blob/main/docs/setup-from-scratch.md
looking to be the best option


![alt text](docs/image.png)

![alt text](docs/image-1.png)


Once in wsl make sure to start ssh agent so you can git commit from within the container
# Start the agent (if not already running)
eval "$(ssh-agent -s)"

# Add your key
ssh-add ~/.ssh/id_rsa

[!NOTE] Must have opened vscode in the folder of ros2_docker_ws to open our container!

[!NOTE] Make sure ssh-agent is running everytime before you open up any container!

lj@DESKTOP-UO7A26Q:~/AutoCombat/ros2_docker_ws$ echo $SSH_AUTH_SOCK lj@DESKTOP-UO7A26Q:~/AutoCombat/ros2_docker_ws$ eval "$(ssh-agent -s)" ssh-add ~/.ssh/id_rsa Agent pid 31937 Identity added: /home/lj/.ssh/id_rsa (louiejoshualabata@berkeley.edu) lj@DESKTOP-UO7A26Q:~/AutoCombat/ros2_docker_ws$ ssh-add -l 4096 SHA256:rnfRyJeLdFOzM9fU6pFIun94nfzZN3+RcMdkG36AanU louiejoshualabata@berkeley.edu (RSA) lj@DESKTOP-UO7A26Q:~/AutoCombat/ros2_docker_ws$ echo $SSH_AUTH_SOCK /tmp/ssh-XXXXXXdEtLNd/agent.31936 lj@DESKTOP-UO7A26Q:~/AutoCombat/ros2_docker_ws$


(WSL!)
https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Installation-Windows.html

ROS 2 Humble Hawksbill is made for Ubuntu 22.04 (Jammy Jellyfish).

(Chat GPT setup)
pip install vcstool


(Windows setup to install docker)
https://docs.docker.com/desktop/setup/install/windows-install/

https://docs.ros.org/en/humble/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.html#install-remote-development-extension
(From here)

How to setup Gazebo simulation:

https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html

Example github repo of X-drive mecanum ROS2 and Gazebo project:
https://github.com/MickySukmana/holonomic

Making a urdf:
https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.html

How to generate a urdf from Fusion360:
https://github.com/dheena2k2/fusion2urdf-ros2

How to generate a urdf from Onshape:
https://github.com/Rhoban/onshape-to-robot?tab=readme-ov-file

ROS2_Controller for our omni directional robot:
https://control.ros.org/humble/doc/ros2_controllers/doc/mobile_robot_kinematics.html#omnidirectional-wheeled-mobile-robots

ros2_control ('hardware-agnostic control framework for abstracting hardware and low-level control for 3rd party solutions like MoveIt2 and Nav2 systems.'):
https://control.ros.org/rolling/doc/resources/roscon2025_workshop.html

Nav2 (highlevel control system, behavior trees, nav, state estimation):
https://docs.nav2.org/concepts/index.html#controllers


cat << 'EOF' >> ~/.bash_profile

# ===========================
# WSL2 SSH Agent Auto-Start
# With debug prints
# ===========================
#!/bin/bash
echo "=== SSH Agent Setup Script Starting ==="

if [ -z "$SSH_AUTH_SOCK" ]; then
    echo "[1] SSH_AUTH_SOCK is empty. Checking for running ssh-agent..."
   
    RUNNING_AGENT="$(ps -ax | grep 'ssh-agent -s' | grep -v grep | wc -l | tr -d '[:space:]')"
    echo "[2] Number of running ssh-agent processes: $RUNNING_AGENT"
   
    if [ "$RUNNING_AGENT" = "0" ]; then
        echo "[3] No running ssh-agent found. Launching a new agent..."
        ssh-agent -s &> "$HOME/.ssh/ssh-agent"
        echo "[4] New ssh-agent started. Info saved in $HOME/.ssh/ssh-agent"
    else
        echo "[5] ssh-agent already running. Will use existing one."
    fi

    echo "[6] Evaluating ssh-agent environment..."
    eval "$(cat "$HOME/.ssh/ssh-agent")"
    echo "[7] SSH_AUTH_SOCK is now: $SSH_AUTH_SOCK"
else
    echo "[8] SSH_AUTH_SOCK is already set: $SSH_AUTH_SOCK"
fi

echo "[9] Running ssh-add..."
ssh-add 

if [ -n "$BASH_VERSION" ]; then
    if [ -f "$HOME/.bashrc" ]; then
        echo "[10] Sourcing .bashrc..."
        . "$HOME/.bashrc"
    fi
fi

echo "=== SSH Agent Setup Script Finished ==="

# ===========================
# End SSH Agent Auto-Start
# ===========================

EOF



# WTF another git thing
cd /home/lj/AutoCombat
sudo chown -R $USER:$USER .
chmod -R u+rwX .
