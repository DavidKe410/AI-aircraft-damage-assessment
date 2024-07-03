# ML-aircraft-damage-assesment
Using machine learning to supplement aircraft damage assesments

Notes for my overall system process. Nvidia Jetson Xavier + Realsense D435i + FLIR Boson IR camera + Arducam ToF camera -> eventaully moving to Jetson Orin though
---
Taskers for my portion of the project which is basically getting all the sensors to work together, not in order:
1. Intrinsic calibratino of the FLIR IR camera
2. Registration of the IR and ToF cameras
3. Converting the aligned depth and RGB/Grayscale images into a world-scaled and positioned point cloud
4. Potentially aligning the D435i and IT/RoF pair
---
Due to the use of ROS and Nvidia GPUs, the Isaac ROS resources will be utilized.
1. Isaac's bridge will be needed to port the IR image to ROS2, which is what is currently supported by Isaac ROS
- Realsense already has ROS2 support
- Arducam is already ROS2
2. Since migration from Xavier to Orin is possible along with how Isaac isn't exactly supported by the Jetpack or Ubuntu versions required by Isaac, a docker structure seems to be necessary
---
Help from this: https://nvidia-isaac-ros.github.io/getting_started/index.html
---
Main notes for working through getting Isaac ROS running on Xavier:
Due to the potential of moving towards Orin again, it won't support Noetic naitively, will need a noetic docker :/
- Issac ROS provides instructions but I don't think it needs to be Isaac ROS specific in this case.
They recommend using the Isaac ROS Dev Docker images.
So to setup the dev environment, at elast 30 GB is needed and NVMe SSD is required :/
1. Getting started: First with the Compute Setup: https://nvidia-isaac-ros.github.io/getting_started/hardware_setup/compute/index.html
2. Moves onto Jetson SSD setup: https://nvidia-isaac-ros.github.io/getting_started/hardware_setup/compute/jetson_storage.html
- we can totally do the 1st option, but second might be quicker as no need to reflash
3. Skip Preempt_rt kernel; no need for the robot to manipulate; has some ethernet latency reduction
4. Skip Hawk camera setup
5. Go through the Isaac ROS RealSense Setup, though it wants you to finsih the Developer Environment Setup first.
  - it also suggests increasing the "maximum linux kernal receive buffer size" (Cycline DDS tuning): https://docs.ros.org/en/rolling/How-To-Guides/DDS-tuning.html#cyclone-dds-tuning; seems like the situation was vastly improved, so we most likely don't need to do this, esp since predicted message sizes are 9 MB and this doens't have to tkae palce at a specific point within setup.

---
To get started with porting the IR image to ROS2, guide: https://nvidia-isaac-ros.github.io/concepts/nitros_bridge/tutorial_isaac_sim.html
- Just as a note, seems to only convert image messages. May need the CPU primary ros1_brige for other things
- Does not auto connect topic names, specifically receives a topic and publishes it on its own
- Seems to be a ROS1 version and ROS2 version that may be needed both to push and pull from GPU
- might not be worth it if we can just use ros1_bridge
- Here is the overview with an example: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nitros_bridge/index.html
---
Some misc. notes for starting Orin development:
1. Installed a new NVMe drive
2. Used Nvidia's sdkmanager on another Ubuntu amd64 computer to flash and install runtime/sdk components to the Orin. (Jetpack 6.0 w/ all runtime and sdk components installed)
- This video basically walks through it: https://www.youtube.com/watch?v=Ucg5Zqm9ZMk&t=90s
- Sdkmanager download: https://developer.nvidia.com/sdk-manager
3. Back on the Orin, sudo apt update && sudo apt upgrade
4. sudo apt install python3-pip
4. sudo pip3 install -U jetson-stats
5. restart and run jtop
  (Some erros may show up from Jetpack isntallation, install missing Jetpack components with these commands: https://developer.nvidia.com/embedded/learn/get-started-jetson-agx-orin-devkit)
    - Just housekeeping libraries and other stats
    -   CUDA: 12.2.140
    -   cuDNN: 8.9.4.25
    -   TensorRT: 8.6.2.3
    -   VPI: 3.1.5
    -   Vulkan: 1.3.204
    -   OpenCV: 4.8.0 with CUDA: NO
    -   Jetpack: 6.0
    -   Distr: Ubuntu 22.04 Jammy Jellyfish
    -   Python: 3.10.12
    -   L4t: 36.3.0
    -   Interfaces: 192.168.1.178, docker0: 172.17.0.1
6. Downloading ROS2 Humble: https://nvidia-isaac-ros.github.io/getting_started/isaac_apt_repository.html
    - Difference from standard installation: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
        - Is that it also installs gnupg and wget, but these are already included.
        - And that is registers the GPG ket with isaac ros keys
    - Installed development tools and ROS tools (common packages) and python3 flake/pytest packages according to the official ros humble docs
    - Installed ros-humble-desktop-full (may not be needed, but also installed ros-dev-tools)
7. Starting with the Arducam ToF
    - Simply followed the online Arducam ToF guide:
    - Make sure to source humble before colcon building
8. Moving onto getting the FLIR IR camera working within a Noetic docker and getting the ISAAC ROS bridge to work
    - Followed this guide to first create the container:https://nvidia-isaac-ros.github.io/concepts/nitros_bridge/setup_ros1_docker.html
        - I also: mkdir -p ~/workspaces/ros1_ws/flir_ir/src
        - Entered it and cloned the flir IR ros wrapper: https://github.com/astuff/flir_boson_usb
        - Then followed the build command included
    - Before continuining with the guide here and running its built in entrypoint: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nitros_bridge/index.html
        - Run the container with /bin/bash as the entrypoint: "docker run -it --cap-add=SYS_PTRACE --privileged --network host --pid host --runtime nvidia --device /dev/video0 --name nitros_bridge --entrypoint=/bin/bash --rm nitros_bridge:latest"

7. Starting with the Intel Realsense D435i
    - At this moment, there is no complete support for the D435i with the JEtpack 6 installation
    -   However, they have added support for JP6.0 with the D457 MIPI
    -   We can use this to allow for full functionality with the D435i
    -     First follow this guide: https://github.com/IntelRealSense/realsense_mipi_platform_driver/tree/dev
    -     This one has simiar details: https://github.com/IntelRealSense/realsense_mipi_platform_driver?tab=readme-ov-file

need to download : sudo apt install python3-colcon-common-extensions ...


figuring out things

workspaces/isaac_ros_1-dev/src/isaac_ros_nitros_bridge/ros1/isaac_ros_nitros_bridge_ros1/launch

---
Arducam ToF seems to take video0, FLIR IR seems to take video1 and video2
Realsense seems to take video3 to video8
Unless plugged in in a different order with flir taking 7 and 8 and realsense taking 1 to 6
---
Things presumably optimized for the Jetson
- Base image from Nvidia's SDKmanager
- Realsense

https://www.intel.com/content/dam/support/us/en/documents/emerging-technologies/intel-realsense-technology/Intel-RealSense-SDK2-Github-Guide.pdf

dpkg -l | grep "realsense" | cut -d " " -f 3 | xargs sudo dpkg --purge
(Reading database ... 246820 files and directories currently installed.)
Removing librealsense2-dev:arm64 (2.55.1-0~realsense.3337) ...
Removing librealsense2-udev-rules:arm64 (2.55.1-0~realsense.3337) ...
Removing librealsense2-utils:arm64 (2.55.1-0~realsense.3337) ...
Removing librealsense2-gl:arm64 (2.55.1-0~realsense.3337) ...
Removing librealsense2:arm64 (2.55.1-0~realsense.3337) ...
Processing triggers for libc-bin (2.35-0ubuntu3.8) ...
