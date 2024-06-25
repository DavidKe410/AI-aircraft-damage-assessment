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

Once NVMe is here, follow this guide:



Some misc. notes for starting Orin development:
