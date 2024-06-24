# ML-aircraft-damage-assesment
Using machine learning to supplement aircraft damage assesments

Notes for my overall system process. Nvidia Jetson Xavier + Realsense D435i + FLIR Boson IR camera + Arducam ToF camera -> eventaully moving to Jetson Orin though

Taskers for my portion of the project which is basically getting all the sensors to work together, not in order:
1. Intrinsic calibratino of the FLIR IR camera
2. Registration of the IR and ToF cameras
3. Converting the aligned depth and RGB/Grayscale images into a world-scaled and positioned point cloud
4. Potentially aligning the D435i and IT/RoF pair

Due to the use of ROS and Nvidia GPUs, the Isaac ROS resources will be utilized.
1. Isaac's bridge will be needed to port the IR image to ROS2, which is what is currently supported by Isaac ROS
- Realsense already has ROS2 support
- Arducam is already ROS2
2. Since migration from Xavier to Orin is possible along with how Isaac isn't exactly supported by the Jetpack or Ubuntu versions required by Isaac, a docker system will be used

Help from this: https://nvidia-isaac-ros.github.io/getting_started/index.html

Main notes for working through getting Isaac ROS running on Xavier:



Some misc. notes for starting Orin development:
