Starting the Orin development with Jetpack 5.1.2. Arducam doesn't with JP 5.1.3 and Realsense is very annoying with JP6 due to kernal + HID issues:
1. Installed a new NVMe drive
2. Used Nvidia's sdkmanager on another Ubuntu amd64 computer to flash and install runtime/sdk components to the Orin. (Jetpack 5.1.2 w/ all runtime and sdk components installed)
- This video basically walks through it: https://www.youtube.com/watch?v=Ucg5Zqm9ZMk&t=90s
- Sdkmanager download: https://developer.nvidia.com/sdk-manager
3. Back on the Orin, sudo apt update && sudo apt upgrade
4. sudo apt install python3-pip
4. sudo pip3 install -U jetson-stats
5. restart and run jtop
  (Some erros may show up from Jetpack isntallation, install missing Jetpack components with these commands: https://developer.nvidia.com/embedded/learn/get-started-jetson-agx-orin-devkit)
    - Just housekeeping libraries and other stats
    -   CUDA: 11.4.315
    -   cuDNN: 8.6.0.166
    -   TensorRT: 8.5.2.2
    -   VPI: 2.3.9
    -   Vulkan: 1.3.204
    -   OpenCV: 4.5.4 with CUDA: NO --> Later do change it to yes w/ 4.6.0
    -   Jetpack: 5.1.2
    -   Distr: Ubuntu 20.04 Focal
    -   Python: 3.8.10
    -   L4t: 35.4.1  
    -   Interfaces: 192.168.1.135, docker0: 172.17.0.1
6. Downloading ROS1 Noetic: https://wiki.ros.org/noetic/Installation/Ubuntu
    - Installed the desktop full, the dependencies that they mention, rosdep
7. Getting the FLIR IR camera up.
    - Make a workspace for the wrapper and enter: mkdir -p air_dmg_assesment_ws/src && cd air_dmg_assesment_ws/src
    - git clone https://github.com/astuff/flir_boson_usb
    - Exit from src to run rosdep: cd .. && rosdep install --from-paths src -y --ignore-src
      - Should download other resources like opencv if you didn't have them already
      - source /opt/ros/noetic/setup.bash before hand too
    - catkin_make
    - Checking funtionality
      - source devel/setup.bash
      - check usb video with ls /dev/video*
      - cd && roslaunch flir_boson_usb flir_boson.launch dev:=/dev/video0
        - switch video0 with your configuration
8. Starting the legacy Realsense SDK and ros wrapper (To restart and remove all realsense: dpkg -l | grep "realsense" | cut -d " " -f 3 | xargs sudo dpkg --purge)
    - Started by looking at what other people did to resolve some of the issues, here is what i did just in case to mititage future potential problems (SEEMS LIKE IN GENERAL THERE ARE A LOT OF PORBLEMS WITH REALSENSE AND JETSONS :()
      - I installed OpenCV with Cuda (4.6.0): https://github.com/AastaNV/JEP/blob/master/script/install_opencv4.6.0_Jetson.sh within a folder called openCVwithCuda :/
        - For this script, remove the installation of v4l2ucp and it hsould be good, though it will take some time
      - Then did this to make sure everything was still intact: sudo apt install ros-noetic-desktop-full
      - sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade
      - sudo apt-get install libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev
      - sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at
    - To actually start installing the SDK, first clone into home: git clone https://github.com/IntelRealSense/librealsense.git [This method is based on Method 2 of this: https://github.com/IntelRealSense/realsense-ros/blob/ros1-legacy/README.md#installation-instructions and "Building from Source using Native Backend" from this https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md#building-from-source-using-native-backend: "
      - cd librealsense && ./scripts/setup_udev_rules.sh
      - mkdir build && cd build
      - cmake .. -DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=release -DFORCE_RSUSB_BACKEND=false -DBUILD_WITH_CUDA=true
      - make -j$(($(nproc)-1))
      - sudo make install
    - Check functionality with realsense-viewer
    - Next, building the ros wrapper: https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy
      - cd ~/air_dmg_assesment_ws/src && git clone https://github.com/IntelRealSense/realsense-ros.git -b ros1-legacy
      - cd realsense-ros/
      - git checkout \`git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1\`
          - Make sure the single back quote is there
          - May not work, try this, using a variable:
              - latest_tag=$(git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1)
              - git checkout $latest_tag
      - cd ~/air_dmg_assesment_ws/src/realsense-ros/realsense2_camera && nano CMakeLists.txt
        - Changes: https://github.com/IntelRealSense/realsense-ros/issues/2326#issuecomment-1107658481
        - And for reference: https://github.com/IntelRealSense/realsense-ros/issues/2467#issuecomment-1268417031, [Did not use that much but another comment: https://github.com/IntelRealSense/librealsense/issues/10722#issuecomment-1230250762]
      - sudo apt-get install ros-noetic-ddynamic-reconfigure
      - cd ~/air_dmg_assesment_ws && catkin_make
      - source devel/setup.bash
    - Check functionality with cmd: roslaunch realsense2_camera rs_camera.launch enable_gyro:=true enable_accel:=true unite_imu_method:=linear_interpolation align_depth:=true enable_pointcloud:=true
      - For the point cloud, make sure the fixed frame is set to smth like camera_link
      - Things that may be concerning but are not :/
        1. "No stream match for pointcloud chosen texture Process - Color"
          - This is whenever there is a frame drop of the pointcloud texture: https://github.com/IntelRealSense/realsense-ros/issues/588#issuecomment-457890773
        2. "(ds-options.cpp:93) Asic Temperature value is not valid!"
          - Just the insignificant moments when the timing is slightly off when trying to read from sensor: https://github.com/IntelRealSense/librealsense/issues/10378
        3. "Param '/camera/rgb_camera/power_line_frequency' has value 3 that is not in the enum { {50Hz: 1} {60Hz: 2} {Disabled: 0} }. Removing this parameter from dynamic reconfigure options."
          - No consequences in performance, just to mititage noise that can come from certain lights that are at those different frequencies: https://github.com/IntelRealSense/realsense-ros/issues/2359
9. Starting with the Arducam ToF, Galactic/Noetic bridge
  - Start following this to install ROS2 Galactic: https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html
    - Downloaded the base
  - For the Arducam: https://docs.arducam.com/Nvidia-Jetson-Camera/Time-of-Flight-Camera/ROS-With-Arducam-ToF-Camera/
    - cd && git clone https://github.com/ArduCAM/Arducam_tof_camera.git
    - cd ~/Arducam_tof_camera/jetson && ./Install_dependecies_jetson.sh 
    - yes to reboot
    - To test functionality: cd ~/Arducam_tof_camera && ./compile.sh
    - mkdir -p ~/ardu_tof_bridge_ws/src
    - cd Arducam_tof_camera/ros2_publisher/src
    - mv arducam ~/ardu_tof_bridge_ws/src
    - source /opt/ros/galactic/setup.bash
    - cd ~/ardu_tof_bridge_ws && colcon build --merge-install
    - source install/setup.bash
      - when running: ros2 run arducam tof_pointcloud
    - I moved the orginal Arducam_tof_camera folder into the arducam package as well for housekeeping
  - Now for bridging ROS1 and ROS2: https://github.com/ros2/ros1_bridge
    - cd ~/ardu_tof_bridge_ws/src && git clone sudo apt https://github.com/ros2/ros1_bridge.git
    - export ROS1_INSTALL_PATH=/opt/ros/noetic
    - export ROS2_INSTALL_PATH=/opt/ros/galactic
    - source ${ROS1_INSTALL_PATH}/setup.bash
    - source ${ROS2_INSTALL_PATH}/setup.bash
    - cd .. && colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure --merge-install
    - cd ~/ardu_tof_bridge_ws/src/ros1_bridge && touch bridge.yaml (Or just downlod the one I uploaded)
  - To check functionality and how to run
    - Terminal 1: source /opt/ros/noetic/setup.bash && roscore
    - Terminal 2: source /opt/ros/noetic/setup.bash && rosparam load ~/ardu_tof_bridge_ws/src/ros1_bridge/bridge.yaml
      - Can run a subscriber here, I did rviz
    - Terminal 3: source ardu_tof_bridge_ws/install/setup.bash && ros2 run arducam tof_pointcloud
    - Terminal 4: source ardu_tof_bridge_ws/install/setup.bash && ros2 run ros1_bridge parameter_bridge


For script

Terminal 1: source ~/air_dmg_assesment_ws/devel/setup.bash \
&& roslaunch flir_boson_usb flir_boson.launch dev:=/dev/video1 \
&& rosparam load ~/ardu_tof_bridge_ws/src/ros1_bridge/bridge.yaml

Terminal 2: source ~/air_dmg_assesment_ws/devel/setup.bash && roslaunch realsense2_camera rs_camera.launch enable_gyro:=true enable_accel:=true unite_imu_method:=linear_interpolation align_depth:=true 

Terminal 3: source ardu_tof_bridge_ws/install/setup.bash && ros2 run arducam tof_pointcloud

Terminal 4: source ardu_tof_bridge_ws/install/setup.bash && ros2 run ros1_bridge parameter_bridge

Random TF terminal: rosrun tf static_transform_publisher 0 0 0.075 0.5 0.5 0.5 -0.5 sensor_frame camera_link 100

Trying to get the model YoloV8 running on the Orin with a Docker
1. Follow this guide: https://docs.ultralytics.com/guides/nvidia-jetson/#run-on-jetpack-5x




------------------
  - sudo docker build -t ardu_bridge . (Dockerfile basically unmodified from official besides deleting the entrypoint that soruces the terminal)
  - sudo docker run -it --cap-add=SYS_PTRACE --privileged --network=host --pid=host --runtime=nvidia -v=/dev:/dev -v /usr/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu -v /usr/lib/tegra:/usr/lib/tegra -v /usr/src:/usr/src --entrypoint=/bin/bash --rm --name=ardu_tof_bridge ardu_bridge:latest
  - sudo docker run -it --cap-add=SYS_PTRACE --privileged --network=host --pid=host --runtime=nvidia -v=/dev:/dev -v=/var/lib/dpkg:/var/lib/dpkg:ro --entrypoint=/bin/bash --rm --name=ardu_tof_bridge ardu_tof_bridgev2:latest

    - Start following this guide to install ROS2 Humble from source: https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html#id4
    - Choose the Ubuntu 20.04 LTS option
    - MAKE SURE NOT TO SOURCE NOETIC/Humble BEFORE BUILDING; To check: printenv | grep -i ROS
      - To unset env variable: unset variable_name
    - This takes A LOT of time when we colcon build, would recommend slimming this https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos down by removing rviz and other unneccesary packages
      - AND I DID! Second time around, just download my ros2.repos into the ros2_humble folder (I just removed the visualizations, rviz, tutorials/examples, and rclpy (which I mention below))
      - Then run this: "vcs import --input ros2.repos src" instead of the link to their page
      - THERE IS A PROBLEM building rclpy. Simply colcon ignore it or use my .repos that already removed it: cd ~/ros2_humble/src/ros2/rclpy && touch COLCON_IGNORE
  - For the Arducam: https://docs.arducam.com/Nvidia-Jetson-Camera/Time-of-Flight-Camera/ROS-With-Arducam-ToF-Camera/
    - I had to sudo apt update && sudo apt apgrade and then sudo apt --fix-broken install the nvidia container [may not need to do this]
    - Simply followed the online Arducam ToF guide:
    - Make sure to source humble before colcon building
    - source Arducam_tof_camera/ros2_publisher/install/setup.bash
          - when running: ros2 run arducam tof_pointcloud


geoclue geoclue 755 /var/lib/geoclue
root crontab 2755 /usr/bin/crontab
root ssl-cert 710 /etc/ssl/private
root messagebus 4754 /usr/lib/dbus-1.0/dbus-daemon-launch-helper
