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
    - source Arducam_tof_camera/ros2_publisher/install/setup.bash
          - when running: ros2 run arducam tof_pointcloud
8. Moving onto getting the FLIR IR camera working within a Noetic docker and getting the ISAAC ROS bridge to work
    - Followed this guide to first create the container:https://nvidia-isaac-ros.github.io/concepts/nitros_bridge/setup_ros1_docker.html
        - Then followed the build command included
    - Before continuining with the guide here and running its built in entrypoint: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nitros_bridge/index.html
        - make sure the flir camera is plugged in so that there is a video1
        - Run the container with /bin/bash as the entrypoint: "sudo docker run -it --cap-add=SYS_PTRACE --privileged --network host --pid host --runtime nvidia -v /dev:/dev --entrypoint=/bin/bash --rm --name nitros_bridge nitros_bridge:latest"
             - Can switch out the -v /dev:/dev with "--device /dev/video1" if you want to be mroe secure, but does not allow dynamic plug/unplug, so just make sure everything is plugged in and stays in.
        - Treat it like any other system: start with cd && sudo apt update && sudo apt upgrade
        - Then "sudo rosdep init" and "rosdep update"
        - Then moving onto the flir ir wrapper
             - mkdir -p flir_ir/src && cd flir_ir/src
             - git clone https://github.com/astuff/flir_boson_usb
             - cd .. && rosdep install --from-paths src -y --ignore-src
                    - should download opencv and etc
             - source /opt/ros/noetic/setup.bash
             - catkin_make
             - cd .. && source flir_ir/devel/setup.bash
             - check usb video with ls /dev/video*
             - roslaunch flir_boson_usb flir_boson.launch
                    - Unfortunately, the flir_boson_rectified.launch is nonfunctional due to problems with using nodelets: "[ERROR] [1720023405.952799862]: Failed to load nodelet [/flir_boson/boson_camera_nodelet_manager_image_proc] of type [image_proc/rectify] even after refreshing the cache: According to the loaded plugin descriptions the class image_proc/rectify with base class type nodelet::Nodelet does not exist. Declared types are  flir_boson_usb/BosonCamera..."
                    - simply not worth it to debug on our timeline, since we can image process after bridging to ROS2
        - In another terminal on host system:
             - "sudo docker ps" to get container ID 
             - "sudo docker exec -it <container_ID> bash" to open another terminal in the docker container
             - cd && source /opt/ros/noetic/setup.bash
             - "rostopic list" and you should see /flir_boson/camera_info and /flir_boson/image_raw
        - Moving onto some quick deconfliction and reconfiguring of the default package values
             - Navigate to the launch file: cd flir_ir/src/flir_boson_usb/launch
             - Use "vi flir_boson.launch" to enter the file; Type "i" to start insterting and then ":wq" to save and exit
                   - jsut change the default device to video1 instead of video0
                   - Other changes can be done here, but nothing too important. I'm not sure the yaml calibration is even used though its linked
             - huh thought there was more, but it was just that I guess
    - Now moving to getting the bridge to work for us. Error when you run their docker run command :

++--++
[ERROR] [1720027747.649430812]: Failed to load nodelet [/ImageConverterNode] of type [isaac_ros_nitros_bridge_ros1/ImageConverterNode] even after refreshing the cache: Failed to load library /workspaces/isaac_ros_1-dev/install_isolated/lib//libisaac_ros_nitros_bridge_ros1.so. Make sure that you are calling the PLUGINLIB_EXPORT_CLASS macro in the library code, and that names are consistent between this macro and your XML. Error string: Could not load library (Poco exception = /lib/aarch64-linux-gnu/libc.so.6: version `GLIBC_2.34' not found (required by /usr/lib/aarch64-linux-gnu/nvidia/libnvrm_gpu.so))
[ERROR] [1720027747.649560256]: The error before refreshing the cache was: Failed to load library /workspaces/isaac_ros_1-dev/install_isolated/lib//libisaac_ros_nitros_bridge_ros1.so. Make sure that you are calling the PLUGINLIB_EXPORT_CLASS macro in the library code, and that names are consistent between this macro and your XML. Error string: Could not load library (Poco exception = /lib/aarch64-linux-gnu/libc.so.6: version `GLIBC_2.34' not found (required by /usr/lib/aarch64-linux-gnu/nvidia/libnvrm_gpu.so))
[FATAL] [1720027747.651327861]: Failed to load nodelet '/ImageConverterNode` of type `isaac_ros_nitros_bridge_ros1/ImageConverterNode` to manager `standalone_nodelet'
++--++

     - Took some time to understand a couple of files in the Issac ROS bridge, now will try running the entrypoint.sh with correct parameters and the FLIR Ir running manually
            - Navigate to: /workspaces/isaac_ros_1-dev/src/isaac_ros_nitros_bridge/scripts
            - sudo chmod +x nitros-bridge-entrypoint.sh isaac_ros_nitros_bridge_ros1.py 
            - then ran ./nitros-bridge-entrypoint.sh nitros_bridge_image_converter.yaml nitros_bridge_image_converter.launch "/flir_boson/image_raw" "/ros1_output_image"
                  - within the same directory: /workspaces/isaac_ros_1-dev/src/isaac_ros_nitros_bridge/scripts
            - **SAME ERROR THOUGH**

     - Think its a problem with GLIBC_2.34 primarily. Asked Chat GPT and looked at multiple stack exchange posts and this is the fix it gave but also said its super risky

++--++
sudo apt update && sudo apt upgrade && sudo apt install gawk bison
wget http://ftp.gnu.org/gnu/libc/glibc-2.34.tar.gz
tar -xzf glibc-2.34.tar.gz
cd glibc-2.34
mkdir build
cd build
../configure --prefix=/opt/glibc-2.34
make -j4
sudo make install
export LD_LIBRARY_PATH=/opt/glibc-2.34/lib:$LD_LIBRARY_PATH
++--++

     - I don't think this is easily fixable since the inherent kernal within the image is non updatable unless we upgrade to 22.04 which noetic can't run on...
     - So the regular bridge works, just make sure the master uri are good. We are getting 60 fps with the regular bridge and there actually isn't a huge CPU load, so this was totally fine from the beginning damn it.
9. Getting started with the ISAAC ros dev environment docker setup for the depth image pipeline and the realsense
       - https://nvidia-isaac-ros.github.io/getting_started/hardware_setup/compute/index.html; already had the plugin though
       - Then: https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html, skip the ssd instructions 
             - Step 4, do Jetson with SSD
10. Looks like that didn't work out too well, just pulling a premade realsense/ros docker at this point
    - https://github.com/2b-t/realsense-ros2-docker/tree/main; just followed this

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
When trying to patch the Jetpack 6 HID kernal issue for the realsense, we get this error:
"
/home/orin/realsense_mipi_platform_driver/l4t-gcc/6.0/bin/aarch64-buildroot-linux-gnu-gcc: unknown compiler
scripts/Kconfig.include:44: Sorry, this compiler is not supported.
make[2]: *** [scripts/kconfig/Makefile:87: defconfig] Error 1
make[1]: *** [Makefile:630: defconfig] Error 2
make[1]: Leaving directory '/home/orin/realsense_mipi_platform_driver/sources_6.0/kernel/kernel-jammy-src'
make: *** [Makefile:24: kernel] Error 2
make: Leaving directory '/home/orin/realsense_mipi_platform_driver/sources_6.0/kernel'
"
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
