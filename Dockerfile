#########################################
# This is the dockerfile for DEPLOY     #
# Please make sure you network is good  #
#########################################
FROM ros:humble-perception-jammy

# source ros environment
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source /home/sentry_ws/install/setup.bash" >> ~/.bashrc
RUN echo "force_color_prompt=yes" >> ~/.bashrc

RUN apt-get update && apt-get install -y \
    # for using add-apt-repository
    software-properties-common \
    # for download 
    wget python3-pip git-lfs \ 
    # useful tools
    aptitude usbutils inetutils-ping\
    # ament
    ros-$ROS_DISTRO-ament-cmake \
    python3-rosdep \
    openssh-server vim libnet-ping-perl libnet-ifconfig-wrapper-perl

# Install dependencies
RUN apt-get update && apt-get install -y \
    # for far-planner simulation environment
    libusb-dev unzip ros-$ROS_DISTRO-sdl2-vendor  \
    ros-$ROS_DISTRO-joint-state-publisher \
    # rviz2
    ros-$ROS_DISTRO-rviz2 \
    # rqt
    ros-$ROS_DISTRO-rqt ros-humble-rqt-robot-steering\
    ros-$ROS_DISTRO-rqt-common-plugins ros-$ROS_DISTRO-rqt-tf-tree \
    # navigation2
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-nav2-map-server \
    # octomap dependencies
    ros-$ROS_DISTRO-octomap ros-$ROS_DISTRO-octomap-msgs ros-$ROS_DISTRO-octomap-mapping \
    ros-$ROS_DISTRO-octomap-ros ros-$ROS_DISTRO-octomap-rviz-plugins ros-$ROS_DISTRO-octomap-server\
    # octomap viewer
    octovis \
    # gridmap dependencies
    # if you are learning gridmap, you may need to install the following packages
    # ros-$ROS_DISTRO-grid-map-demos
    # ros-$ROS_DISTRO-grid-map ros-$ROS_DISTRO-grid-map-cmake-helpers ros-$ROS_DISTRO-grid-map-core \
    # ros-$ROS_DISTRO-grid-map-costmap-2d ros-$ROS_DISTRO-grid-map-cv ros-$ROS_DISTRO-grid-map-filters \
    # ros-$ROS_DISTRO-grid-map-loader ros-$ROS_DISTRO-grid-map-msgs ros-$ROS_DISTRO-grid-map-octomap \
    # ros-$ROS_DISTRO-grid-map-pcl ros-$ROS_DISTRO-grid-map-ros ros-$ROS_DISTRO-grid-map-rviz-plugin \
    # ros-$ROS_DISTRO-grid-map-sdf ros-$ROS_DISTRO-grid-map-visualization \
    # for control
    ros-$ROS_DISTRO-teleop-twist-keyboard ros-$ROS_DISTRO-serial-driver \
    # # for lio-sam
    # libgeographic-dev ros-humble-gtsam \
    # visualization
    ros-$ROS_DISTRO-foxglove-bridge ros-$ROS_DISTRO-foxglove-msgs \
    # driver 
    ros-humble-serial-driver 

RUN pip3 install --upgrade pip
RUN pip3 install \
    -i https://pypi.tuna.tsinghua.edu.cn/simple rosdepc

WORKDIR /home/sentry_ws

# Groot2 for behaviour tree visualization
# IF YOU MEET PROBLEM IN THIS LINE, PLEASE CHECK https://www.behaviortree.dev/groot/ for the latest download link
# RUN wget https://s3.us-west-1.amazonaws.com/download.behaviortree.dev/groot2_linux_installer/Groot2-v1.6.1-linux-installer.run
RUN echo "alias groot2='~/Groot2/bin/groot2'" >> ~/.bashrc

# Add source code into images and build
ADD assert /home/sentry_ws/src/assert
ADD auto_aim_interfaces /home/sentry_ws/src/auto_aim_interfaces
ADD autonomous_exploration_development_environment /home/sentry_ws/src/autonomous_exploration_development_environment
ADD BehaviorTree.CPP /home/sentry_ws/src/BehaviorTree.CPP
ADD cmd_chassis /home/sentry_ws/src/cmd_chassis
ADD control_panel /home/sentry_ws/src/control_panel
ADD FAST_LIO /home/sentry_ws/src/FAST_LIO
ADD icp_relocalization /home/sentry_ws/src/icp_relocalization
ADD livox_ros_driver2 /home/sentry_ws/src/livox_ros_driver2
ADD nav2_plugins /home/sentry_ws/src/nav2_plugins
ADD performance_analysis /home/sentry_ws/src/performance_analysis
ADD rm_decision_cpp /home/sentry_ws/src/rm_decision_cpp
ADD rm_interfaces /home/sentry_ws/src/rm_interfaces
ADD sentry_bringup /home/sentry_ws/src/sentry_bringup
ADD sentry_description /home/sentry_ws/src/sentry_description
ADD .git/ /home/sentry_ws/src/.git/
ADD README.md /home/sentry_ws/src/README.md
ADD DevcontainterGuide.md /home/sentry_ws/src/DevcontainterGuide.md

# build
RUN rosdepc init && rosdepc update && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    rosdepc install -y --from-paths src --ignore-src -r -y --rosdistro $ROS_DISTRO

# build dependencies first
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select livox_ros_driver2 rm_interfaces behaviortree_cpp

# build the rest
RUN . /opt/ros/$ROS_DISTRO/setup.sh && . /home/sentry_ws/install/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# add start script
CMD /bin/bash
