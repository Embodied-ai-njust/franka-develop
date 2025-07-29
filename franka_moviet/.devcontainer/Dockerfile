FROM nagisa1201/frankahub:franka

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble

ARG USERNAME=Nagisa
ARG GROUP_NAME=wheel
# WORKDIR /home/$USERNAME/packages
# USER root
# # Set environment variables


# # Install essential packages and ROS development tools
# RUN apt-get update && \
#     apt-get install -y --no-install-recommends \
#         openssh-client \
#         python3-colcon-argcomplete \
#         python3-colcon-common-extensions \
#     && apt-get clean \
#     && rm -rf /var/lib/apt/lists/*

# # Setup user configuration
# RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc \
#     && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc

# RUN mkdir -p franka_ros2_ws/src && cd franka_ros2_ws/src && git clone https://github.com/frankarobotics/franka_ros2.git
# RUN cd /home/$USERNAME/packages && mkdir -p ws_moviet_mtc/src && cd ws_moviet_mtc/src && \
# git clone --branch humble https://github.com/moveit/moveit_task_constructor.git
# USER $USERNAME

# # Install some ROS 2 dependencies to create a cache layer,py-binding-tools和moviet-core是MTC需要的依赖项
# RUN sudo apt-get update \
#     && sudo apt-get install -y --no-install-recommends \
#         ros-humble-ros-gz \
#         ros-humble-sdformat-urdf \
#         ros-humble-joint-state-publisher-gui \
#         ros-humble-ros2controlcli \
#         ros-humble-controller-interface \
#         ros-humble-hardware-interface-testing \
#         ros-humble-ament-cmake-clang-format \
#         ros-humble-ament-cmake-clang-tidy \
#         ros-humble-controller-manager \
#         ros-humble-ros2-control-test-assets \
#         libignition-gazebo6-dev \
#         libignition-plugin-dev \
#         ros-humble-hardware-interface \
#         ros-humble-control-msgs \
#         ros-humble-backward-ros \
#         ros-humble-generate-parameter-library \
#         ros-humble-realtime-tools \
#         ros-humble-joint-state-publisher \
#         ros-humble-joint-state-broadcaster \
#         ros-humble-moveit-ros-move-group \
#         ros-humble-moveit-kinematics \
#         ros-humble-moveit-planners-ompl \
#         ros-humble-moveit-ros-visualization \
#         ros-humble-joint-trajectory-controller \
#         ros-humble-moveit-simple-controller-manager \
#         ros-humble-rviz2 \
#         ros-humble-xacro \
#         ros-$ROS_DISTRO-py-binding-tools \ 
#         ros-$ROS_DISTRO-moveit-core \
#     && sudo apt-get clean \
#     && sudo rm -rf /var/lib/apt/lists/*

# WORKDIR /home/$USERNAME/packages/franka_ros2_ws

# # Install the missing ROS 2 dependencies
# RUN sudo chown -R $USERNAME:$GROUP_NAME ./ \
#     && vcs import src < /home/$USERNAME/packages/franka_ros2_ws/src/franka_ros2/franka.repos --recursive --skip-existing \
#     && sudo apt-get update \
#     && rosdep update \
#     && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y \
#     && sudo apt-get clean \
#     && sudo rm -rf /var/lib/apt/lists/*
# RUN sudo chown -R $USERNAME:$GROUP_NAME /home/$USERNAME/packages/ws_moviet_mtc/  

# # 构建第一个工作空间
# RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
#     cd /home/$USERNAME/packages/franka_ros2_ws && \
#     colcon build

# # 构建第二个工作空间
# RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
#     cd /home/$USERNAME/packages/ws_moviet_mtc && \
#     colcon build

# RUN sudo chown -R $USERNAME:$GROUP_NAME /home/$USERNAME/.config
# # 统一配置所有工作空间的source顺序 (防止覆盖)
# RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc \
#     && echo "source /home/$USERNAME/packages/franka_ros2_ws/install/setup.bash" >> /home/$USERNAME/.bashrc \
#     && echo "source /home/$USERNAME/packages/ws_moviet_mtc/install/setup.bash" >> /home/$USERNAME/.bashrc \
#     && echo "export ROS_LOCALHOST_ONLY=1" >> /home/$USERNAME/.bashrc \
#     \
#     && echo "bass source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.config/fish/config.fish \
#     && echo "bass source /home/$USERNAME/packages/franka_ros2_ws/install/setup.bash" >> /home/$USERNAME/.config/fish/config.fish \
#     && echo "bass source /home/$USERNAME/packages/ws_moviet_mtc/install/setup.bash" >> /home/$USERNAME/.config/fish/config.fish \
#     && echo "set -x ROS_LOCALHOST_ONLY 1" >> /home/$USERNAME/.config/fish/config.fish

# # COPY ./franka_entrypoint.sh /franka_entrypoint.sh
# # RUN sudo chmod +x /franka_entrypoint.sh
# # RUN /franka_entrypoint.sh
# # RUN cd /home/$USERNAME/packages/franka_ros2_ws && colcon build 

# # Set the default shell to bash and the workdir to the source directory
# SHELL [ "/bin/bash", "-c" ]
CMD [ "/bin/bash" ]