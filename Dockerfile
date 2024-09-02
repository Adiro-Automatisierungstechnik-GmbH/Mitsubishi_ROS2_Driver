FROM moveit/moveit2:humble-release

SHELL ["/bin/bash","-c"]

ENV DEBIAN_FRONTEND=noninteractive

ENV CUDA_VERSION=12

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility

# Create local catkin workspace
ENV ROS_WS=/root/ros2_ws
RUN mkdir -p "$ROS_WS/src"
WORKDIR $ROS_WS

# Always source the workspaces
# Create the ros_entrypoint script
RUN echo '#!/bin/bash' > /root/ros_entrypoint.sh && \
    echo '' >> /root/ros_entrypoint.sh && \
    echo '# Source ROS distro environment and local catkin workspace' >> /root/ros_entrypoint.sh && \
    echo 'source "/opt/ros/$ROS_DISTRO/setup.bash" && source "$ROS_WS/install/setup.bash"' >> /root/ros_entrypoint.sh && \
    echo '' >> /root/ros_entrypoint.sh && \
    echo 'exec "$@"' >> /root/ros_entrypoint.sh && \        
    echo 'alias cb="cd ${ROS_WS} && colcon build && bash"' >> /root/ros_entrypoint.sh && \
    echo 'alias cc="cd ${ROS_WS} && rm -r build install log"' >> /root/ros_entrypoint.sh && \
    echo 'export RCUTILS_COLORIZED_OUTPUT=1' >> /root/.bashrc && \
    chmod +x /root/ros_entrypoint.sh && \    
    echo "source /root/ros_entrypoint.sh" >> /root/.bashrc

RUN apt-get update \ 
    && apt-get upgrade -y \
    && apt-get install -y ros-${ROS_DISTRO}-ros-gz ros-${ROS_DISTRO}-gazebo-ros2-control ros-${ROS_DISTRO}-gazebo-ros-pkgs

COPY src /root/ros2_ws/src 

RUN source /root/ros_entrypoint.sh \
    && cd ${ROS_WS} \
    && colcon build

ENTRYPOINT ["/root/ros_entrypoint.sh"]
CMD ["bash"]
