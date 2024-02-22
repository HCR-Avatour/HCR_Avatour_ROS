FROM ros:noetic
SHELL ["/bin/bash", "-c"] 

# Some useful packages
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
    python3-catkin-tools \
    git \
    ros-noetic-teleop-twist-keyboard \
    python3-pip

WORKDIR /root/ros_ws/
RUN mkdir src && source /opt/ros/noetic/setup.bash && catkin init

RUN source /opt/ros/noetic/setup.bash && \
    DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
    ros-noetic-tf

RUN source /opt/ros/noetic/setup.bash && catkin init && catkin build


# Add the following line to run motion_server.py
CMD python3 motion_server.py

