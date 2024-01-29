FROM ros:noetic
SHELL ["/bin/bash", "-c"] 

# Some useful packages
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
    python3-catkin-tools \
    git

####################################################################################################
##################################### BUILDING ARIA FROM SOURCE  ###################################
####################################################################################################

WORKDIR /aria
RUN git clone https://github.com/moshulu/aria-legacy/ && \
    mv aria-legacy/ Aria && \
    cd Aria && \
    make && \
    make install 

####################################################################################################
###################################### ROS WORKSPACE & ROSARIA #####################################
####################################################################################################

WORKDIR /root/ros_ws/
RUN mkdir src && source /opt/ros/noetic/setup.bash && catkin init

RUN cd src && \
    git clone https://github.com/amor-ros-pkg/rosaria.git

RUN source /opt/ros/noetic/setup.bash && \
    DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
        ros-noetic-tf

RUN source /opt/ros/noetic/setup.bash && catkin build

#CMD /bin/bash
CMD source devel/setup.bash && rosrun rosaria RosAria _port:=${P3AT_USB_PORT} 