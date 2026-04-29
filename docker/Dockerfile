FROM osrf/ros:noetic-desktop-full

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        wget \
        ca-certificates \
        pkg-config \
        libarmadillo-dev \
        libeigen3-dev \
        libompl-dev \
        libdw-dev \
        libgoogle-glog-dev \
        libpcl-dev \
        ros-noetic-pcl-ros \
        ros-noetic-pcl-conversions \
        ros-noetic-tf \
        ros-noetic-tf2-ros \
        ros-noetic-cv-bridge \
        ros-noetic-image-transport \
        ros-noetic-message-filters \
        ros-noetic-dynamic-reconfigure \
        ros-noetic-rviz \
        ros-noetic-xacro \
        python3-catkin-tools \
        python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# --- nlopt v2.7.1 from source ---
RUN cd /tmp \
 && git clone -b v2.7.1 --depth 1 https://github.com/stevengj/nlopt.git \
 && cd nlopt && mkdir build && cd build \
 && cmake .. \
 && make -j"$(nproc)" \
 && make install \
 && ldconfig \
 && rm -rf /tmp/nlopt

# --- LKH-3.0.6 ---
RUN cd /tmp \
 && wget -q http://akira.ruc.dk/~keld/research/LKH-3/LKH-3.0.6.tgz \
 && tar xfz LKH-3.0.6.tgz \
 && cd LKH-3.0.6 \
 && make -j"$(nproc)" \
 && cp LKH /usr/local/bin/ \
 && rm -rf /tmp/LKH-3.0.6*

# Catkin workspace skeleton — repo will be bind-mounted at /catkin_ws/src/RACER
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws

# Source ROS + (if built) devel on every shell
RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc \
 && echo '[ -f /catkin_ws/devel/setup.bash ] && source /catkin_ws/devel/setup.bash' >> /root/.bashrc

CMD ["/bin/bash"]
