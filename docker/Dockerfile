FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND noninteractive

ARG NUM_THREADS=1

# User setup
ARG USER=docker_mpc
ARG UID=1000
ARG GID=1000
ARG PW=user

# Add user and his password
RUN useradd -m ${USER} --uid=${UID} && \
    echo "${USER}:${PW}" | chpasswd && \
    usermod -s /bin/bash ${USER} && \
    usermod -a -G sudo ${USER}

#COPY context/ /context
#WORKDIR /context

# Install dependencies
RUN apt-get update && apt-get install -y \
    clang \
    cmake \
    g++ \
    git \
    google-mock \
    libboost-all-dev \
    libcairo2-dev \
    libcurl4-openssl-dev \
    libeigen3-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    liblua5.2-dev \
    libsuitesparse-dev \
    lsb-release \
    ninja-build \
    stow \
    tmux \
    tar \
    vim \
    python3-pip \
    python3-sphinx \
    libgmock-dev \
    ros-noetic-ros-numpy \
    ros-noetic-move-base-msgs && \
    rm -rf /var/lib/apt/lists/*

   
# Make symlink from python3 to python
RUN PYTHON3_PATH=`which python3` && \
    ln -s $PYTHON3_PATH `dirname $PYTHON3_PATH`/python

RUN mkdir -p /home/${USER}/catkin_ws/src && \
    chown -R ${UID}:${GID} /home/${USER}

# Install Acados
COPY /acados /home/${USER}/acados
RUN cd /home/${USER}/acados \
 && git submodule update --recursive --init \
 && rm -rf build \
 && mkdir build \
 && cd build \
 && cmake -DACADOS_WITH_QPOASES=ON -DACADOS_INSTALL_DIR="/home/docker_mpc/acados" .. \
 && make install -j4 \
 && cd /home/${USER}/acados && make shared_library \
 && cd /home/${USER}/acados && make examples_c \
 && pip3 install -e /home/${USER}/acados/interfaces/acados_template

WORKDIR /home/${USER}
USER ${UID}:${GID}
