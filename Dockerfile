FROM ubuntu:jammy

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# 1. Dependencias básicas
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    locales \
    software-properties-common \
    python3-pip \
    git \
    libreadline-dev \
    openjdk-17-jdk \
    openjdk-17-jre \
    wget \
    ca-certificates && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# 2. Clave GPG y repositorio de ROS 2
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list && \
    apt-get update

# 3. Instalar ROS 2 Humble
RUN apt-get install -y ros-humble-desktop ros-humble-behaviortree-cpp-v3 ros-humble-test-msgs ros-humble-qt-gui-cpp ros-humble-rqt-gui-cpp ros-humble-navigation2 ros-humble-nav2-bringup

# 4. Instalar herramientas de desarrollo ROS 2
RUN apt-get install -y \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    python3-argcomplete

# 5. Inicializar rosdep
RUN rosdep init && rosdep update

# 6. Instalar Python tools para UPF4ROS2
RUN pip install --upgrade pip && \
    pip install \
        setuptools \
        typing_extensions==4.7.1 \
        ConfigSpace && \
    pip install --pre 'unified-planning[pyperplan,tamer]'

RUN pip install --upgrade pip scipy


# 7. Crear workspace y clonar UPF4ROS2
WORKDIR /workspace
RUN mkdir -p src && cd src && \
    git clone https://github.com/PlanSys2/UPF4ROS2.git && \
    vcs import . < UPF4ROS2/upf.repos

# 8. Resolver dependencias con rosdep
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y

# 9. Compilar
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install

# 10. Configurar entorno
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /workspace/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
