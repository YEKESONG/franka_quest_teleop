# Quest 3S 遥操 FR3 —— 运行环境镜像
#
# 这份配方是把【真机上已跑通的 franka_dev 容器】反向还原出来的：
# 逐条对比了运行中容器与旧镜像的 apt/pip 差异，把当初手工 apt install 的包
# 全部固化进来，保证 clone 本仓库后 build 出的镜像 = 现在能跑的那个环境。
#
#   docker compose -f docker_launch_files/docker-compose.yml build
#   docker compose -f docker_launch_files/docker-compose.yml run --rm franka_dev
#
# 镜像里【不含】任何工作区源码：三个 ROS 工作区都从本仓库挂进 /docker_volume，
# 编译产物落在宿主机，容器随时可删可重建。

FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

### 基础工具链
RUN apt-get update -y && \
    apt-get install -y --no-install-recommends \
        git \
        cmake \
        build-essential \
        python3-pip \
        libeigen3-dev \
        libpoco-dev \
        libfmt-dev \
        librange-v3-dev \
        freeglut3-dev \
        usbutils \
        iproute2 \
        iputils-ping \
        net-tools \
        vim \
        less && \
    rm -rf /var/lib/apt/lists/*

### ROS 2 / ros2_control / MoveIt 基础
RUN apt-get update -y && \
    apt-get install -y --allow-unauthenticated \
        ros-humble-ament-cmake \
        ros-humble-ament-cmake-clang-format \
        ros-humble-ament-cmake-clang-tidy \
        ros-humble-angles \
        ros-humble-ros2-controllers \
        ros-humble-ros2-control \
        ros-humble-ros2-control-test-assets \
        ros-humble-controller-manager \
        ros-humble-control-msgs \
        ros-humble-control-toolbox \
        ros-humble-generate-parameter-library \
        ros-humble-joint-state-publisher \
        ros-humble-joint-state-publisher-gui \
        ros-humble-moveit \
        ros-humble-pinocchio \
        ros-humble-realtime-tools \
        ros-humble-xacro \
        ros-humble-hardware-interface \
        ros-humble-hardware-interface-testing \
        ros-humble-ros-gz && \
    rm -rf /var/lib/apt/lists/*

### MoveIt2 2.13.0 源码编译所需（apt 版 moveit 不带这些，缺了 colcon 会中途报错）
RUN apt-get update -y && \
    apt-get install -y --allow-unauthenticated \
        ros-humble-stomp \
        ros-humble-osqp-vendor \
        ros-humble-google-benchmark-vendor \
        ros-humble-ament-cmake-google-benchmark \
        ros-humble-warehouse-ros-sqlite \
        ros-humble-ros-testing \
        ros-humble-launch-pytest \
        ros-humble-topic-tools \
        ros-humble-rmf-utils \
        ros-humble-joy \
        ros-humble-teleop-twist-joy \
        ros-humble-teleop-twist-keyboard && \
    rm -rf /var/lib/apt/lists/*

### franka_ros2 v2.3.0 所需（franka_mobile_sensors 依赖 realsense / sick，缺了整包编不过）
RUN apt-get update -y && \
    apt-get install -y --allow-unauthenticated \
        ros-humble-realsense2-camera \
        ros-humble-realsense2-description \
        ros-humble-sick-safetyscanners2 \
        ros-humble-gz-ros2-control && \
    rm -rf /var/lib/apt/lists/*

### libfranka —— 版本必须对上机器人服务器版本
# FR3 服务器 version 10 要求 libfranka >= 0.20；装 0.13.x 会直接报
#   "Incompatible library version (server 10, library 7)"。
# 用 apt 的 ros-humble-libfranka（find_package(Franka) 能直接在 /opt/ros/humble 找到），
# 不再走旧配方里"从源码编 libfranka_v0.13.2 再 dpkg -i"那条路——那条路装出来的是错版本。
RUN apt-get update -y && \
    apt-get install -y --allow-unauthenticated \
        "ros-humble-libfranka=0.20.4-*" && \
    rm -rf /var/lib/apt/lists/*

### Quest 手柄读取：adb（桥接节点靠 adb logcat 取手柄位姿+按键）
RUN apt-get update -y && \
    apt-get install -y --no-install-recommends adb && \
    rm -rf /var/lib/apt/lists/*

### 桥接脚本的 Python 依赖
#   tf_transformations/transforms3d：四元数运算
#   pure-python-adb：ppadb，reader.py 用它连头显
RUN apt-get update -y && \
    apt-get install -y --allow-unauthenticated \
        ros-humble-tf-transformations \
        python3-transforms3d \
        python3-tk \
        python3-gi-cairo \
        python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/* && \
    pip3 install --no-cache-dir pure-python-adb==0.3.0.dev0 termcolor

WORKDIR /docker_volume

COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]
CMD ["/bin/bash"]

### 提醒：宿主机每次重启后要 xhost +local:docker，RViz 才能显示
