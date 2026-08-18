# --- Stage 1: Build Stage ---
FROM ubuntu:22.04 AS builder

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# 必要なツールとROS 2リポジトリの完全なセットアップ
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release ca-certificates git python3-pip \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake \
    ros-humble-ros-base \
    ros-humble-pcl-msgs \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-rviz2 \
    python3-colcon-common-extensions python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# ワークスペース構築
WORKDIR /ws/src
RUN git clone --depth 1 https://github.com/Hokuyo-aut/hokuyo_rsf.git && \
    git clone --depth 1 https://github.com/hokuyo-rd-release/nmea_msgs.git

# 依存関係の解決とビルド
WORKDIR /ws
RUN rosdep init || true && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro $ROS_DISTRO && \
    . /opt/ros/humble/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# --- Stage 2: Runtime Stage (Minimal) ---
FROM ubuntu:22.04

ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive
ENV QT_X11_NO_MITSHM=1

# 実行環境でもROSリポジトリを登録し、必要なランタイムパッケージのみをインストール
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release ca-certificates \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-pcl-msgs \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-rviz2 \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    && apt-get autoremove -y && apt-get clean && rm -rf /var/lib/apt/lists/*

# ビルド成果物をコピー
COPY --from=builder /ws/install /ws/install

# ROS環境設定
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ws/install/setup.bash" >> ~/.bashrc

WORKDIR /ws