FROM ros:humble-ros-base

# Instalar herramientas de video y dependencias de OpenCV
RUN apt-get update && apt-get install -y \
    python3-opencv \
    ros-humble-v4l2-camera \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

# Configurar el espacio de trabajo
WORKDIR /ros2_ws

# Configurar el entorno para que ROS se active automáticamente
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc