FROM mcr.microsoft.com/devcontainers/cpp:1.0.0-debian-11

ARG REINSTALL_CMAKE_VERSION_FROM_SOURCE="none"
COPY ./reinstall-cmake.sh /tmp/
RUN if [ "${REINSTALL_CMAKE_VERSION_FROM_SOURCE}" != "none" ]; then \
        chmod +x /tmp/reinstall-cmake.sh && /tmp/reinstall-cmake.sh ${REINSTALL_CMAKE_VERSION_FROM_SOURCE}; \
    fi \
    && rm -f /tmp/reinstall-cmake.sh

# Install Eigen 3
RUN apt-get update
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends libeigen3-dev

# Install OpenCV 3.3
RUN DEBIAN_FRONTEND=noninteractive sudo apt-get install -y --no-install-recommends build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
RUN git clone https://github.com/opencv/opencv.git /tmp/opencv && \
    cd /tmp/opencv && \
    git checkout 3.3.0 && \
    mkdir build && \
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local .. -D BUILD_opencv_dnn=OFF -D BUILD_opencv_videoio=OFF && \
    make -j$(nproc) && \
    sudo make install
