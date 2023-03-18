# This is a Docker file to build a Docker image with ORB-SLAM 3 and all its dependencies pre-installed
# For more info about ORB-SLAM 3 dependencies go check https://github.com/UZ-SLAMLab/ORB_SLAM3
FROM ubuntu:18.04

    #-[] Install dependencies
RUN apt-get update && \
    apt-get -y upgrade && \

    #-> Install general usage dependencies
    echo "Installing general usage dependencies ..." && \
    apt-get install -y apt-file && \
    apt-file update && \
    apt-get install -y nano \
    pkg-config && \

    #-> Install OpenCV dependencies
    #-? From : http://techawarey.com/programming/install-opencv-c-c-in-ubuntu-18-04-lts-step-by-step-guide/
    echo "Installing OpenCV dependencies ..." && \
    apt-get install -y\
    libgtk2.0-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    software-properties-common && \

    #-> Install Pangolin dependencies
    #-? From : https://cdmana.com/2021/02/20210204202321078t.html
    echo "Installing Pangolin dependencies ..." && \
    apt-get install -y \
    libglew-dev \
    libboost-dev \
    libboost-thread-dev \
    libboost-filesystem-dev \
    ffmpeg \
    libavutil-dev \
    libpng-dev && \

    #-> Install Eigen 3 last version
    #-? Needs to be installed BEFORE Pangolin as it also needs Eigen
    #-> Linear algebra library
    echo "Installing Eigen 3 last version ..." && \
    apt-get install -y libeigen3-dev && \
   
    #-> Install Pangolin last version
    #-? 3D Vizualisation tool
    #-? From : https://cdmana.com/2021/02/20210204202321078t.html
    echo "Installing Pangolin last version ..." && \
    cd /dpds/ && \
    git clone https://github.com/stevenlovegrove/Pangolin.git Pangolin && \
    cd /dpds/Pangolin/ && \
    mkdir build && \
    cd build/ && \
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -DCPP11_NO_BOOST=1 \
    /dpds/Pangolin/ && \
    make -j4 && \
    make install

    #-[] Install OpenCV last version
    #-? From : http://techawarey.com/programming/install-opencv-c-c-in-ubuntu-18-04-lts-step-by-step-guide/
    #-? Another RUN command in order to free memory
    #-? Usual computer vision library
RUN echo "Installing OpenCV last version ..." && \
    cd /dpds/ && \
    git clone https://github.com/Itseez/opencv.git opencv && \
    git clone https://github.com/Itseez/opencv_contrib.git opencv_contrib && \
    cd opencv/ && \
    mkdir build && \
    cd build/ && \
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D BUILD_TIFF=ON \
    -D WITH_CUDA=OFF \
    -D ENABLE_AVX=OFF \
    -D WITH_OPENGL=OFF \
    -D WITH_OPENCL=OFF \
    -D WITH_IPP=OFF \
    -D WITH_TBB=ON \
    -D BUILD_TBB=ON \
    -D WITH_EIGEN=ON \
    -D WITH_V4L=OFF \
    -D WITH_VTK=OFF \
    -D BUILD_TESTS=OFF \
    -D BUILD_PERF_TESTS=OFF \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D OPENCV_EXTRA_MODULES_PATH=/dpds/opencv_contrib/modules \
    /dpds/opencv/ && \
    make -j4 && \
    make install && \
    ldconfig

COPY . /dpds/ORB_SLAM3

RUN echo "Getting ORB-SLAM 3 installation ready ..." && \
    cd /dpds/ORB_SLAM3 && \
    ./build.sh
