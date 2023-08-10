FROM ubuntu:22.04


# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    openjdk-11-jdk \
    wget \
    libxext6 \
    libxrender1 \
    libxtst6 \
    libxi6 \
    git\
    dbus-x11 \
    x11-xserver-utils\
    adb\
    sudo\
    pv\
    unzip\
    cmake\
    tar\
    make\
    bzip2\
    nano\
    g++


# Set up a non-root user
# RUN useradd -ms /bin/bash android
# RUN echo 'android ALL=NOPASSWD: /usr/bin/apt-get' >> /etc/sudoers

# Set up directories
RUN mkdir -p /home/android/ORB_SLAM3
RUN mkdir -p /home/android/DemoApp

# Download and extract ndk
RUN wget -P /home/android https://dl.google.com/android/repository/android-ndk-r25c-linux.zip
RUN unzip /home/android/android-ndk-r25c-linux.zip -d /home/android
RUN rm -f /home/android/android-ndk-r25c-linux.zip

# Create a Volume to be mapped to host's DemoApp
VOLUME [ "/home/android/DemoApp" ]

# Copy repo to Volume
COPY --chown=android . /home/android/ORB_SLAM3
#RUN chown -R android /home/android/ORB_SLAM3 
# USER android

WORKDIR /home/android/ORB_SLAM3

# Launch Android Studio
CMD ["/bin/bash"]
