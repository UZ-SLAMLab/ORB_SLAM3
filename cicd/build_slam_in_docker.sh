#!/bin/bash

sudo /home/android/ORB_SLAM3/android_build/clone_deps.sh    --git_https --clean
sudo /home/android/ORB_SLAM3/android_build/build_android.sh  --ndk_path /home/android/android-ndk-r25c --abi arm64-v8a --platform 30 --clean
