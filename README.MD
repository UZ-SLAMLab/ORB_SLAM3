# ORB-SLAM3

## Building ORB-SLAM3 library

First, clone the repository and checkout libmaster (main dev branch):
```
git clone https://github.com/LibbaXR/ORB_SLAM3.git
git checkout libmaster
```
### Build for Android on Linux:
Clone the dependencies, execute:
```
chmod +x ./android_build/clone_deps.sh
./android_build/clone_deps.sh
```

Then, to build ORB_SLAM3 and to copy the required files to DemoApp execute:
```
chmod +x ./android_build/build_android.sh
./android_build/build_android.sh --app_path PATH_TO_DEMOAPP
```
Note: app_path is the only required option. to learn about other options related to android build run with --help.

This will build ORB_SLAM3 for android and will copy the necessary shared objects and headers to the appropriate locations in DemoApp.

### Build for Android with Docker:
Build and run container, execute:
```
docker build -f ./Utils/Docker/Dockerfile -t slam .
docker run --name slam -v "Host path to DemoApp":/home/android/DemoApp -v "Host path to ORB_SLAM3":/home/android/ORB_SLAM3 -it slam
```
Replace "Host path to DemoApp" and "Host path to ORB_SLAM3" with your host's local path, example:
```
docker run --name slam -v ~/Desktop/Repos/DemoApp:/home/android/DemoApp -v ~/Desktop/Repos/ORB_SLAM3:/home/android/ORB_SLAM3 -it slam
```


Then, in the container bash execute:
```
android_build/clone_deps.sh --git_https --clean
android_build/build_android.sh --app_path /home/android/DemoApp --ndk_path /home/android/android-ndk-r25c --abi arm64-v8a --platform 30 --clean
```

### Build for Linux:
Refer to ORB_SLAM3_README.md

