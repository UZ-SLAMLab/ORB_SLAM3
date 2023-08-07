# ORB-SLAM3

# 3. Building ORB-SLAM3 library and examples

First, clone the repository:
```
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
```
**Android:**
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

This will build ORB_SLAM3 for android and will the necessary shared objects and headers to the appropriate locations in DemoApp.

**Linux:**
Refer to ORB_SLAM3_README.md

