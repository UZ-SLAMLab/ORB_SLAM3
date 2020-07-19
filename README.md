# ORB-SLAM3

### V0.2: Beta version, 21 Jul 2020
**Authors:** [Carlos Campos], [Richard Elvira], [Juan J. Gómez], [José M. M. Montiel](http://webdiis.unizar.es/~josemari/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/).

ORB-SLAM3 is the first real time SLAM library able to perform **Visual, Visual-Inertial and Multi-Map SLAM** with **monocular, stereo and RGB-D** cameras, using **pin-hole and fisheye** lens models. In all sensor configurations, ORB-SLAM3 is as robust as the best systems available in the literature, and significantly more accurate. 

We provide examples to run ORB-SLAM3 system in the [EuRoC dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) using stereo or monocular with or without IMU, and in the [TUM-VI dataset](https://vision.in.tum.de/data/datasets/visual-inertial-dataset) using fisheye stereo or monocular with or without IMU.

This software is based on [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) developed by [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2)).


### Related Publications:

Carlos Campos, Richard Elvira, Juan J. Gómez, J. M. M. Montiel and Juan D. Tardós, **ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM**, Under review. **[PDF](https://arxiv.org/pdf)**.

[IMU-Initialization] Carlos Campos, J. M. M. Montiel and Juan D. Tardós, **Inertial-Only Optimization for Visual-Inertial Initialization**, *ICRA 2020*. **[PDF](https://arxiv.org/pdf/2003.05766.pdf)**

[ORBSLAM-Atlas] Richard Elvira, J. M. M. Montiel and Juan D. Tardós, **ORBSLAM-Atlas: a robust and accurate multi-map system**, *IROS 2019*. **[PDF](https://arxiv.org/pdf/1908.11585.pdf)**.

[ORBSLAM-VI] Raúl Mur-Artal, and Juan D. Tardós, **Visual-inertial monocular SLAM with map reuse**, IEEE Robotics and Automation Letters, vol. 2 no. 2, pp. 796-803, 2017. **[PDF](https://arxiv.org/pdf/1610.05949.pdf)**. 

[Stereo and RGB-D] Raúl Mur-Artal and Juan D. Tardós. **ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras**. *IEEE Transactions on Robotics,* vol. 33, no. 5, pp. 1255-1262, 2017. **[PDF](https://arxiv.org/pdf/1610.06475.pdf)**.

[Monocular] Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. **ORB-SLAM: A Versatile and Accurate Monocular SLAM System**. *IEEE Transactions on Robotics,* vol. 31, no. 5, pp. 1147-1163, 2015. (**2015 IEEE Transactions on Robotics Best Paper Award**). **[PDF](https://arxiv.org/pdf/1502.00956.pdf)**.

[DBoW2 Place Recognition] Dorian Gálvez-López and Juan D. Tardós. **Bags of Binary Words for Fast Place Recognition in Image Sequences**. *IEEE Transactions on Robotics,* vol. 28, no. 5, pp. 1188-1197, 2012. **[PDF](http://doriangalvez.com/php/dl.php?dlp=GalvezTRO12.pdf)**

# 1. License

ORB-SLAM3 is released under [GPLv3 license](https://github.com/UZ-SLAMLab/ORB_SLAM3/LICENSE). For a list of all code/library dependencies (and associated licenses), please see [Dependencies.md](https://github.com/UZ-SLAMLab/ORB_SLAM3/Dependencies.md).

For a closed-source version of ORB-SLAM3 for commercial purposes, please contact the authors: orbslam (at) unizar (dot) es.

If you use ORB-SLAM3 in an academic work, please cite:

    @article{Campos2020,
      title={{ORB-SLAM3}: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map {SLAM}},
      author={Campos, Carlos AND Elvira, Richard AND G\´omez, Juan J. AND Montiel, J. M. M. AND Tard\'os, Juan D.},
      journal={arXiv preprint arXiv:xxxx.yyyy},
      year={2020}
     }

# 2. Prerequisites
We have tested the library in **Ubuntu 16.04** and **18.04**, but it should be easy to compile in other platforms. A powerful computer (e.g. i7) will ensure real-time performance and provide more stable and accurate results.

## C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.

## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 3.0. Tested with OpenCV 3.2.0**.

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

# 3. Building ORB-SLAM3 library and examples

Clone the repository:
```
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
```

We provide a script `build.sh` to build the *Thirdparty* libraries and *ORB-SLAM3*. Please make sure you have installed all required dependencies (see section 2). Execute:
```
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```

This will create **libORB_SLAM3.so**  at *lib* folder and the executables in *Examples* folder.

# 4. Monocular Examples

## EuRoC Dataset(Pinhole)

1. Download a sequence (ASL format) from http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

2. Execute the following first command to execute as pure visual, or the second command to visual-inertial. Change PATH_TO_SEQUENCE_FOLDER and SEQUENCE according to the sequence you want to run.
```
./Examples/Monocular/mono_euroc Vocabulary/ORBvoc.txt Examples/Monocular/EuRoC.yaml PATH_TO_SEQUENCE_FOLDER/mav0/cam0/data Examples/Monocular/EuRoC_TimeStamps/SEQUENCE.txt 
```

```
./Examples/Monocular-Inertial/mono_inertial_euroc Vocabulary/ORBvoc.txt Examples/Monocular-Inertial/EuRoC.yaml PATH_TO_SEQUENCE_FOLDER/cam0/data Examples/Monocular-Inertial/EuRoC_TimeStamps/SEQUENCE.txt PATH_TO_SEQUENCE_FOLDER/mav0/imu0/data.csv
```

## TUM-VI Dataset(Fisheye)

1. Download a sequence from https://vision.in.tum.de/data/datasets/visual-inertial-dataset and uncompress it.

2. Execute the following command to execute as monocular-inertial. Change PATH_TO_SEQUENCE_FOLDER and SEQUENCE according to the sequence you want to run.

```
./Examples/Monocular-Inertial/mono_inertial_tum Vocabulary/ORBvoc.txt Examples/Monocular-Inertial/TUM_512.yaml PATH_TO_SEQUENCE_FOLDER/mav0/cam0/data Examples/Monocular-Inertial/TUM_TimeStamps/SEQUENCE.txt Examples/Monocular-Inertial/TUM_IMU/SEQUENCE.txt
```

# 5. Stereo Examples

## EuRoC Dataset(Pinhole)

1. Download a sequence (ASL format) from http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

2. Execute the following first command to execute as pure visual , or the second command to visual-inertial. Change PATH_TO_SEQUENCE_FOLDER and SEQUENCE according to the sequence you want to run.
```
./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml PATH_TO_SEQUENCE_FOLDER/mav0/cam0/data PATH_TO_SEQUENCE_FOLDER/mav0/cam1/data Examples/Stereo/EuRoC_TimeStamps/SEQUENCE.txt
```
```
./Examples/Stereo-Inertial/stereo_inertial_euroc Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/EuRoC.yaml PATH_TO_SEQUENCE_FOLDER/cam0/data PATH_TO_SEQUENCE_FOLDER/cam1/data Examples/Stereo-Inertial/EuRoC_TimeStamps/SEQUENCE.txt PATH_TO_SEQUENCE_FOLDER/mav0/imu0/data.csv
```

## TUM-VI Dataset(Fisheye)

1. Download a sequence from https://vision.in.tum.de/data/datasets/visual-inertial-dataset and uncompress it.

2. Execute the following command to execute as monocular-inertial. Change PATH_TO_SEQUENCE_FOLDER and SEQUENCE according to the sequence you want to run.

```
./Examples/Stereo-Inertial/stereo_inertial_tum Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/TUM_512.yaml PATH_TO_SEQUENCE_FOLDER/mav0/cam0/data PATH_TO_SEQUENCE_FOLDER/mav0/cam1/data Examples/Stereo-Inertial/TUM_TimeStamps/SEQUENCE.txt Examples/Stereo-Inertial/TUM_IMU/SEQUENCE.txt
``` 

# 6. Multiple-Map Examples

There is an executable to process a set of dataset sequentially, each dataset starts from scratch a new map. 

## EuRoC Monocular

1. Download a sequence (ASL format) from http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

2. Execute the following command to execute for N dataset. Change PATH_TO_SEQUENCE_FOLDER_i and SEQUENCE_i according to the sequence you want to run.


## EuRoC Monocular-Inertial
1. Download a sequence (ASL format) from http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

2. Execute the following command to execute for N dataset. Change PATH_TO_SEQUENCE_FOLDER_i and SEQUENCE_i according to the sequence you want to run.

```
./Examples/Monocular-Inertial/mono_inertial_euroc_2 Vocabulary/ORBvoc.txt Examples/Monocular-Inertial/EuRoC.yaml PATH_TO_SEQUENCE_FOLDER_1/mav0/cam0/data Examples/Monocular-Inertial/EuRoC_TimeStamps/SEQUENCE_1.txt PATH_TO_SEQUENCE_FOLDER_1/mav0/imu0/data.csv PATH_TO_SEQUENCE_FOLDER_2/mav0/cam0/data Examples/Monocular-Inertial/EuRoC_TimeStamps/SEQUENCE_2.txt PATH_TO_SEQUENCE_FOLDER_2/mav0/imu0/data.csv ... PATH_TO_SEQUENCE_FOLDER_N/mav0/cam0/data Examples/Monocular-Inertial/EuRoC_TimeStamps/SEQUENCE_N.txt PATH_TO_SEQUENCE_FOLDER_N/mav0/imu0/data.csv
```

Examples to process the whole Machine Hall. Change PATH_TO_SEQUENCE_FOLDER to the path of the sequences:
```
./Examples/Monocular-Inertial/mono_inertial_euroc_2 Vocabulary/ORBvoc.txt Examples/Monocular-Inertial/EuRoC.yaml PATH_TO_SEQUENCE_FOLDER/MH01/mav0/cam0/data Examples/Monocular-Inertial/EuRoC_TimeStamps/MH01.txt PATH_TO_SEQUENCE_FOLDER/MH01/mav0/imu0/data.csv PATH_TO_SEQUENCE_FOLDER/MH02/mav0/cam0/data Examples/Monocular-Inertial/EuRoC_TimeStamps/MH02.txt PATH_TO_SEQUENCE_FOLDER/MH02/mav0/imu0/data.csv
PATH_TO_SEQUENCE_FOLDER/MH03/mav0/cam0/data Examples/Monocular-Inertial/EuRoC_TimeStamps/MH03.txt PATH_TO_SEQUENCE_FOLDER/MH03/mav0/imu0/data.csv PATH_TO_SEQUENCE_FOLDER/MH04/mav0/cam0/data Examples/Monocular-Inertial/EuRoC_TimeStamps/MH04.txt PATH_TO_SEQUENCE_FOLDER/MH04/mav0/imu0/data.csv PATH_TO_SEQUENCE_FOLDER/MH05/mav0/cam0/data Examples/Monocular-Inertial/EuRoC_TimeStamps/MH05.txt PATH_TO_SEQUENCE_FOLDER/MH05/mav0/imu0/data.csv
```

## EuRoC Stereo
1. Download a sequence (ASL format) from http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

2. Execute the following command to execute for N dataset. Change PATH_TO_SEQUENCE_FOLDER_i and SEQUENCE_i according to the sequence you want to run.

## EuRoC Stereo-Inertial
1. Download a sequence (ASL format) from http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

2. Execute the following command to execute for N dataset. Change PATH_TO_SEQUENCE_FOLDER_i and SEQUENCE_i according to the sequence you want to run.

## TUM-VI Monocular-Inertial

## TUM-VI Stereo-Inertial



