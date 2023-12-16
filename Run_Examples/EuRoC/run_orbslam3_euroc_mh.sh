#!/bin/bash
# Set paths
pathVocabulary='/home/justmohsen/Documents/SLAM/ORBSLAM/Vocabulary/ORBvoc.txt'
pathDatasetEuroc='/home/justmohsen/Documents/SLAM/Datasets/euroc/MachineHall'
pathEurocRun='/home/justmohsen/Documents/SLAM/ORBSLAM/Run_Examples/EuRoC'
pathMonocularResults='/home/justmohsen/Documents/SLAM/Datasets/euroc/MachineHall/ORBSLAM3_Run/monocular'
pathStereoResults='/home/justmohsen/Documents/SLAM/Datasets/euroc/MachineHall/ORBSLAM3_Run/stereo'
pathMonocularInertialResults='/home/justmohsen/Documents/SLAM/Datasets/euroc/MachineHall/ORBSLAM3_Run/monocular-inertial'
pathStereoInertialResults='/home/justmohsen/Documents/SLAM/Datasets/euroc/MachineHall/ORBSLAM3_Run/stereo-inertial'
# Create directories
mkdir /home/justmohsen/Documents/SLAM/Datasets/euroc/MachineHall/ORBSLAM3_Run/monocular/MH01
mkdir /home/justmohsen/Documents/SLAM/Datasets/euroc/MachineHall/ORBSLAM3_Run/monocular/MH02
mkdir /home/justmohsen/Documents/SLAM/Datasets/euroc/MachineHall/ORBSLAM3_Run/monocular/MH03
mkdir /home/justmohsen/Documents/SLAM/Datasets/euroc/MachineHall/ORBSLAM3_Run/monocular/MH04
mkdir /home/justmohsen/Documents/SLAM/Datasets/euroc/MachineHall/ORBSLAM3_Run/monocular/MH05
mkdir /home/justmohsen/Documents/SLAM/Datasets/euroc/MachineHall/ORBSLAM3_Run/monocular/MH01_to_MH05
mkdir /home/justmohsen/Documents/SLAM/Datasets/euroc/MachineHall/ORBSLAM3_Run/stereo/MH01
mkdir /home/justmohsen/Documents/SLAM/Datasets/euroc/MachineHall/ORBSLAM3_Run/stereo/MH02
mkdir /home/justmohsen/Documents/SLAM/Datasets/euroc/MachineHall/ORBSLAM3_Run/stereo/MH03
mkdir /home/justmohsen/Documents/SLAM/Datasets/euroc/MachineHall/ORBSLAM3_Run/stereo/MH04
mkdir /home/justmohsen/Documents/SLAM/Datasets/euroc/MachineHall/ORBSLAM3_Run/stereo/MH05
mkdir /home/justmohsen/Documents/SLAM/Datasets/euroc/MachineHall/ORBSLAM3_Run/stereo/MH01_to_MH05
mkdir /home/justmohsen/Documents/SLAM/Datasets/euroc/MachineHall/ORBSLAM3_Run/monocular-inertial/MH01
mkdir /home/justmohsen/Documents/SLAM/Datasets/euroc/MachineHall/ORBSLAM3_Run/monocular-inertial/MH02
mkdir /home/justmohsen/Documents/SLAM/Datasets/euroc/MachineHall/ORBSLAM3_Run/monocular-inertial/MH03
mkdir /home/justmohsen/Documents/SLAM/Datasets/euroc/MachineHall/ORBSLAM3_Run/monocular-inertial/MH04
mkdir /home/justmohsen/Documents/SLAM/Datasets/euroc/MachineHall/ORBSLAM3_Run/monocular-inertial/MH05
mkdir /home/justmohsen/Documents/SLAM/Datasets/euroc/MachineHall/ORBSLAM3_Run/monocular-inertial/MH01_to_MH05
mkdir /home/justmohsen/Documents/SLAM/Datasets/euroc/MachineHall/ORBSLAM3_Run/stereo-inertial/MH01
mkdir /home/justmohsen/Documents/SLAM/Datasets/euroc/MachineHall/ORBSLAM3_Run/stereo-inertial/MH02
mkdir /home/justmohsen/Documents/SLAM/Datasets/euroc/MachineHall/ORBSLAM3_Run/stereo-inertial/MH03
mkdir /home/justmohsen/Documents/SLAM/Datasets/euroc/MachineHall/ORBSLAM3_Run/stereo-inertial/MH04
mkdir /home/justmohsen/Documents/SLAM/Datasets/euroc/MachineHall/ORBSLAM3_Run/stereo-inertial/MH05
mkdir /home/justmohsen/Documents/SLAM/Datasets/euroc/MachineHall/ORBSLAM3_Run/stereo-inertial/MH01_to_MH05

#------------------------------------
# Monocular Examples
echo "Launching MH01 with Monocular sensor"
mkdir
"$pathEurocRun"/mono_euroc "$pathVocabulary" "$pathEurocRun"/EuRoC_mono.yaml "$pathDatasetEuroc"/MH01 "$pathEurocRun"/EuRoC_TimeStamps/MH01.txt "$pathMonocularResults"/MH01/results

echo "Launching MH02 with Monocular sensor"
"$pathEurocRun"/mono_euroc "$pathVocabulary" "$pathEurocRun"/EuRoC_mono.yaml "$pathDatasetEuroc"/MH02 "$pathEurocRun"/EuRoC_TimeStamps/MH02.txt "$pathMonocularResults"/MH02/results

echo "Launching MH03 with Monocular sensor"
"$pathEurocRun"/mono_euroc "$pathVocabulary" "$pathEurocRun"/EuRoC_mono.yaml "$pathDatasetEuroc"/MH03 "$pathEurocRun"/EuRoC_TimeStamps/MH03.txt "$pathMonocularResults"/MH03/results

echo "Launching MH04 with Monocular sensor"
"$pathEurocRun"/mono_euroc "$pathVocabulary" "$pathEurocRun"/EuRoC_mono.yaml "$pathDatasetEuroc"/MH04 "$pathEurocRun"/EuRoC_TimeStamps/MH04.txt "$pathMonocularResults"/MH04/results

echo "Launching MH05 with Monocular sensor"
"$pathEurocRun"/mono_euroc "$pathVocabulary" "$pathEurocRun"/EuRoC_mono.yaml "$pathDatasetEuroc"/MH05 "$pathEurocRun"/EuRoC_TimeStamps/MH05.txt "$pathMonocularResults"/MH05/results

# MultiSession Monocular Examples
echo "Launching Machine Hall with Monocular sensor"
"$pathEurocRun"/mono_euroc "$pathVocabulary" "$pathEurocRun"/EuRoC_mono.yaml "$pathDatasetEuroc"/MH01 "$pathEurocRun"/EuRoC_TimeStamps/MH01.txt "$pathDatasetEuroc"/MH02 "$pathEurocRun"/EuRoC_TimeStamps/MH02.txt "$pathDatasetEuroc"/MH03 "$pathEurocRun"/EuRoC_TimeStamps/MH03.txt "$pathDatasetEuroc"/MH04 "$pathEurocRun"/EuRoC_TimeStamps/MH04.txt "$pathDatasetEuroc"/MH05 "$pathEurocRun"/EuRoC_TimeStamps/MH05.txt "$pathMonocularResults"/MH01_to_MH05/results

#------------------------------------
# Stereo Examples
echo "Launching MH01 with Stereo sensor"
"$pathEurocRun"/stereo_euroc "$pathVocabulary" "$pathEurocRun"/EuRoC_Stereo.yaml "$pathDatasetEuroc"/MH01 "$pathEurocRun"/EuRoC_TimeStamps/MH01.txt "$pathStereoResults"/MH01/results

echo "Launching MH02 with Stereo sensor"
"$pathEurocRun"/stereo_euroc "$pathVocabulary" "$pathEurocRun"/EuRoC_Stereo.yaml "$pathDatasetEuroc"/MH02 "$pathEurocRun"/EuRoC_TimeStamps/MH02.txt "$pathStereoResults"/MH02/results

echo "Launching MH03 with Stereo sensor"
"$pathEurocRun"/stereo_euroc "$pathVocabulary" "$pathEurocRun"/EuRoC_Stereo.yaml "$pathDatasetEuroc"/MH03 "$pathEurocRun"/EuRoC_TimeStamps/MH03.txt "$pathStereoResults"/MH03/results

echo "Launching MH04 with Stereo sensor"
"$pathEurocRun"/stereo_euroc "$pathVocabulary" "$pathEurocRun"/EuRoC_Stereo.yaml "$pathDatasetEuroc"/MH04 "$pathEurocRun"/EuRoC_TimeStamps/MH04.txt "$pathStereoResults"/MH04/results

echo "Launching MH05 with Stereo sensor"
"$pathEurocRun"/stereo_euroc "$pathVocabulary" "$pathEurocRun"/EuRoC_Stereo.yaml "$pathDatasetEuroc"/MH05 "$pathEurocRun"/EuRoC_TimeStamps/MH05.txt "$pathStereoResults"/MH05/results

# MultiSession Stereo Examples
echo "Launching Machine Hall with Stereo sensor"
"$pathEurocRun"/stereo_euroc "$pathVocabulary" "$pathEurocRun"/EuRoC_Stereo.yaml "$pathDatasetEuroc"/MH01 "$pathEurocRun"/EuRoC_TimeStamps/MH01.txt "$pathDatasetEuroc"/MH02 "$pathEurocRun"/EuRoC_TimeStamps/MH02.txt "$pathDatasetEuroc"/MH03 "$pathEurocRun"/EuRoC_TimeStamps/MH03.txt "$pathDatasetEuroc"/MH04 "$pathEurocRun"/EuRoC_TimeStamps/MH04.txt "$pathDatasetEuroc"/MH05 "$pathEurocRun"/EuRoC_TimeStamps/MH05.txt "$pathStereoResults"/MH01_to_MH05/results

#------------------------------------
# Monocular-Inertial Examples
echo "Launching MH01 with Monocular-Inertial sensor"
"$pathEurocRun"/mono_inertial_euroc "$pathVocabulary" "$pathEurocRun"/EuRoC_Mono_Inertial.yaml "$pathDatasetEuroc"/MH01 "$pathEurocRun"/EuRoC_TimeStamps/MH01.txt "$pathMonocularInertialResults"/MH01/results

echo "Launching MH02 with Monocular-Inertial sensor"
"$pathEurocRun"/mono_inertial_euroc "$pathVocabulary" "$pathEurocRun"/EuRoC_Mono_Inertial.yaml "$pathDatasetEuroc"/MH02 "$pathEurocRun"/EuRoC_TimeStamps/MH02.txt "$pathMonocularInertialResults"/MH02/results

echo "Launching MH03 with Monocular-Inertial sensor"
"$pathEurocRun"/mono_inertial_euroc "$pathVocabulary" "$pathEurocRun"/EuRoC_Mono_Inertial.yaml "$pathDatasetEuroc"/MH03 "$pathEurocRun"/EuRoC_TimeStamps/MH03.txt "$pathMonocularInertialResults"/MH03/results

echo "Launching MH04 with Monocular-Inertial sensor"
"$pathEurocRun"/mono_inertial_euroc "$pathVocabulary" "$pathEurocRun"/EuRoC_Mono_Inertial.yaml "$pathDatasetEuroc"/MH04 "$pathEurocRun"/EuRoC_TimeStamps/MH04.txt "$pathMonocularInertialResults"/MH04/results

echo "Launching MH05 with Monocular-Inertial sensor"
"$pathEurocRun"/mono_inertial_euroc "$pathVocabulary" "$pathEurocRun"/EuRoC_Mono_Inertial.yaml "$pathDatasetEuroc"/MH05 "$pathEurocRun"/EuRoC_TimeStamps/MH05.txt "$pathMonocularInertialResults"/MH05/results

# MultiSession Monocular Examples
echo "Launching Machine Hall with Monocular-Inertial sensor"
"$pathEurocRun"/mono_inertial_euroc "$pathVocabulary" "$pathEurocRun"/EuRoC_Mono_Inertial.yaml "$pathDatasetEuroc"/MH01 "$pathEurocRun"/EuRoC_TimeStamps/MH01.txt "$pathDatasetEuroc"/MH02 "$pathEurocRun"/EuRoC_TimeStamps/MH02.txt "$pathDatasetEuroc"/MH03 "$pathEurocRun"/EuRoC_TimeStamps/MH03.txt "$pathDatasetEuroc"/MH04 "$pathEurocRun"/EuRoC_TimeStamps/MH04.txt "$pathDatasetEuroc"/MH05 "$pathEurocRun"/EuRoC_TimeStamps/MH05.txt "$pathMonocularInertialResults"/MH01_to_MH05/results

#------------------------------------
# Stereo-Inertial Examples
echo "Launching MH01 with Stereo-Inertial sensor"
"$pathEurocRun"/stereo_inertial_euroc "$pathVocabulary" "$pathEurocRun"/EuRoC_Stereo_Inertial.yaml "$pathDatasetEuroc"/MH01 "$pathEurocRun"/EuRoC_TimeStamps/MH01.txt "$pathStereoInertialResults"/MH01/results

echo "Launching MH02 with Stereo-Inertial sensor"
"$pathEurocRun"/stereo_inertial_euroc "$pathVocabulary" "$pathEurocRun"/EuRoC_Stereo_Inertial.yaml "$pathDatasetEuroc"/MH02 "$pathEurocRun"/EuRoC_TimeStamps/MH02.txt "$pathStereoInertialResults"/MH02/results

echo "Launching MH03 with Stereo-Inertial sensor"
"$pathEurocRun"/stereo_inertial_euroc "$pathVocabulary" "$pathEurocRun"/EuRoC_Stereo_Inertial.yaml "$pathDatasetEuroc"/MH03 "$pathEurocRun"/EuRoC_TimeStamps/MH03.txt "$pathStereoInertialResults"/MH03/results

echo "Launching MH04 with Stereo-Inertial sensor"
"$pathEurocRun"/stereo_inertial_euroc "$pathVocabulary" "$pathEurocRun"/EuRoC_Stereo_Inertial.yaml "$pathDatasetEuroc"/MH04 "$pathEurocRun"/EuRoC_TimeStamps/MH04.txt "$pathStereoInertialResults"/MH04/results

echo "Launching MH05 with Stereo-Inertial sensor"
"$pathEurocRun"/stereo_inertial_euroc "$pathVocabulary" "$pathEurocRun"/EuRoC_Stereo_Inertial.yaml "$pathDatasetEuroc"/MH05 "$pathEurocRun"/EuRoC_TimeStamps/MH05.txt "$pathStereoInertialResults"/MH05/results

# MultiSession Stereo-Inertial Examples
echo "Launching Machine Hall with Stereo-Inertial sensor"
"$pathEurocRun"/stereo_inertial_euroc "$pathVocabulary" "$pathEurocRun"/EuRoC_Stereo_Inertial.yaml "$pathDatasetEuroc"/MH01 "$pathEurocRun"/EuRoC_TimeStamps/MH01.txt "$pathDatasetEuroc"/MH02 "$pathEurocRun"/EuRoC_TimeStamps/MH02.txt "$pathDatasetEuroc"/MH03 "$pathEurocRun"/EuRoC_TimeStamps/MH03.txt "$pathDatasetEuroc"/MH04 "$pathEurocRun"/EuRoC_TimeStamps/MH04.txt "$pathDatasetEuroc"/MH05 "$pathEurocRun"/EuRoC_TimeStamps/MH05.txt "$pathStereoInertialResults"/MH01_to_MH05/results
