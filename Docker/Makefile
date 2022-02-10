REALSENSE_DIRECTORY=realsense-container/
REALSENSE_IM_NAME=lmwafer/realsense-ready
REALSENSE_IM_TAG=2.0-ubuntu18.04
REALSENSE_CONT_NAME=realsense-container

ORB_DIRECTORY=orb-container/
ORB_IM_NAME=lmwafer/orb-slam-3-ready
ORB_IM_TAG=1.0-ubuntu18.04
ORB_CONT_NAME=orb-3-container # You will need to apply the exact same name to container_name in orb-container/docker-compose.yml

default: up-orb enter-orb


up-orb:
	sudo xhost +local:root && cd ${ORB_DIRECTORY} && sudo docker-compose up -d

enter-orb:
	clear && docker exec -it ${ORB_CONT_NAME} bash

down-orb:
	cd ${ORB_DIRECTORY} && docker-compose down

build-orb:
	cd ${ORB_DIRECTORY} && sudo docker build -t ${ORB_IM_NAME}:${ORB_IM_TAG} .


up-realsense:
	sudo xhost +local:root && cd ${REALSENSE_DIRECTORY} && sudo docker-compose up -d

enter-realsense:
	clear && docker exec -it ${REALSENSE_CONT_NAME} bash

down-realsense:
	cd ${REALSENSE_DIRECTORY} && docker-compose down

build-realsense:
	cd ${REALSENSE_DIRECTORY} && sudo docker build -t ${REALSENSE_IM_NAME}:${REALSENSE_READY_IM_TAG} .