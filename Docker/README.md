# **ORB-SLAM 3** image

This images contains a pre-installed ORB-SLAM 3 in */dpds/ORB_SLAM3*. See on Docker Hub [lmwafer/orb-slam-3-ready](https://hub.docker.com/r/lmwafer/orb-slam-3-ready). 

It is based on Docker image realsense-ready to use Intel RealSense 2 SDK and cameras. See on Docker Hub [lmwafer/realsense-ready](https://hub.docker.com/r/lmwafer/realsense-ready/tags). 

## Image prerequisites

- Docker (tested with Docker 20.10.7), see [Install Docker Engine](https://docs.docker.com/engine/install/)

- Docker Compose (tested with Docker Compose 1.29.2), see [Install Docker Compose](https://docs.docker.com/compose/install/)
  You may have a `/usr/local/bin/docker-compose: no such file or directory` error. In this case, use
  ```bash
  sudo mkdir /usr/local/bin/docker-compose
  ```
  before restarting the installation process

- Nvidia Container Toolkit (tested with ubuntu20.04 distribution), see [NVIDIA Container Toolkit Installation Guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

- The `device id` parameter in **orb-slam/docker-compose.yml** may take another number on different machines. Use
  ```bash
  lshw -c display
  ```
  to get the id of your GPU. See [Enabling GPU access with Compose](https://docs.docker.com/compose/gpu-support/)

## Image installation

The tag may be outdated. See on [Dockerhub](https://hub.docker.com/r/lmwafer/orb-slam-3-ready/tags).

```bash
docker pull lmwafer/orb-slam-3-ready:1.0-ubuntu18.04
```

## Image usage

All the commands need to be run in **Docker** directory. 

Get inside a freshly new container (basically `up` + `enter`)
```bash
make
```

Start an *orb-3-container* (uses **orb-container/docker-compose.yml**)
```bash
make up-orb
```

Enter running *orb-3-container*
```bash
make enter-orb
```

Stop running *orb-3-container* (and removes it, add a *rw* volume in *docker-compose.yml* to save data)
```bash
make down-orb
```

Build *orb-slam-3-ready* image (uses **orb-container/Dockerfile**)
```bash
make build-orb
```

# **Realsense** image

This images contains a pre-installed Intel Realsense 2 SDK in */dpds/librealsense-2.50.0*. See on Docker Hub [lmwafer/realsense-ready](https://hub.docker.com/r/lmwafer/realsense-ready). 

## Image prerequisites

Nothing except an Internet connexion and a GPU !

## Image installation

The tag may be outdated. See on [Dockerhub](https://hub.docker.com/r/lmwafer/realsense-ready/tags).

```bash
docker pull lmwafer/realsense-ready:2.0-ubuntu18.04
```

## Image usage

All the commands need to be run in **Docker** directory. 

Start a container (uses **realsense-container/docker-compose.yml**)
```bash
make up-realsense
```

Enter running *realsense-container*
```bash
make enter-realsense
```

Stop running *realsense-container* (and removes it, add a *rw* volume in *docker-compose.yml* to save data)
```bash
make down-realsense
```

Build *realsense-ready* image (uses **realsense-container/Dockerfile**)
```bash
make build-realsense
```