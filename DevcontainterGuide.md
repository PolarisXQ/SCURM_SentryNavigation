# Docker Image Quick Start

## Get the image

### OPTION 1: Pull from Docker Hub

```bash
docker pull polarisxq/scurm:sentry_nav
```

### OPTION 2: Build from Dockerfile

```bash
cd SCURM_SentryNavigation
docker build -t scurm:sentry_nav .
```

## RUN 

This image support both DockerDesktop and Docker in Linux.

for Linux user, run

```bash
sudo xhost + && sudo docker run -dit --network=host --pid=host --privileged -v /dev:/dev -e DISPLAY=${DISPLAY} scurm:sentry_nav
```

for windows user, run

```shell
docker run -dit --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY=host.docker.internal:0.0 --network=host --pid=host --privileged polarisxq/scurm:sentry_nav
```

## Insatll Groot2

You must install groot2 manually inside the container.

```bash
chmod +x . Groot2-1.3.1-linux-installer.run
./Groot2-1.3.1-linux-installer.run
# There is a GUI installer, just follow it.
# Try to run groot2 in terminal, if it works, then it is installed successfully.
groot2
```
