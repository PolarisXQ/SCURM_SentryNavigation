# How to use DevContainer and develop at any platform

👉[Read Me For Detailed Instructions.](https://polaris-notebook.readthedocs.io/zh-cn/latest/Docker/RunDevEnv.html#)

# RUN 

sudo xhost + && sudo docker run -dit --network=host --privileged -v /dev:/dev -e DISPLAY=${DISPLAY} [image_name:tag]

# Insatll Groot2

```
chmod +x . Groot2-1.3.1-linux-installer.run
./Groot2-1.3.1-linux-installer.run
# There is a GUI installer, just follow it.
# Try to run groot2 in terminal, if it works, then it is installed successfully.
groot2
```

docker run --network NUC_bridge -e ROS_IP=192.168.137.1 -e ROS_MASTER_URI=http://192.168.137.1:11311 -e ROS_HOSTNAME=192.168.137.1 -e ROS_TRANSPORT=ros2 --gpus all -dit --ipc=host --privileged -e DISPLAY=host.docker.internal:0.0 -e NVIDIA_DRIVER_CAPABILITIES=all rm_sentry:deplot

docker run --gpus all -dit --net=host --pid=host --privileged -e DISPLAY=host.docker.internal:0.0 -e NVIDIA_DRIVER_CAPABILITIES=all rm_sentry:deplot
