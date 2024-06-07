# Docker Image Quick Start

# RUN 

sudo xhost + && sudo docker run -dit --network=host --pid=host --privileged -v /dev:/dev -e DISPLAY=${DISPLAY} [image_name:tag]

# Insatll Groot2

You must install groot2 manually.

```
chmod +x . Groot2-1.3.1-linux-installer.run
./Groot2-1.3.1-linux-installer.run
# There is a GUI installer, just follow it.
# Try to run groot2 in terminal, if it works, then it is installed successfully.
groot2
```
