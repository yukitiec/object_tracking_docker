# Procedure

## How to setup WSL/Ubuntu22.04 environment.
- [WSL install](https://qiita.com/zakoken/items/61141df6aeae9e3f8e36)
- [Ubuntu22.04 image](https://cloud-images.ubuntu.com/wsl/jammy/current/)
- make directory
```sh
mkdir C:\WSL\Ubuntu2204
```
- Import the tarball into WSL2
```sh
wsl --import Ubuntu-22.04 C:\WSL\Ubuntu2204 C:\Users\kawaw\Downloads\ubuntu-jammy-wsl-amd64-ubuntu22.04lts.rootfs.tar.gz --version 2
```
- Confirm Ubuntu22.04 was imported
```sh
wsl -l -v
```
- Launch the new distro
```sh
wsl -d Ubuntu-22.04
```
- Now, we are inside the Ubuntu-22.04.05. Check the Ubuntu version.
```sh
cat /etc/os-release

#Return:
#root@wakki:/mnt/c# cat /etc/os-release
#PRETTY_NAME="Ubuntu 22.04.5 LTS"
#NAME="Ubuntu"
#VERSION_ID="22.04"
#VERSION="22.04.5 LTS (Jammy Jellyfish)"
#VERSION_CODENAME=jammy
#ID=ubuntu
#ID_LIKE=debian
#HOME_URL="https://www.ubuntu.com/"
#SUPPORT_URL="https://help.ubuntu.com/"
#BUG_REPORT_URL="https://bugs.launchpad.net/ubuntu/"
#PRIVACY_POLICY_URL="https://www.ubuntu.com/legal/terms-and-policies/privacy-policy"
#UBUNTU_CODENAME=jammy
```
- (Optional) Create the new user
```sh
#adduser [name]
adduser yuki
```
- Give the user sudo privileges
```sh
usermod -aG sudo yuki
```
- Set the default WSL user.
```sh
cat <<EOF > /etc/wsl.conf
[user]
default=yuki
EOF
```
- exit the distro
```sh
exit
```
- shutdown WSL and restart
```sh
wsl --shutdown
```
- start Ubuntu22.04 again
```sh
wsl -d Ubuntu-22.04
```
- verify the current user
```sh
whoami
#- response: yuki
```
- Update the packages
```sh
sudo apt update
sudo apt upgrade -y
```
- set Ubuntu-22.04 default
```sh
wsl -s Ubuntu-22.04
```

## How to create Docker image in windows.
- Docker Desktop + WSL2 integration
0. turn on the Docker Desktop
1. Check whether Docker is available in Ubuntu 22.04 WSL => If okay, skip to step 11.
```sh
docker --version
docker compose version
```
2. Start Docker Desktop on Windows.
3. Open Docker Desktop settings.
- gear icon >> Settings.
4. Confirm WSL2 backend is enabled.
- General >> Use the WSL2 based engine.
5. Enable integration for Ubuntu 22.04 distro
- Resources (Gear icon >> Resources in Docker Desktop) >> WSL Integration.
- Turn on Enable integration with additional distros: Ubuntu-22.04
6. Exit the WSL Ubuntu-22.04
```sh
exit
```
7. restart Ubuntu-22.04 and turn on the Docker Desktop
```sh
wsl -d Ubuntu-22.04
```
8. Check whether docker engine is available. >> Step 1.
9. (Test) Run a simple test container.
```sh
docker run --rm hello-world
```
10. (Test) Check Docker context
```sh
docker context ls
```
11. Go to the docker project
```sh
cd /mnt/c/Users/kawaw/cpp/object_tracking_linux/docker
```
12. (Optional) For faster file I/O: Move the repository into the Linux filesystem.

```sh
# Copy the entire project directory from the Windows filesystem to your Linux home directory
cp -r /mnt/c/Users/kawaw/cpp/object_tracking_linux ~/object_tracking_linux

# (Optional) If you want to start fresh, you can delete the copied directory first to avoid old contents:
# rm -rf ~/object_tracking_linux

# Change directory to the docker folder inside the copied repo
cd ~/object_tracking_linux/docker
```

13. build Dockerfile to create Docker image.
```sh
#if linux user.
docker compose build

# For preventing windows-specific format error.
#convert line endings to Unix format
sed -i 's/\r$//' ros_entrypoint.sh
#check
# yuki@wakki:~/object_tracking_linux/docker$ file ros_entrypoint.sh
#ros_entrypoint.sh: Bourne-Again shell script, ASCII text executable

#executable
chmod +x ros_entrypoint.sh
#check the contents
cat -A ros_entrypoint.sh
#rebuild
docker compose build --no-cache
```

## Repository architecture
repository/ \
├── docker/ \
│   ├── Dockerfile \
│   ├── docker-compose.yml \
│   └── ros_entrypoint.sh \
├── ros_ws/ \
│   └── src/ \
│       └── tracker_pkg/ \
│           ├── CMakeLists.txt \
│           ├── package.xml \
│           ├── include/ \
│           │   └── tracker_pkg/ \
│           │       ├── yolo_node.h \
│           │       ├── tracker_node.h \
│           │       ├── global_parameters.h \
│           │       ├── hungarian.h \
│           │       ├── kalmanfilter.h \
│           │       ├── manage_tracker.h \
│           │       ├── utils.h \
│           │       └── yolo_detect_batch.h \
│           ├── src/ \
│           │   ├── main.cpp \
│           │   ├── yolo_node.cpp \
│           │   ├── tracker_node.cpp \
│           │   ├── global_parameters.cpp \
│           │   ├── hungarian.cpp \
│           │   ├── kalmanfilter.cpp \
│           │   ├── manage_tracker.cpp \
│           │   ├── utils.cpp \
│           │   └── yolo_detect_batch.cpp \
│           ├── config/ \
│           │   ├── default.txt \
│           │   └── yolov10m_w640_h480.torchscript \
│           └── tests/ \
│               └── tests.cpp \
├── make_torchscript/ \
│   └── makeTorchScript.ipynb \
├── analysis/ \
│   └── analysis.ipynb \
├── .pre-commit-config.yaml \
├── Makefile \
└── README.md

## Run the code in Docker container

0. Enter WSL env.
```sh
wsl -d Ubuntu-22.04
```

1. Move to the docker directory
```sh
cd ~/object_tracking_linux/docker
```

2. Start the container
```sh
#if Linux
docker compose run --rm ros2_tracker bash

# (Trouble shooting)
# If /ros_entrypoint.sh does not exist.
# problem may be "ros_entrypoint.sh has Windows line endings (CRLF)"
cd ~/object_tracking_linux/docker
#convert line endings to Unix format
sed -i 's/\r$//' ros_entrypoint.sh
#check
# yuki@wakki:~/object_tracking_linux/docker$ file ros_entrypoint.sh
#ros_entrypoint.sh: Bourne-Again shell script, ASCII text executable

#executable
chmod +x ros_entrypoint.sh
#check the contents
cat -A ros_entrypoint.sh
#rebuild
docker compose build --no-cache
#run again.
docker compose run --rm ros2_tracker bash
# return ; yuki@wakki:~/object_tracking_linux/docker$ docker compose run --rm ros2_tracker bash
# root@docker-desktop:/ros_ws#
```
3. Source ROS2
```sh
source /opt/ros/humble/setup.bash
#check which version to use
echo $ROS_DISTRO
# return : humble (ROS2 humble)
```
4. (Optional) Check the source files are mounted.
```sh
ls /ros_ws
ls /ros_ws/src
ls /ros_ws/src/tracker_pkg
ls /ros_ws/src/tracker_pkg/config
ls /ros_ws/src/tracker_pkg/video
```
5. Build the workspace inside the container
```sh
cd /ros_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DTorch_DIR=/opt/libtorch/share/cmake/Torch

#build -> source 
source /ros_ws/install/setup.bash
```

6. (Optional) Verify the package is recognized.
```sh
ros2 pkg list | grep tracker_pkg
#tracker_pkg
```

7. (Optional) Find the available executables
```sh
ros2 pkg executables tracker_pkg
# return : tracker_pkg tracker_pipeline
```

8. Run tests/test.cpp
```sh
#locate tracker_tests.
find /ros_ws/build -name tracker_tests
find /ros_ws/install -name tracker_tests
#run
#if in /ros_ws/build
/ros_ws/build/tracker_pkg/tracker_tests
#if in /ros_ws/install
/ros_ws/install/tracker_pkg/lib/tracker_pkg/tracker_tests

#expected return
#root@docker-desktop:/ros_ws# /ros_ws/build/tracker_pkg/tracker_tests
#[PASS] test_synthetic_image_creation
#Warning: Unknown config key at line 1: display=1
#Warning: Unknown config key at line 2: time_capture=5.0
#Warning: Unknown config key at line 3: yolo_path=/tmp/model.torchscript
#Warning: Unknown config key at line 4: yoloWidth=640
#Warning: Unknown config key at line 5: yoloHeight=480
#Warning: Unknown config key at line 6: object_index=0,1
#Warning: Unknown config key at line 7: IoU_threshold=0.5
#Warning: Unknown config key at line 8: conf_threshold=0.4
#[PASS] test_load_config
#[PASS] test_tracker_update2d
#All tests passed.
```

9. Run main.cpp
```sh
#check config file.
cat /ros_ws/src/tracker_pkg/config/default.txt
ls -l /ros_ws/src/tracker_pkg/config/yolov10m_w640_h480.torchscript

#if config file is awkward.
cat > /ros_ws/src/tracker_pkg/config/default.txt <<'EOF'
display true
time_capture 70
video_path /ros_ws/src/tracker_pkg/video/test1_original/video.mp4

yolo_path /ros_ws/src/tracker_pkg/config/yolov10m_w640_h480.torchscript
yoloWidth 640
yoloHeight 480
IoU_threshold 0.7
conf_threshold 0.3

object_index 73,76
EOF
#run main.cpp
ros2 run tracker_pkg tracker_pipeline
```

## When change the code in docker
- Install extensions of "Dev Containers"
- attach to the running container
- open /ros_ws inside the container
- change the code
- build/source
```sh
cd /ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DTorch_DIR=/opt/libtorch/share/cmake/Torch
source /ros_ws/install/setup.bash
```

## Rebuild the docker
```sh
cd ~/object_tracking_linux/docker
docker compose build
#clean rebuild
docker compose build --no-cache
```

## (Trouble shooting) W
- rm: cannot remove '/home/yuki/object_tracking_linux/ros_ws/build/tracker_pkg/cmake_install.cmake': Permission denied
rm: cannot remove '/home/yuki/object_tracking_linux/ros_ws/build/.built_by': Permission denied

1. stop all containers
```sh
cd /mnt/c/Users/kawaw/cpp/object_tracking_linux/docker
docker compose down

#check the current docker image status
docker ps -a
```
2. fix ownership of the broken WSL copy
```sh
sudo chown -R yuki:yuki ~/object_tracking_linux
```
3. remove the broken WSL copy
```sh
rm -rf ~/object_tracking_linux
```
4. Re-copy the host directory.
```sh
cp -r /mnt/c/Users/kawaw/cpp/object_tracking_linux ~/object_tracking_linux

# (Optional) If you want to start fresh, you can delete the copied directory first to avoid old contents:
# rm -rf ~/object_tracking_linux

# Change directory to the docker folder inside the copied repo
cd ~/object_tracking_linux/docker
```
5. Rebuild the docker image
```sh
#if Linux
docker compose run --rm ros2_tracker bash

# (Trouble shooting)
# If /ros_entrypoint.sh does not exist.
# problem may be "ros_entrypoint.sh has Windows line endings (CRLF)"
cd ~/object_tracking_linux/docker
#convert line endings to Unix format
sed -i 's/\r$//' ros_entrypoint.sh
#check
# yuki@wakki:~/object_tracking_linux/docker$ file ros_entrypoint.sh
#ros_entrypoint.sh: Bourne-Again shell script, ASCII text executable

#executable
chmod +x ros_entrypoint.sh
#check the contents
cat -A ros_entrypoint.sh
#rebuild
docker compose build --no-cache
#run again.
docker compose run --rm ros2_tracker bash
# return ; yuki@wakki:~/object_tracking_linux/docker$ docker compose run --rm ros2_tracker bash
# root@docker-desktop:/ros_ws#
```