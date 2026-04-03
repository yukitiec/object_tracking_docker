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
docker compose build
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
```

3.