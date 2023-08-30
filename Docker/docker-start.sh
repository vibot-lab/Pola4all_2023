#!/bin/bash
# We add a non-network connection between the X server and a user
dummyVar=$(xhost local:root)

ROOTDIR=$(dirname $(readlink -f "$0"))
cd ${ROOTDIR}

function show_help {
    printf "This script will start the Docker container to work with a DoFP polarization camera.\n" > /dev/tty
    printf "The entire system is supposed to be composed by ROS server that interacts directly with the\n" > /dev/tty
    printf "used camera, and a ROS client that will receive the raw images from it. Then, this client is\n" > /dev/tty
    printf "attached to a GUI software to show the images, and do computer vision algorithms on it.\n" > /dev/tty
    printf "\n" > /dev/tty
    printf "Usage:\n" > /dev/tty
    printf "                docker-start.sh \n" > /dev/tty
    printf "\n" > /dev/tty
    printf "\n" > /dev/tty
    printf " --intel                      Initialize the docker container with the Intel Graphics capabilities.\n" > /dev/tty
    printf "\n" > /dev/tty
    printf " --nvidia                     Initialize the docker container with the NVIDIA Graphics capabilities.\n" > /dev/tty
    printf "\n" > /dev/tty


}

DOCKER_INIT_FLAGS="-ti --rm --privileged --device=/dev/dri --group-add video --group-add dialout -v /tmp/.X11-unix:/tmp/.X11-unix -v /dev:/dev"
USE_NVIDIA=0
USE_INTEL=0
while [ $# -gt 0 ]; do
    case "$1" in
        --nvidia)
            USE_NVIDIA=1
            shift
            ;;
        --intel)
            USE_INTEL=1
            shift
            ;;
        -h|--help )
            show_help
            exit 0
            ;;
        *)
            echo "Invalid option $1"
            show_help
            exit 0
            ;;
    esac
done

if [[ "${USE_NVIDIA}" == 1 ]] && [[ "${USE_INTEL}" == 1 ]]; then
    echo "ERROR: You need to specify only one of the flags: either --intel either --nvidia flags"
    exit -1
fi

if [[ "${USE_NVIDIA}" == 0 ]] && [[ "${USE_INTEL}" == 0 ]]; then
    echo "ERROR: You need to specify either --intel either --nvidia flags"
    exit -1
fi

if [[ "${USE_NVIDIA}" == 1 ]]; then
    DOCKER_INIT_FLAGS+=" --gpus all -e LIBVA_DRIVER_NAME=i965"
fi

if [[ "${USE_INTEL}" == 1 ]]; then
    DOCKER_INIT_FLAGS+=" -e LIBVA_DRIVER_NAME=iHD"
fi

sudo docker run  ${DOCKER_INIT_FLAGS} \
    --env="DISPLAY=$DISPLAY" \
    --env="SHELL=$SHELL" \
    --env="USER=$USER" \
    --network=host \
    --ipc=host \
    --user $USER \
    --entrypoint /home/$USER/docker-entrypoint.sh \
    --workdir /home/$USER/Pola4All \
    docker_pola4all \
    /bin/bash
