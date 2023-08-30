#!/bin/bash

ROOTDIR=$(dirname $(readlink -f "$0"))
cd ${ROOTDIR}

ROS_DISTRIBUTION="noetic"
BUILDDIR=$(readlink -f "${ROOTDIR}/../build")
ROS_BUILDDIR=$(readlink -f "${ROOTDIR}/RosCameraServer/build")
ROS_DEVELDIR=$(readlink -f "${ROOTDIR}/RosCameraServer/devel")
BINARY_FILE=${BUILDDIR}/Pola4All
QT_PATH=/home/${USER}/Qt5.12.9/5.12.9

CMAKEFLAGS=""

function show_help {
    printf "This script will build and run the developed software to interact\n" > /dev/tty
    printf "with the Polarimetric camera. This script provides several\n" > /dev/tty
    printf "options to configure the way we build and the way we run the program.\n" > /dev/tty
    printf "There is no behavior by default.\n" > /dev/tty
    printf "\n" > /dev/tty
    printf "Run: build_project.sh [OPTIONS] \n" > /dev/tty
    printf "\n" > /dev/tty
    printf "Options:\n" > /dev/tty
    printf "\n" > /dev/tty
    printf " --help | -h                 Show this help message\n" > /dev/tty
    printf "\n" > /dev/tty
    printf " --clean | -c                Clean the build directories. That \n" > /dev/tty
    printf "                           includes the Qt application and the ROS Server\n" > /dev/tty
    printf "\n" > /dev/tty
    printf " --build | -b                 Build the GUI application.\n" > /dev/tty
    printf "\n" > /dev/tty
    printf " --run-after-build | -r       Run the application. This does not condition\n" > /dev/tty
    printf "                           the script to build the application. If it has been \n" > /dev/tty
    printf "                           build previously, and the build directory has not been erased,\n" > /dev/tty
    printf "                           then the script will just run the program if only this flag\n" > /dev/tty
    printf "                           is provided\n" > /dev/tty
    printf "\n" > /dev/tty
}

BUILD=0
RELEASE=0
RUN=0

while [ $# -gt 0 ]; do
    case "$1" in
        -c|--clean)
            echo "Erasing previous builds"
            rm -rf ${BUILDDIR}
            if [ $? -ne 0 ]; then
                echo "ERROR: Cannot erase the build directory ${BUILDDIR}. Exiting"
                exit -1
            fi
            rm -rf ${ROS_BUILDDIR} ${ROS_DEVELDIR}
            if [ $? -ne 0 ]; then
                echo "ERROR: Cannot erase the ROS build and devel directories at ${ROS_BUILDDIR} and ${ROS_DEVELDIR}. Exiting"
                exit -1
            fi
            shift
            ;;
        -b|--build)
            BUILD=1
            shift
            ;;
        -r|--run-after-build)
            RUN=1
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

mkdir -p ${BUILDDIR}
if [ $? -ne 0 ]; then
    echo "ERROR: Cannot create directory ${BUILDDIR}"
    exit -1
fi

# We add the link libraries to the path
export LD_LIBRARY_PATH=/opt/pylon/lib
export LD_LIBRARY_PATH=${QT_PATH}/Src/qtbase/src/plugins/platforms:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRIBUTION}/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

source /opt/ros/${ROS_DISTRIBUTION}/setup.bash

if [ ${BUILD} -eq 1 ]; then
    (
        cd RosCameraServer
        catkin_make
        if [ $? -ne 0 ]; then
            echo "ERROR: Cannot compile ROS Server"
            exit -1
        fi
        echo "Sourcing ROS server"
        source ${ROS_DEVELDIR}/setup.bash
    )
    if [ $? -ne 0 ]; then
        exit -1
    fi

    {
        cd ${BUILDDIR}
        cmake ${ROOTDIR} ${CMAKEFLAGS}
        if [ $? -ne 0 ]; then
            echo "ERROR: Cannot compile the project."
            exit -1
        fi
        echo "Executing Make"
        make -j8
        if [ $? -ne 0 ]; then
            echo "ERROR: Cannot compile the project. Exiting"
            exit -1
        fi
        echo "Building successful!"
    }
fi

if [ ${RUN} -eq 1 ]; then
    if [ ${RELEASE} -eq 1 ]; then
        LD_LIBRARY_PATH=${BUILDDIR}/lib:${BUILDDIR}; ${BINARY_FILE}
    else
        ${BINARY_FILE}
    fi

    if [ $? -ne 0 ] && [ $? -ne 1 ]; then
        echo "Program has not finished correctly!!"
    fi
fi
echo "Script finished!"
