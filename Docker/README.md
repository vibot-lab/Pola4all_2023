# Installing the software in a Docker image
1. Open a terminal and install Docker compose (in case these instructions are out
   of date, follow the instructions in [this
   link](https://docs.docker.com/engine/install/ubuntu/)).
```bash
sudo apt-get update && sudo apt-get install ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg
echo \
  "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update && sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```
2. Test if the installation finished successfully:
```bash
sudo docker run hello-world
```
If everything went fine, you should see a terminal output similar to this:
```bash
Unable to find image 'hello-world:latest' locally
latest: Pulling from library/hello-world
719385e32844: Pull complete 
Digest: sha256:dcba6daec718f547568c562956fa47e1b03673dd010fe6ee58ca806767031d1c
Status: Downloaded newer image for hello-world:latest

Hello from Docker!
This message shows that your installation appears to be working correctly.

To generate this message, Docker took the following steps:
 1. The Docker client contacted the Docker daemon.
 2. The Docker daemon pulled the "hello-world" image from the Docker Hub.
    (amd64)
 3. The Docker daemon created a new container from that image which runs the
    executable that produces the output you are currently reading.
 4. The Docker daemon streamed that output to the Docker client, which sent it
    to your terminal.

To try something more ambitious, you can run an Ubuntu container with:
 $ docker run -it ubuntu bash

Share images, automate workflows, and more with a free Docker ID:
 https://hub.docker.com/

For more examples and ideas, visit:
 https://docs.docker.com/get-started/
```

2.1 ***OPTIONAL***: If you have an NVIDIA graphics card, install nvidia-docker2 before continuing:
```bash
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update
sudo apt-get install nvidia-docker2
```

3. Clone this repository (for instance, in your HOME directory)
```bash
git clone https://github.com/vibot-lab/Pola4all_2023.git ${HOME}/Pola4All
```

4. Go into the Docker directory of the repository, and build the image.
```bash
cd ${HOME}/Pola4All/Docker
sudo docker build -t docker_pola4all -f ./Dockerfile_<CARD> --build-arg USER=${USER} .
```
There are two options for <CARD>: either ***nvidia***, either ***intel***. The
option depends on the graphics card you have. We provide the USER variable so
that you have the same user name inside and outside the docker image. This way
you can store files inside and have them outside the docker image, without
permission changes.

5. Wait until the image build finishes successfully. This may take time,
   depending on your machine, and your internet connection

6. Copy all the udev rules in your host linux machine. If not, once in docker,
   you will not have access to the cameras. I have included a few in this
   repository, but if your camera is not in the repository already, you need to
   find it and copy it.
```bash
sudo cp ${HOME}/Docker/udev_rules/*.rules /etc/udev/rules.d/
```

7. Start the docker container:
```bash
${HOME}/Docker/docker-start.sh --<CARD>
```
where <CARD> is again, either ***nvidia*** either ***intel***.

8. Once there, you have a single folder in your Docker home directory: Pola4All.
There you will find the software freshly cloned from this repository.

9. Now you can connect to the camera, and use the GUI software.

## NOTE: Sharing content between the host and the Docker image
Since the Docker image is isolated from the host machine, whatever is in the
host machine is not seen by the Docker container, and vice-versa. Therefore,
whichever work is done in the container will not be saved once shut down. In
other words, any work done in the container will be erased as soon as we exit
from it, unless we commit the image changes before exiting.

A commonly used method is to share a folder with the container, and store all
the important data there. Consequently, once exited from the container, the data
stored in that folder will be kept in the host machine. To share a folder with
the container, that folder needs to exists, and we need to assign a path to it
in the container image. Let's say our host folder is placed at ${HOME}/MySharedFolder,
and we want to mount it in /home/SharedDocs in the container. Next, we have to
add a line as this one to the docker run command (bottom command in the docker-start.sh script).
```bash
...
    --network=host \
    --ipc=host \
    --user $USER \
    -v ${HOME}/MySharedFolder:/home/SharedDocs \            ## <<--- Add this line
    --entrypoint /home/$USER/docker-entrypoint.sh \
    --workdir /home/$USER/Pola4All \
    docker_pola4all \
    /bin/bash
```

Then, everything stored in the folder /home/SharedDocs can be checked in the
${HOME}/MySharedFolder at the host OS.