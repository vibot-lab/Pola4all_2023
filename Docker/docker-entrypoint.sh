#!/bin/bash
cp /home/${USER}/vimrc /home/${USER}/.vimrc
cp /home/${USER}/bashrc /home/${USER}/.bashrc
cp /home/${USER}/gitconfig /home/${USER}/.gitconfig

source /home/${USER}/.bashrc
exec "$@"
