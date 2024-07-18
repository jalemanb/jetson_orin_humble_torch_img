#!/usr/bin/env bash

# On Crontab -e
# @reboot sleep 3 && tmux new-session -d -s temi_ros "~/Commands/run_container.sh"

docker run -it --rm --net=host --runtime nvidia --privileged --device=/dev/d435 --name temi jalemanb/temijetson:v1.0

