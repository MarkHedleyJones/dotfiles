#!/usr/bin/env bash

# Edit aliases
alias ea="vim ~/repos/dotfiles/bashrc_custom.sh"
# Edit bashrc
alias eb="vim ~/.bashrc"
# Edit i3 config
alias ei="vim ~/repos/dotfiles/i3/config"

alias mycal="ncal -w3Mb"
alias mycal12="mycal -B6 -A6"
alias mycal6="mycal -B3 -A3"
alias mycal3="mycal"

alias arst="subl --new-window . && smerge --new-window ."
alias cmd="cd ~/catkin_ws && catkin_make -j $(( $(nproc) - 1 )) && rospack profile && source devel/setup.bash && cd -"
alias cmr="cd ~/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release -j $(($(nproc) - 1)) && rospack profile && source devel/setup.bash && cd -"
alias cs="source ~/catkin_ws/devel/setup.bash"
alias sm="smerge --new-window ."
alias st="subl --new-window ."
alias n="nautilus . --new-window &"

alias c="clear"
alias s="cd ~/repos/sq-slammer"
alias df="cd ~/repos/dotfiles"
alias slam="cd ~/repos/sq-slammer"
alias ah="cd ~/repos/akiyahopper"
alias as="cd ~/repos/akiya-scraper"
alias aa="cd ~/repos/akiyahopper-ai"
alias dev="code . && sleep 1 && sm"
alias pv="pcl_viewer -fc 0,255,0 -ps 1 -opaque 0.5"
alias r="cd ~/repos"
alias rl='c && cs && catkin_make roslint 2>&1 1>/dev/null | grep -v "make" | awk -F: '\''{ "basename " $1 | getline result} { print result ":" $2 $3 }'\'''
alias rtplog='c && cs && catkin_make test_plog_detection && rostest --text sq_plog_lidar_detection plog_detection.test && cd -'
alias rtphoto='c && cs && catkin_make test_take_target_photo && rostest --text  sq_take_target_photo take_target_photo.test && cd -'
alias rtlidar='c && cs && catkin_make test_plog_detection && rostest --text  sq_plog_lidar_detection lidar_postprocessor.test && cd -'
alias rtocc='c && cs && catkin_make test_occlusion_detection_server && rostest --text sq_take_target_photo occlusion_detection_server.test && cd -'
alias rtoccs='c && cs && catkin_make test_occlusion_detection_server && rostest --reuse-master --text sq_take_target_photo occlusion_detection_server.test && cd -'
alias rt='rtplog rtphoto'
alias rlt='rl && rt'

alias workon='source .venv/bin/activate'

alias lt='l && t'
# Moved to i3 config as keybord shortcuts
# xset r rate 175 25
# setxkbmap -option "shift:both_capslock"
# setxkbmap -option "shift:both_capslock" -layout us -variant ,qwerty
# xmodmap -e "clear Lock"

LOCAL_BIN="${HOME}/.local/bin"

# If a local binary folder exists but isn't in the $PATH, add it to the $PATH
if [ -d ${LOCAL_BIN} ] && [ "$(export | grep PATH | grep ${LOCAL_BIN})" == "" ]; then
    export PATH="${PATH}:${LOCAL_BIN}"
fi

# From https://mhoffman.github.io/2015/05/21/how-to-navigate-directories-with-the-shell.html
alias ..="cd .."
alias ...="cd ../.."
alias ....="cd ../../.."
alias .....="cd ../../../.."
alias ......="cd ../../../../.."
alias .......="cd ../../../../../.."
alias ........="cd ../../../../../../.."
alias .........="cd ../../../../../../../.."
alias ..........="cd ../../../../../../../../.."
alias ...........="cd ../../../../../../../../../.."
alias ............="cd ../../../../../../../../../../.."
alias .............="cd ../../../../../../../../../../../.."
