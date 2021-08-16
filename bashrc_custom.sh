#!/usr/bin/env bash

alias arst="subl --new-window . && smerge --new-window ."
alias cm="cd ~/catkin_ws ; catkin_make -j$(nproc); rospack profile ; source devel/setup.bash ; cd -"
alias sm="smerge --new-window ."
alias st="subl --new-window ."
alias oien="cd ~/repos/sq-slammer"
alias edit-dot="subl ~/repos/dotFiles --new-window"
alias dir-dot="cd ~/repos/dotFiles"
alias merge-dot="sublime_merge ~/repos/dotFiles --new-window"
alias aliases="subl ~/repos/dotFiles/bashrc_custom.sh --new-window"
alias cal="cal --iso --week -3"

xset r rate 175 25
setxkbmap -option "shift:both_capslock"
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
