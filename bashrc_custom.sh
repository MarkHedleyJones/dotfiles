#!/usr/bin/env bash

alias aliases="vim ~/repos/dotfiles/bashrc_custom.sh"
alias eb="vim ~/.bashrc"

# alias cal="cal --iso --week -3"
alias cal="ncal -w3Mb"

alias dir-dot="cd ~/repos/dotfiles"
alias edit-dot="subl ~/repos/dotfiles --new-window"
alias merge-dot="sublime_merge ~/repos/dotfiles --new-window"

alias arst="subl --new-window . && smerge --new-window ."
alias cm="cd ~/catkin_ws ; catkin_make -j$(nproc); rospack profile ; source devel/setup.bash ; cd -"
alias cs="source ~/catkin_ws/devel/setup.bash"
alias sm="smerge --new-window ."
alias st="subl --new-window ."

alias c="cd ~/catkin_ws"
alias s="cd ~/repos/seqsense"
alias d='cd ~/repos/dotfiles'
alias r='cd ~/repos'

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
