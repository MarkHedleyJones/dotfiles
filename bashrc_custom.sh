#!/usr/bin/env bash

# Custom aliases
alias arst='subl --new-window . && smerge --new-window .'
alias cm='cd ~/catkin_ws ; catkin_make -j8; rospack profile ; source devel/setup.bash ; cd -'
alias sm='smerge --new-window .'
alias st='subl --new-window .'
alias oien='cd ~/repos/'
alias edit-dot='subl ~/repos/dotFiles --new-window'
alias dir-dot='cd ~/repos/dotFiles'
alias merge-dot='smerge ~/repos/dotFiles --new-window'
alias edit-bash='subl ~/repos/dotFiles/bashrc_custom.sh --new-window'
alias cal='cal --iso --week -3'

xset r rate 175 25
setxkbmap -option "shift:both_capslock"

if [ -f ~/.ssh/agent.env ] ; then
    . ~/.ssh/agent.env > /dev/null
    if ! kill -0 $SSH_AGENT_PID > /dev/null 2>&1; then
        echo "Stale agent file found. Spawning a new agent. "
        eval `ssh-agent | tee ~/.ssh/agent.env`
        ssh-add
    fi
else
    echo "Starting ssh-agent"
    eval `ssh-agent | tee ~/.ssh/agent.env`
    ssh-add
fi

# xmodmap -e "clear Lock"


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

export CDPATH=".:$HOME:$HOME/repos"
