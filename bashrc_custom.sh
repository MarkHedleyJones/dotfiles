#! /bin/bash
# These are custom aliases
alias arst='subl --new-window . && smerge --new-window .'
alias cm='cd ~/catkin_ws ; catkin_make -j8; rospack profile ; source devel/setup.bash ; cd -'
alias sm='smerge --new-window .'
alias st='subl --new-window .'
alias oien='cd ~/repos/sq-slammer'

xset r rate 175 25
setxkbmap -option "shift:both_capslock"
xmodmap -e "clear Lock"
