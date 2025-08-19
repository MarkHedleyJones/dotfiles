#!/usr/bin/env bash

# Edit aliases
alias ea="vim ~/repos/dotfiles/bashrc_custom.sh"
# Edit bashrc
alias eb="vim ~/.bashrc"
# Edit i3 config
alias ei="vim ~/repos/dotfiles/i3/config"

# Calendar aliases
alias mycal="ncal -w3Mb"
alias mycal12="mycal -B6 -A6"
alias mycal6="mycal -B3 -A3"
alias mycal3="mycal"

# Development tools
alias sm="smerge --new-window ."
alias st="subl --new-window ."
alias n="nautilus . --new-window &"
alias c="clear"
alias dev="code . && sleep 1 && sm"

# Python virtual environment
alias workon="source .venv/bin/activate"

# Navigation shortcuts (using .. pattern)
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

# Set up local bin directory
LOCAL_BIN="${HOME}/.local/bin"

# If a local binary folder exists but isn't in the $PATH, add it to the $PATH
if [ -d ${LOCAL_BIN} ] && [ "$(export | grep PATH | grep ${LOCAL_BIN})" == "" ]; then
    export PATH="${PATH}:${LOCAL_BIN}"
fi

# Load personal/private aliases if they exist
PERSONAL_ALIASES="${HOME}/.aliases"
if [ -f "${PERSONAL_ALIASES}" ]; then
    source "${PERSONAL_ALIASES}"
fi