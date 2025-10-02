#!/usr/bin/env bash

# Set default terminal
export TERMINAL=wezterm

# Calendar aliases
alias mycal="ncal -w3Mb"
alias mycal12="mycal -B6 -A6"
alias mycal6="mycal -B3 -A3"
alias mycal3="mycal"

alias f="nautilus . --new-window &"
alias c="clear"
alias r="reset"
alias tmp="cd /tmp"

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
if [ -d "${LOCAL_BIN}" ] && [ "$(export | grep PATH | grep ${LOCAL_BIN})" = "" ]; then
	export PATH="${PATH}:${LOCAL_BIN}"
fi

# Track terminal workspace and window ID for notify command
update_terminal_workspace() {
	if [ -n "$DISPLAY" ] && command -v i3-msg >/dev/null 2>&1; then
		local focused_window=$(xdotool getwindowfocus 2>/dev/null)
		if [ -n "$focused_window" ] && [ "$focused_window" != "1" ]; then
			# Store the window ID for this terminal
			export TERMINAL_WINDOW_ID="$focused_window"

			# Also get the workspace
			export TERMINAL_WORKSPACE=$(i3-msg -t get_tree 2>/dev/null | python3 -c "
import sys, json
try:
    tree = json.load(sys.stdin)
    target = $focused_window
    def search(node, ws=None):
        if node.get('type') == 'workspace':
            ws = node.get('name')
        if node.get('window') == target:
            return ws
        for child in node.get('nodes', []) + node.get('floating_nodes', []):
            result = search(child, ws)
            if result: return result
        return None
    print(search(tree) or '')
except: pass
" 2>/dev/null)
		fi
	fi
}

# Update workspace info when terminal starts
update_terminal_workspace

# Load personal/private aliases if they exist
PERSONAL_ALIASES="${HOME}/.aliases"
if [ -f "${PERSONAL_ALIASES}" ]; then
	source "${PERSONAL_ALIASES}"
fi
