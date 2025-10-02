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
		# Find the actual terminal window by searching for windows belonging to this process or its parents
		local terminal_window=""

		# Method 1: Use WINDOWID if set by the terminal emulator (xterm, urxvt, etc.)
		if [ -n "$WINDOWID" ]; then
			terminal_window="$WINDOWID"
		fi

		# Method 2: Search for windows belonging to parent processes (terminal emulator)
		if [ -z "$terminal_window" ] && command -v xdotool >/dev/null 2>&1; then
			# Try parent process first (usually the terminal emulator)
			terminal_window=$(xdotool search --pid $PPID 2>/dev/null | head -1)

			# If that didn't work, try grandparent
			if [ -z "$terminal_window" ]; then
				local ppid=$(ps -o ppid= -p $PPID 2>/dev/null | tr -d ' ')
				if [ -n "$ppid" ]; then
					terminal_window=$(xdotool search --pid $ppid 2>/dev/null | head -1)
				fi
			fi
		fi

		if [ -n "$terminal_window" ] && [ "$terminal_window" != "1" ]; then
			# Validate window exists in i3 tree and get workspace in one go
			local workspace=$(i3-msg -t get_tree 2>/dev/null | python3 -c "
import sys, json
try:
    tree = json.load(sys.stdin)
    target = $terminal_window
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

			# Only export if we found a valid workspace (validates window exists in i3)
			if [ -n "$workspace" ]; then
				export TERMINAL_WINDOW_ID="$terminal_window"
				export TERMINAL_WORKSPACE="$workspace"
			fi
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
