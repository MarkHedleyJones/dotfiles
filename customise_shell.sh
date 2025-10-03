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

# Enable autocd - typing a directory name will cd to it
if [ -n "$BASH_VERSION" ]; then
	shopt -s autocd 2>/dev/null
fi
# Zsh has autocd enabled by default in most configs

# Set up fzf keybindings if fzf is installed
if command -v fzf >/dev/null 2>&1; then
	# Detect shell type and load appropriate fzf configuration
	if [ -n "$BASH_VERSION" ]; then
		# Bash configuration
		if [ -f /usr/share/doc/fzf/examples/key-bindings.bash ]; then
			source /usr/share/doc/fzf/examples/key-bindings.bash
		elif [ -f ~/.fzf.bash ]; then
			source ~/.fzf.bash
		fi

		# Custom functions for file/directory selection
		__fzf_select_file__() {
			local cmd="${FZF_CTRL_F_COMMAND:-"command find -L . -mindepth 1 \\( -path '*/\\.*' -o -fstype 'sysfs' -o -fstype 'devfs' -o -fstype 'devtmpfs' -o -fstype 'proc' \\) -prune -o -type f -print 2> /dev/null | cut -b3-"}"
			eval "$cmd" | fzf +m
		}

		__fzf_select_dir__() {
			local cmd="${FZF_CTRL_G_COMMAND:-"command find -L . -mindepth 1 \\( -path '*/\\.*' -o -fstype 'sysfs' -o -fstype 'devfs' -o -fstype 'devtmpfs' -o -fstype 'proc' \\) -prune -o -type d -print 2> /dev/null | cut -b3-"}"
			eval "$cmd" | fzf +m
		}

		# Ctrl+F for files - insert at cursor with space
		__fzf_file_insert__() {
			local file
			file=$(__fzf_select_file__)
			if [[ -n $file ]]; then
				# Add a space before the file if we're not at the beginning and there's no space already
				local prefix=""
				if [[ $READLINE_POINT -gt 0 && "${READLINE_LINE:$((READLINE_POINT-1)):1}" != " " ]]; then
					prefix=" "
				fi
				READLINE_LINE="${READLINE_LINE:0:$READLINE_POINT}${prefix}${file}${READLINE_LINE:$READLINE_POINT}"
				((READLINE_POINT += ${#prefix} + ${#file}))
			fi
		}
		bind -m emacs-standard -x '"\C-f": __fzf_file_insert__'
		bind -m vi-command -x '"\C-f": __fzf_file_insert__'
		bind -m vi-insert -x '"\C-f": __fzf_file_insert__'

		# Ctrl+G for directories (Go to - actually cd there)
		__fzf_cd__() {
			local dir
			dir=$(__fzf_select_dir__) && cd "$dir"
		}
		bind -m emacs-standard -x '"\C-g": __fzf_cd__'
		bind -m vi-command -x '"\C-g": __fzf_cd__'
		bind -m vi-insert -x '"\C-g": __fzf_cd__'

	elif [ -n "$ZSH_VERSION" ]; then
		# Zsh configuration
		if [ -f /usr/share/doc/fzf/examples/key-bindings.zsh ]; then
			source /usr/share/doc/fzf/examples/key-bindings.zsh
		elif [ -f ~/.fzf.zsh ]; then
			source ~/.fzf.zsh
		fi

		# Custom widgets for file/directory selection
		fzf-file-widget() {
			local cmd="${FZF_CTRL_F_COMMAND:-"command find -L . -mindepth 1 \\( -path '*/\\.*' -o -fstype 'sysfs' -o -fstype 'devfs' -o -fstype 'devtmpfs' -o -fstype 'proc' \\) -prune -o -type f -print 2> /dev/null | cut -b3-"}"
			setopt localoptions pipefail no_aliases 2> /dev/null
			local file
			file=$(eval "$cmd" | FZF_DEFAULT_OPTS="--height 40% --reverse --bind=ctrl-z:ignore $FZF_DEFAULT_OPTS" fzf +m)
			local ret=$?
			if [[ -n $file ]]; then
				# Add space if needed
				if [[ -n $LBUFFER && "${LBUFFER[-1]}" != " " ]]; then
					LBUFFER+=" "
				fi
				LBUFFER+="${(q)file}"
			fi
			zle reset-prompt
			return $ret
		}

		fzf-dir-widget() {
			local cmd="${FZF_CTRL_G_COMMAND:-"command find -L . -mindepth 1 \\( -path '*/\\.*' -o -fstype 'sysfs' -o -fstype 'devfs' -o -fstype 'devtmpfs' -o -fstype 'proc' \\) -prune -o -type d -print 2> /dev/null | cut -b3-"}"
			setopt localoptions pipefail no_aliases 2> /dev/null
			local dir
			dir=$(eval "$cmd" | FZF_DEFAULT_OPTS="--height 40% --reverse --bind=ctrl-z:ignore $FZF_DEFAULT_OPTS" fzf +m) && cd "$dir"
			local ret=$?
			zle reset-prompt
			return $ret
		}

		zle -N fzf-file-widget
		zle -N fzf-dir-widget
		bindkey '\C-f' fzf-file-widget
		bindkey '\C-g' fzf-dir-widget
	fi

	# Custom fzf options for better experience
	export FZF_DEFAULT_OPTS="--height 40% --layout=reverse --border --preview 'ls -la {}' --preview-window=right:50%:wrap"

	# Use fd if available for faster file/directory searching
	if command -v fd >/dev/null 2>&1; then
		export FZF_CTRL_F_COMMAND='fd --type f --hidden --follow --exclude .git'
		export FZF_CTRL_G_COMMAND='fd --type d --hidden --follow --exclude .git'
	fi
fi
