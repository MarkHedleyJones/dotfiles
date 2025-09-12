#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
LOCAL_BIN="${HOME}/.local/bin"
PERSONAL_ALIASES="${HOME}/.aliases"
SHELL_CUSTOMIZATION="${SCRIPT_DIR}/customise_shell.sh"

# Function to add source line to shell config if not already present
add_source_to_shell_config() {
	local config_file="$1"
	local source_line="source ${SHELL_CUSTOMIZATION}"

	if [ -f "${config_file}" ]; then
		if ! grep -q "source.*customise_shell.sh" "${config_file}"; then
			echo "Adding shell customization to ${config_file}"
			echo "" >>"${config_file}"
			echo "# Dotfiles shell customization" >>"${config_file}"
			echo "${source_line}" >>"${config_file}"
			echo "Added source line to ${config_file}"
		else
			echo "Shell customization already configured in ${config_file}"
		fi
	else
		echo "Creating ${config_file} with shell customization"
		echo "# Dotfiles shell customization" >"${config_file}"
		echo "${source_line}" >>"${config_file}"
	fi
}

if [ ! -L ${HOME}/.config/i3 ]; then
	echo "Linking ~/.config/i3 to ${SCRIPT_DIR}/i3"
	if [ -d ${HOME}/.config/i3 ]; then
		mv ${HOME}/.config/i3 ${HOME}/.config/i3_original
	fi
	ln -s ${SCRIPT_DIR}/i3 ${HOME}/.config/i3
fi

if [ ! -d ${LOCAL_BIN} ]; then
	echo "Creating a local binary folder at: ${LOCAL_BIN}"
	mkdir -p ${LOCAL_BIN}
fi

for filename in $(ls ${SCRIPT_DIR}/bin); do
	if [ ! -L ${LOCAL_BIN}/${filename} ]; then
		echo "Linking ${filename} to ${LOCAL_BIN}/${filename}"
		ln -s ${SCRIPT_DIR}/bin/${filename} ${LOCAL_BIN}/${filename}
	fi
done

if [ ! -L ${HOME}/.screenlayout ]; then
	echo "Linking ~/.screenlayout to ${SCRIPT_DIR}/screenlayout"
	if [ -d ${SCRIPT_DIR}/.screenlayout ]; then
		ln -s ${SCRIPT_DIR}/.screenlayout ${HOME}
	else
		echo "Warning: ${SCRIPT_DIR}/.screenlayout does not exist, skipping"
	fi
fi

# Set up keyboard configuration
KEYBOARD_CONFIG_DIR="${HOME}/.config/keyboard"
if [ ! -d ${KEYBOARD_CONFIG_DIR} ]; then
	echo "Creating keyboard configuration directory at: ${KEYBOARD_CONFIG_DIR}"
	mkdir -p ${KEYBOARD_CONFIG_DIR}
fi

# Link keyboard color configuration files
if [ -d ${SCRIPT_DIR}/keyboard ]; then
	for config_file in $(ls ${SCRIPT_DIR}/keyboard/*.json 2>/dev/null); do
		filename=$(basename "${config_file}")
		target="${KEYBOARD_CONFIG_DIR}/${filename}"
		if [ ! -L "${target}" ]; then
			echo "Linking keyboard config: ${filename}"
			ln -sf "${config_file}" "${target}"
		fi
	done
else
	echo "Warning: ${SCRIPT_DIR}/keyboard does not exist, skipping keyboard configuration"
fi

# Recursively link config files from config/ to ~/.config/dotfiles/
if [ -d "${SCRIPT_DIR}/config" ]; then
	echo ""
	echo "Setting up configuration files..."
	CONFIG_TARGET_DIR="${HOME}/.config/dotfiles"
	mkdir -p "${CONFIG_TARGET_DIR}"

	# Use find to recursively process all files in config/
	while IFS= read -r -d '' config_file; do
		# Get relative path from config directory
		relative_path="${config_file#${SCRIPT_DIR}/config/}"
		target_file="${CONFIG_TARGET_DIR}/${relative_path}"
		target_dir="$(dirname "${target_file}")"

		# Create target directory if it doesn't exist
		mkdir -p "${target_dir}"

		# Link the file if not already linked
		if [ ! -L "${target_file}" ]; then
			echo "Linking config: ${relative_path}"
			ln -sf "${config_file}" "${target_file}"
		fi
	done < <(find "${SCRIPT_DIR}/config" -type f -print0)
else
	echo "Warning: ${SCRIPT_DIR}/config does not exist, skipping config linking"
fi

# Set up personal aliases file if it doesn't exist
if [ ! -f "${PERSONAL_ALIASES}" ]; then
	echo "Creating personal aliases file from template..."
	cp "${SCRIPT_DIR}/.aliases.example" "${PERSONAL_ALIASES}"
	echo "Created ${PERSONAL_ALIASES} - customize this file with your personal aliases"
	echo "This file will not be tracked by git"
fi

# Add shell customization to bashrc and zshrc
echo ""
echo "Setting up shell customization..."
add_source_to_shell_config "${HOME}/.bashrc"
add_source_to_shell_config "${HOME}/.zshrc"

# Check for git-hooks repository
echo ""
echo "Checking for git-hooks repository..."
GIT_HOOKS_DIR="${HOME}/repos/git-hooks"

if [ -d "${GIT_HOOKS_DIR}" ]; then
	echo "Found git-hooks repository at ${GIT_HOOKS_DIR}"
	echo "Run '${GIT_HOOKS_DIR}/install.sh' to set up git hooks and formatters"
else
	echo "Git hooks repository not found."
	echo "Clone it with: git clone git@github.com:MarkHedleyJones/git-hooks.git ~/repos/git-hooks"
	echo "Then run: ~/repos/git-hooks/install.sh"
fi

echo ""
echo "Installation complete! Please restart your terminal or run:"
echo "  source ${SHELL_CUSTOMIZATION}"
