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

# Function to check and optionally install formatters
check_and_install_formatters() {
	local missing_tools=()
	local install_commands=()

	echo ""
	echo "Checking for code formatters..."

	# Check Python formatter (ruff)
	if ! command -v ruff &>/dev/null; then
		missing_tools+=("ruff")
		install_commands+=("pip install --user --break-system-packages ruff")
	else
		echo "  ✓ ruff (Python formatter) installed"
	fi

	# Check JavaScript/TypeScript formatter (prettier)
	if ! command -v prettier &>/dev/null; then
		missing_tools+=("prettier")
		install_commands+=("npm install -g prettier")
	else
		echo "  ✓ prettier (JS/TS/JSON formatter) installed"
	fi

	# Check C/C++ formatter (clang-format)
	if ! command -v clang-format-18 &>/dev/null && ! command -v clang-format &>/dev/null; then
		missing_tools+=("clang-format")
		install_commands+=("sudo apt install clang-format")
	else
		echo "  ✓ clang-format (C/C++ formatter) installed"
	fi

	# Check shell formatter (shfmt)
	if ! command -v shfmt &>/dev/null; then
		missing_tools+=("shfmt")
		install_commands+=("sudo apt install shfmt")
	else
		echo "  ✓ shfmt (shell formatter) installed"
	fi

	# If tools are missing, offer to install
	if [ ${#missing_tools[@]} -gt 0 ]; then
		echo ""
		echo "The following formatters are not installed:"
		for i in "${!missing_tools[@]}"; do
			echo "  - ${missing_tools[$i]}: ${install_commands[$i]}"
		done
		echo ""
		read -p "Would you like to install these formatters? (y/N): " -n 1 -r
		echo ""
		if [[ $REPLY =~ ^[Yy]$ ]]; then
			for cmd in "${install_commands[@]}"; do
				echo "Running: $cmd"
				eval "$cmd"
			done
		else
			echo "Skipping formatter installation. You can install them manually later."
		fi
	else
		echo "All formatters are installed!"
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

# Set up git hooks
echo ""
echo "Setting up git hooks..."
if [ -d "${SCRIPT_DIR}/git/hooks" ]; then
	# Set global git hooks path
	current_hooks_path=$(git config --global core.hookspath)
	expected_hooks_path="${SCRIPT_DIR}/git/hooks"

	if [ "$current_hooks_path" != "$expected_hooks_path" ]; then
		echo "Setting global git hooks path to: ${expected_hooks_path}"
		git config --global core.hookspath "${expected_hooks_path}"
	else
		echo "Git hooks path already configured"
	fi

	# Make pre-commit hook executable
	if [ -f "${SCRIPT_DIR}/git/hooks/pre-commit" ]; then
		chmod +x "${SCRIPT_DIR}/git/hooks/pre-commit"
		echo "Pre-commit hook is executable"
	fi
else
	echo "Warning: ${SCRIPT_DIR}/git/hooks does not exist, skipping git hooks setup"
fi

# Set up formatter configurations
echo ""
echo "Setting up formatter configurations..."

# Create config directories if they don't exist
mkdir -p "${HOME}/.config/ruff"

# Link ruff configuration
if [ -f "${SCRIPT_DIR}/ruff.toml" ]; then
	if [ ! -L "${HOME}/.config/ruff/ruff.toml" ]; then
		echo "Linking ruff configuration to ~/.config/ruff/ruff.toml"
		ln -sf "${SCRIPT_DIR}/ruff.toml" "${HOME}/.config/ruff/ruff.toml"
	else
		echo "Ruff configuration already linked"
	fi

	# Also link to home directory for broader compatibility
	if [ ! -L "${HOME}/.ruff.toml" ]; then
		ln -sf "${SCRIPT_DIR}/ruff.toml" "${HOME}/.ruff.toml"
	fi
fi

# Link prettier configuration
if [ -f "${SCRIPT_DIR}/.prettierrc" ]; then
	if [ ! -L "${HOME}/.prettierrc" ]; then
		echo "Linking prettier configuration to ~/.prettierrc"
		ln -sf "${SCRIPT_DIR}/.prettierrc" "${HOME}/.prettierrc"
	else
		echo "Prettier configuration already linked"
	fi
fi

# Check for formatters
check_and_install_formatters

echo ""
echo "Installation complete! Please restart your terminal or run:"
echo "  source ${SHELL_CUSTOMIZATION}"
