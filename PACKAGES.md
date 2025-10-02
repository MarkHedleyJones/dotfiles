# Required Packages

This document lists all packages required for this dotfiles configuration.

## Core Components

### Window Manager
- `i3` - i3 window manager
- `gnome-flashback` - GNOME integration for i3
- `picom` - Compositor for transparency and window effects

### Desktop Environment
- `polybar` - Status bar
- `dunst` - Notification daemon
- `wezterm` - Terminal emulator

## Utilities

### System Tools
- `ddcutil` - External monitor brightness control (DDC/CI)
- `xdotool` - X11 automation tool (window management)
- `jq` - JSON processor (used by notify script)
- `i3-msg` - i3 IPC tool (usually included with i3)
- `notify-send` - Desktop notifications (usually included with libnotify-bin)

### Development Tools
- `gh` - GitHub CLI (optional, for aliases sync scripts)

## Installation

Install all required packages on Ubuntu:

```bash
sudo apt install i3 gnome-flashback picom polybar dunst ddcutil xdotool jq libnotify-bin
```

### Manual Installation Required
- `wezterm` - See https://wezfurlong.org/wezterm/installation.html
- `gh` - See https://cli.github.com/ (optional)

## Verification

Check installed packages:
```bash
dpkg -l | grep -E 'i3|gnome-flashback|picom|polybar|dunst|ddcutil|xdotool|jq|libnotify-bin'
which wezterm gh
```
