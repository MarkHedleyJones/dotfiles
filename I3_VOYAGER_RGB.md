# i3 Workspace Voyager RGB Monitor

This tool monitors your i3 workspaces and updates the RGB LEDs on your ZSA Voyager keyboard's F-keys to show which workspaces are active.

## Features

- **F1-F10 keys** correspond to workspaces 1-10
- **LED Colors**:
  - ⚫ **Off** = Empty workspace
  - 🔵 **Blue** = Workspace has windows
  - 🟢 **Green** = Visible workspace (on another monitor)
  - ⚪ **White** = Currently focused workspace
  - 🔴 **Red** = Urgent workspace

## Prerequisites

1. **Keymapp** must be running (the ZSA keyboard configuration software)
2. **Python grpcio module** - Install using ONE of:
   - System package: `sudo apt install python3-grpcio`
   - With pipx: `pipx install grpcio`
   - In a venv: `python3 -m venv venv && venv/bin/pip install grpcio`

## Installation

The scripts are already in `bin/` and will be linked to `~/.local/bin` when you run `./install.sh`

## Usage

### Manual Start
```bash
# Start the monitor (Keymapp must be running first)
i3-voyager-rgb
```

### Automatic Start with i3

Add to your `~/.config/i3/config`:
```
# Start Voyager RGB workspace indicator (after a delay for Keymapp to start)
exec --no-startup-id sleep 5 && i3-voyager-rgb
```

### Run as systemd User Service

```bash
# Install the service
cp ~/repos/dotfiles/i3-voyager-rgb.service ~/.config/systemd/user/

# Enable and start the service
systemctl --user daemon-reload
systemctl --user enable i3-voyager-rgb.service
systemctl --user start i3-voyager-rgb.service

# Check status
systemctl --user status i3-voyager-rgb.service

# View logs
journalctl --user -u i3-voyager-rgb.service -f
```

## Scripts Included

- **`i3-voyager-rgb`** - Main wrapper script that runs the monitor
- **`i3-workspace-voyager-rgb-lite`** - Lightweight implementation using i3-msg
- **`i3-workspace-voyager-rgb`** - Full implementation using i3ipc (requires `pip install i3ipc`)
- **`i3-workspace-voyager-rgb-setup`** - Setup script to install i3ipc dependency

## Customization

### LED Mapping

Edit the `WORKSPACE_TO_LED` dictionary in the Python scripts to change which LEDs correspond to which workspaces. The default maps workspaces 1-10 to the F1-F10 keys (LED indices 0-9).

### Colors

Edit the `COLORS` dictionary to change the RGB values for each workspace state:
```python
COLORS = {
    'empty': (0, 0, 0),          # Off
    'occupied': (0, 100, 255),    # Blue
    'visible': (0, 255, 100),     # Green
    'focused': (255, 255, 255),   # White
    'urgent': (255, 0, 0),        # Red
}
```

## Troubleshooting

1. **"Cannot connect to Keymapp"**
   - Make sure Keymapp is running
   - Check that your Voyager is connected

2. **"No module named 'keymapp_pb2'"**
   - Ensure the speech-to-text repo is set up
   - Run `cd ~/repos/speech-to-text && ./setup.sh`

3. **LEDs not updating**
   - Check that the script is running: `pgrep -f i3-voyager-rgb`
   - Check logs if using systemd: `journalctl --user -u i3-voyager-rgb -f`
   - Restart Keymapp and the script

4. **Wrong LEDs lighting up**
   - The LED indices might be different for your keyboard layout
   - Experiment with the `WORKSPACE_TO_LED` mapping