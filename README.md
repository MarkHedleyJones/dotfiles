
# dotfiles

This guide is intended to be run on Ubuntu 22.04.
I use this guide to keep a consistent system configuration between machines and reduce setup-time on fresh installs.

Please note: I use a Colmak keyboard layout and my i3 configuration is adjusted to suit this.

### SSH Keys
Fist, copy contents of SSH keys into `~/.ssh/id_rsa` and `~/.ssh/id_rsa.pub`
Then set permissions with:
```
chmod 644 ~/.ssh/id_rsa.pub && chmod 600 ~/.ssh/id_rsa
```

## i3 Window Manager
This will install a vanilla gnome environment, i3 (with gaps), dmenu-extended and the configuration files from this repo.
1. Follow instructions to setup Stable Releases repo and install i3 at: https://i3wm.org/docs/repositories.html
2. Install some basic packages `sudo apt-get update && sudo apt-get install polybar git vim python3-pip arandr feh curl xfonts-terminus`
3. Install dmenu-extended `sudo pip install dmenu-extended` (maybe don't use sudo as it complains)
4. Link configuration files into place by running `install.sh`
5. Log out and log into an i3 session
6. Open a terminal (Meta + Enter), right-click, preferences, check "Custom font" and select Terminus (size 12)
7. Install extra packages

```
sudo apt-get install \
  gimp \
  inkscape \
  blender \
  docker.io \
  docker-buildx


sudo usermod -a -G docker $USER
```

## Visual Studio Code
1. Visit https://code.visualstudio.com/download
2. Download .deb
3. Open terminal and execute `sudo dpkg -i ~/Downloads/code_*_amd64.deb`

```

### Japanese Language Support
While in Gnome or Unity launch *Language Support*.
If "The language support is not installed correctly" click Install.
Otherwise, click *Install / Remove Languages* and enable Japanese

```
sudo apt-get install fcitx-mozc
```
* Run "Input Method"
* Hit OK
* Select "Yes" to update the settings
* Select fcitx and select OK.
* Confirm the settings by cliking OK.

Log out and log into i3:
* Right-click Keyboard icon in system tray and click "Configure"
* Global Config
* Trigger Input Method set to "Super+Space"
* Change *Share State Among Window* to *All*
* *Input Method* -> Click *+* to add new input mode
* Untick *Only Show Current Language*, add *Mozc*


### DCC Monitor Control

DANGER: THIS WILL BREAK YOUR SYSTEM AND STOP YOU FROM BOOTING!
Install DCCUtil

    sudo apt-get install ddcutil

If using Nvidia drivers and xorg, copy Xorg config across (this adds rules to fix i2c bus from Nvidia cards)

    sudo mkdir -p /etc/X11/xorg.conf.d
    sudo cp /usr/share/ddcutil/data/90-nvidia-i2c.conf /etc/X11/xorg.conf.d/

Allow current user to access i2c devices (replace 'mark' with your username)

    sudo bash -c 'echo "KERNEL==\"i2c-[0-9]*\", OWNER=\"mark\", MODE=\"0660\"" > /etc/udev/rules.d/45-ddcutil-i2c.rules'

Restart


## Dev tools
### General
```
sudo apt-get install -y libpcl-dev pcl-tools libopencv-dev cmake vim-gtk3 clang-format
git config --global user.name "Mark Hedley Jones"
git config --global user.email "markhedleyjones@gmail.com"
```
