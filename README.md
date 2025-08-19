
# dotfiles

This guide is intended to be run on Ubuntu 18.04.
I use this guide to keep a consistent system configuration between machines and reduce setup-time on fresh installs.

Please note: I use a Colmak keyboard layout and my i3 configuration is adjusted to suit this.

### SSH Keys
Fist, copy contents of SSH keys into `~/.ssh/id_rsa` and `~/.ssh/id_rsa.pub`
Then set permissions with:
```
chmod 644 ~/.ssh/id_rsa.pub && chmod 600 ~/.ssh/id_rsa
```

## Vanilla Gnome
```
sudo apt-get remove pipewire-alsa apport*
sudo apt-get install vanilla-gnome-desktop gnome-shell gnome-session gnome-tweaks
```

## Terminus Font (optional)
```
sudo apt-get install -y xfonts-terminus
```
Visit https://files.ax86.net/terminus-ttf/#download and download the Terminus TTF for Java or non-Windows® applications archive.
Extract and install each of the font packages.

### Configure gnome-terminal

* Right click -> Preferences.
* Text, Custom font, "Terminus" size 12
* Colors, Palette, GNOME


## Window Manager (i3 & gnome-shell)
This will install a i3-gaps, dmenu-extended and the configuration files from this repo.
```
sudo add-apt-repository ppa:kgilmer/speed-ricer
sudo apt-get update && sudo apt-get install -y gnome-session i3-gaps polybar xfonts-terminus* gnome-tweak-tool git feh arandr neovim python3-pip fonts-terminus curl
mkdir ~/repos && cd ~/repos
git clone git@github.com:MarkHedleyJones/dotfiles.git
git clone git@github.com:MarkHedleyJones/dmenu-extended.git
cd dmenu-extended
sudo python setup.py install
ln -s ~/repos/dotfiles/i3 ~/.config
ln -s ~/repos/dotfiles/.Xresources ~/.Xresources
```
Install dotfiles
```
~/repos/dotfiles/install.sh
```
You should be able to log-out and log-in to an i3 session now.

## Third Party Software
Using the software installer "Apps" install:
* 0ad
* Gimp
* Inkscape
* Sublime Merge
* Sublime Text
* blender
* code
* firmware-updater
* termdown

Terminal:
```
sudo apt-get install \
    xclip \
```

## Keyboard Settings

### Repeat rate and delay configuration (GNOME)
Open Accessibility under system settings, Typing, Repeat Keys.
Slow the speed down by focusing on the speed slider and pressing the left arrow key 20 times.
Set the delay by moving the slider all the way to the left and pressing right 15 times.

### Repeat rate and delay (i3)
Automatically call the custom shell script on shell start

For bash:
```
echo "source ~/repos/dotfiles/customise_shell.sh" >> ~/.bashrc
```

For zsh:
```
echo "source ~/repos/dotfiles/customise_shell.sh" >> ~/.zshrc
```

### Both Shifts as Capslock
```
Set double-shift as capslock by editing:
`/etc/default/keyboard` and setting `XKBOPTIONS="shift:both_capslock"`
```

### Japanese Language Support
While in Gnome Search ang run *Language Support*.
If "The language support is not installed correctly" click Install.
Click *Install / Remove Languages* and enable Japanese.


### DCC Monitor Control

Install DCCUtil

    sudo apt-get install ddcutil

If using Nvidia drivers and xorg, copy Xorg config across (this adds rules to fix i2c bus from Nvidia cards)

    sudo mkdir -p /etc/X11/xorg.conf.d
    sudo cp /usr/share/ddcutil/data/90-nvidia-i2c.conf /etc/X11/xorg.conf.d/

Allow current user to access i2c devices (replace 'mark' with your username)

    sudo bash -c 'echo "KERNEL==\"i2c-[0-9]*\", OWNER=\"mark\", MODE=\"0660\"" > /etc/udev/rules.d/45-ddcutil-i2c.rules'

Restart

## Aliases Management

Personal aliases are stored in `~/.aliases` (not tracked in git). To sync them across machines:

1. Install GitHub CLI and authenticate:
```bash
sudo apt install gh
gh auth login
```

2. Push your aliases to a private gist:
```bash
aliases-push
```

3. On another machine, pull them down:
```bash
aliases-pull
```

4. Compare local vs remote:
```bash
aliases-diff
```

## Tools
```
Setup git
git config --global user.name "Mark Hedley Jones"
git config --global user.email "markhedleyjones@gmail.com"
```
### Devleopment
Install development tools
```
sudo apt-get install -y \
    libpcl-dev \
    pcl-tools \
    libopencv-dev \
    cmake \
    clang-format
```


### Docker
```
sudo apt-get install apt-transport-https ca-certificates curl software-properties-common
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io
sudo usermod -aG docker $USER
```
Restart your system

#### NVIDIA Container Toolkit
Then follow the instructions on
https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#installing-with-apt
