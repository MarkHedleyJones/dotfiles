
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
sudo apt-get remove pipewire-alsa
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
Enable bitmap fonts
```
sudo rm /etc/fonts/conf.d/70-no-bitmaps.conf
sudo ln -s ../conf.avail/70-force-bitmaps.conf /etc/fonts/conf.d/
sudo dpkg-reconfigure fontconfig-config
sudo dpkg-reconfigure fontconfig
```
Install dotfiles
```
~/repos/dotfiles/install.sh
```
You should be able to log-out and log-in to an i3 session now.


## ROS
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update && sudo apt-get install -y ros-noetic-desktop-full python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```

## Sublime Text
```
sudo apt-get install -y apt-transport-https
wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -
echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list
sudo apt-get update && sudo apt-get install -y sublime-text sublime-merge clang clang-format
subl
```
Install Package Control then
* Clang Format
* EasyClangComplete
* Bracket Highlighter
* JSON Reindent
* Markdown Preview
* PyYapf - Python Formatter
* python-black
* LiveReload
* Jedi - Python autocompletion
```
ln -fs ~/repos/dotfiles/Preferences.sublime-settings ~/.config/sublime-text/Packages/User
ln -fs ~/repos/dotfiles/snippet_cout.sublime-snippet ~/.config/sublime-text/Packages/User
cd ~/repos
git clone git@github.com:seqsense/ros_style.git
ln -s ~/repos/ros_style/.clang-format ~/
sudo pip install yapf
mkdir -p ~/.config/yapf
ln -s ~/repos/ros_style/.style.yapf ~/.config/yapf

```

## Others
```
sudo apt-get install -y gimp inkscape xclip
sudo apt-get remove -y apport
```
Automaticaly call the custom bashrc script on bash start

    echo "eval \"\$(cat ~/repos/dotfiles/bashrc_custom.sh)\"" >> ~/.bashrc

Alternatively, you can set double-shift as capslock by editing:
`/etc/default/keyboard` and setting `XKBOPTIONS="shift:both_capslock"`

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

Install DCCUtil

    sudo apt-get install ddcutil

If using Nvidia drivers and xorg, copy Xorg config across (this adds rules to fix i2c bus from Nvidia cards)

    sudo mkdir -p /etc/X11/xorg.conf.d
    sudo cp /usr/share/ddcutil/data/90-nvidia-i2c.conf /etc/X11/xorg.conf.d/

Allow current user to access i2c devices (replace 'mark' with your username)

    sudo bash -c 'echo "KERNEL==\"i2c-[0-9]*\", OWNER=\"mark\", MODE=\"0660\"" > /etc/udev/rules.d/45-ddcutil-i2c.rules'

Restart





## Tools
### Devleopment
```
sudo apt-get install -y \
    libpcl-dev \
    pcl-tools \
    libopencv-dev \
    cmake \
    clang-format

git config --global user.name "Mark Hedley Jones"
git config --global user.email "markhedleyjones@gmail.com"
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
