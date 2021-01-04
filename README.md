# dotFiles

This guide is intended to be run on Ubuntu 18.04.
I use this guide to keep a consistent system configuration between machines and reduce setup-time on fresh installs.

Please note: I use a Colmak keyboard layout and my i3 configuration is adjusted to suit this. 

## Window Manager (i3 & gnome-shell)
This will install a vanilla gnome environment, i3-gaps, dmenu-extended and the configuration files from this repo.
```
sudo add-apt-repository ppa:kgilmer/speed-ricer
sudo apt-get update && sudo apt-get install -y gnome-session i3-gaps polybar xfonts-terminus* gnome-tweak-tool git feh arandr
sudo update-alternatives --config gdm3.css
mkdir ~/repos && cd ~/repos
git clone git@github.com:MarkHedleyJones/dotFiles.git
git clone git@github.com:MarkHedleyJones/dmenu-extended.git
cd dmenu-extended
sudo python setup.py install
ln -s ~/repos/dotFiles/i3 ~/.config
ln -s ~/repos/dotFiles/.Xresources ~/.Xresources
```

### Configure gnome-terminal

```gnome-terminal```

* Right click -> Preferences.
* Text, Custom font, Terminus Regular size 13
* Colors, Untick "Use colors from system theme", Tango Dark
* Colours -> Background -> #263238 (Matches Sublime Material - Dark)
* General -> untick *Show menubar by default in new terminals*


### Configure GTK-3
```
gnome-tweaks
```
Appearance, Applications -> Adwaita-dark

## ROS
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update && sudo apt-get install -y ros-melodic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
sudo apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential
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
* Theme - Material Dark
* Bracket Highlighter
* JSON Reindent
* Markdown Preview
* PyYapf
* Jedi
```
ln -fs ~/repos/dotFiles/Preferences.sublime-settings ~/.config/sublime-text-3/Packages/User
ln -fs ~/repos/dotFiles/snippet_cout.sublime-snippet ~/.config/sublime-text-3/Packages/User
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
cd ~/Downloads
wget https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb
sudo dpkg -i google-chrome-stable_current_amd64.deb
```
Automaticaly call the custom bashrc script on bash start

    echo "eval \"\$(cat ~/repos/dotFiles/bashrc_custom.sh)\"" >> ~/.bashrc

Alternatively, you can set double-shift as capslock by editing:
`/etc/default/keyboard` and setting `XKBOPTIONS="shift:both_capslock"`

### Japanese Language Support
While in Gnome or Unity launch *Language Support*.
If "The language support is not installed correctly" click Install.
Otherwise, click *Install / Remove Languages* and enable Japanese

```
sudo apt-get remove ibus*
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

### SSH Keys
Fist, copy contents of SSH keys into `~/.ssh/id_rsa` and `~/.ssh/id_rsa.pub`
Then set permissions with:
```
chmod 644 ~/.ssh/id_rsa.pub && chmod 600 ~/.ssh/id_rsa
```

### DCC Monitor Control
Install DCCUtil

    sudo apt-get install dccutil

If using Nvidia drivers and xorg, copy Xorg config across (this adds rules to fix i2c bus from Nvidia cards)
    
    sudo mkdir -p /etc/X11/xorg.conf.d
    sudo cp /usr/share/ddcutil/data/90-nvidia-i2c.conf /etc/X11/xorg.conf.d/

Allow current user to access i2c devices (replace 'mark' with your username)

    echo bash -c '"KERNEL==\"i2c-[0-9]*\", OWNER=\"mark\", MODE=\"0660\"" > /etc/udev/rules.d/45-ddcutil-i2c.rules'

Restart





## Dev tools
### General
```
sudo apt-get install -y libpcl-dev pcl-tools libopencv-dev cmake vim-gtk3 clang-format
cd ~/repos
git clone git@github.com:seqsense/ros_style.git
ln -s ~/repos/dotFiles/bin ~/bin
git config --global user.name "Mark Hedley Jones"
git config --global user.email "markhedleyjones@gmail.com"
echo "export PATH=\"\${PATH}:/home/mark/bin\"" >> ~/.bashrc
echo "alias cm='cd ~/catkin_ws ; catkin_make -j$(nproc); rospack profile ; source devel/setup.bash ; cd -'" >> ~/.bashrc
```

### Google Test
```
sudo apt-get install cmake libgtest-dev
cd /usr/src/googletest/googletest
sudo mkdir build && cd build
sudo cmake .. && sudo make && sudo make install
cd .. && sudo rm -rf build
```

### Docker
```
sudo apt-get install apt-transport-https ca-certificates curl software-properties-common
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
sudo apt update && sudo apt-get install -y docker-ce
sudo usermod -aG docker $USER
```
Restart your computer to enable non-root execution of Docker
