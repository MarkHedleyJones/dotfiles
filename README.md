# dotFiles

This guide is intended to be run on Ubuntu 18.04.
I use this guide to keep a consistent system configuration between machines and reduce setup-time on fresh installs.

Please note: I use a Colmak keyboard layout and my i3 configuration is adjusted to suit this. 

## Window Manager (i3 & gnome-shell)
This will install a vanilla gnome environment, i3-gaps, dmenu-extended and the configuration files from this repo.
```
sudo pacman -S i3-gaps i3lock i3status i3blocks network-manager-applet xorg-xsetroot terminus-font arandr gsimplecal xorg-xauth xorg-xev dhclient
sudo yay -S polybar dmenu-extended-git
ln -s ~/repos/dotFiles/i3 ~/.config
ln -s ~/repos/dotFiles/.config/gsimplecal ~/.config
```

Install the GB, JP, and NZ locales!

### General tools
```
sudo pacman -S meld

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
yay -S --sudoloop --noconfirm ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
sudo rosdep init
rosdep update
```

## Sublime Text
```
sudo pacman -S sublime-text sublime-merge
```
Install Package Control then
* Clang Format
* EasyClangComplete
* Bracket Highlighter
* JSON Reindent
* Markdown Preview
* PyYapf
* LiveReload
* Jedi - Python Autocompletion
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
sudo pacman -S gimp inkscape xclip blender htop gpick ranger
```
Automaticaly call the custom bashrc script on bash start

    echo "eval \"\$(cat ~/repos/dotFiles/bashrc_custom.sh)\"" >> ~/.bashrc

Alternatively, you can set double-shift as capslock by editing:
`/etc/default/keyboard` and setting `XKBOPTIONS="shift:both_capslock"`

### Japanese Language Support

```
yay -S ibus-mozc 
pacman -S otf-ipamjfont otf-ipaexfont adobe-source-han-sans-jp-fonts adobe-source-han-serif-jp-fonts otf-ipafont ttf-hanazono
```

### SSH Keys
Fist, copy contents of SSH keys into `~/.ssh/id_rsa` and `~/.ssh/id_rsa.pub`
Then set permissions with:
```
chmod 644 ~/.ssh/id_rsa.pub && chmod 600 ~/.ssh/id_rsa
```

### DCC Monitor Control
Install DCCUtil

    sudo pacman -S ddcutil

If using Nvidia drivers and xorg, copy Xorg config across (this adds rules to fix i2c bus from Nvidia cards)
    
    sudo mkdir -p /etc/X11/xorg.conf.d
    sudo cp /usr/share/ddcutil/data/90-nvidia-i2c.conf /etc/X11/xorg.conf.d/

Allow current user to access i2c devices (replace 'mark' with your username)

    echo bash -c '"KERNEL==\"i2c-[0-9]*\", OWNER=\"mark\", MODE=\"0660\"" > /etc/udev/rules.d/45-ddcutil-i2c.rules'

Restart


### 3D Mouse
`
   yay -S spacenavd
   sudo systemctl enable spacenavd
   sudo systemctl start spacenavd
`

## Dev tools
### General
```
sudo apt-get install -y libpcl-dev pcl-tools libopencv-dev cmake vim-gtk3 clang-format
cd ~/repos
git clone git@github.com:seqsense/ros_style.git
ln -s ~/repos/dotFiles/bin ~/bin
git config --global user.name "Mark Hedley Jones"
git config --global user.email "markhedleyjones@gmail.com"
echo "export PATH=\"\${PATH}:/home/mark/.local/bin\"" >> ~/.bashrc
echo "alias cm='cd ~/catkin_ws ; catkin_make -j$(nproc); rospack profile ; source devel/setup.bash ; cd -'" >> ~/.bashrc
```


### Docker
```
sudo pacman -S docker
sudo usermod -aG docker $USER
sudo systemctl enable docker
sudo systemctl start docker
yay -S nvidia-container-toolkit
```
Log-out and log-in or restart the computer
