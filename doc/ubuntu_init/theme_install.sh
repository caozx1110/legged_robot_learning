#!/bin/bash
sudo apt-get install gnome-tweak-tool gnome-shell-extensions -y

sudo apt-get install git -y

# Graphite-gtk-theme
git clone https://github.com/vinceliuice/Graphite-gtk-theme.git
cd Graphite-gtk-theme
./install.sh
cd ..
sudo rm -rf Graphite-gtk-theme

# McMojave-circle
git clone https://github.com/vinceliuice/McMojave-circle.git
cd McMojave-circle
./install.sh
cd ..
sudo rm -rf McMojave-circle

# then you can choose the theme in gnome-tweak-tool
