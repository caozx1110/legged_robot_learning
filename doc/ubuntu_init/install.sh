#!/bin/bash

# 清华镜像源
sudo sed -i "s@http://.*archive.ubuntu.com@https://mirrors.tuna.tsinghua.edu.cn@g" /etc/apt/sources.list
sudo sed -i "s@http://.*security.ubuntu.com@https://mirrors.tuna.tsinghua.edu.cn@g" /etc/apt/sources.list

# update
sudo apt update -y
sudo apt upgrade -y

# pkg
sudo apt-get install git vim tmux zsh curl wget build-essential -y

# app
sudo apt install terminator -y

# vscode
echo "[INFO] installing vscode..."
sudo apt install software-properties-common apt-transport-https -y
wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add -
sudo add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
sudo apt update -y
sudo apt install code -y

# typora
echo "[INFO] installing typora..."
wget -qO - https://typoraio.cn/linux/public-key.asc | sudo tee /etc/apt/trusted.gpg.d/typora.asc
sudo add-apt-repository 'deb https://typoraio.cn/linux ./'
sudo apt-get update
sudo apt-get install typora -y

# oh-my-zsh
echo "[INFO] installing oh-my-zsh..."
sh -c "$(wget https://raw.github.com/robbyrussell/oh-my-zsh/master/tools/install.sh -O -)"
chsh -s /bin/zsh

# ros
echo "[INFO] installing ros..."
sudo apt install ros-noetic-desktop-full -y

