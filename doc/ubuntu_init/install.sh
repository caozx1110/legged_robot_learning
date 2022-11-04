#!/bin/bash
# install on Ubuntu 20.04 LTS

# 清华镜像源
sudo sed -i "s@http://.*archive.ubuntu.com@https://mirrors.tuna.tsinghua.edu.cn@g" /etc/apt/sources.list
sudo sed -i "s@http://.*security.ubuntu.com@https://mirrors.tuna.tsinghua.edu.cn@g" /etc/apt/sources.list

# update
sudo apt update -y
sudo apt upgrade -y

# pkg
sudo apt-get install git vim tmux zsh curl wget build-essential python3-pip -y

# app
sudo apt install terminator ibus-pinyin -y

# config
pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple

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

# open zsh
/bin/zsh

# oh-my-zsh
echo "[INFO] installing oh-my-zsh..."
sh -c "$(curl -fsSL https://gitee.com/mirrors/oh-my-zsh/raw/master/tools/install.sh)"
sudo chsh -s /bin/zsh

# install powerlevel10k
# clone
git clone --depth=1 https://gitee.com/romkatv/powerlevel10k.git ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/themes/powerlevel10k
# config
sudo sed -i "s@ZSH_THEME=\".*\"@ZSH_THEME=\"powerlevel10k/powerlevel10k\"@g" ~/.zshrc
source ~/.zshrc

# ros
echo "[INFO] installing ros..."
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop-full -y
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source /opt/ros/noetic/setup.zsh" >> ~/.zshrc

# TODO: docker