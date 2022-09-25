#!/bin/bash
# install powerlevel10k

# clone
git clone --depth=1 https://gitee.com/romkatv/powerlevel10k.git ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/themes/powerlevel10k
# config
sudo sed -i "s@ZSH_THEME=\".*\"@ZSH_THEME=\"powerlevel10k/powerlevel10k\"@g" ~/.zshrc
source ~/.zshrc
