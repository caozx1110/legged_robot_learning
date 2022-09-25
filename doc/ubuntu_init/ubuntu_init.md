# ubuntu init by czx

use the scripts to customize czx's ubuntu 20.04

@author: Cao Zhanxiang

@mail: caozx1110@163.com



#### install nvidia GPU driver

​		you need install `sudo apt-get install build-essential` firstly

> + see this [article](https://blog.csdn.net/qq_41166909/article/details/123917167)



#### install needed pkg & software

​		firstly use the Software&Update to change the source

+ terminator
+ zsh
+ vscode
+ typora
+ ros

```sh
./install.sh
```



#### install zsh-theme

```sh
./powerlevel10k_install.sh
```



#### install OS theme

```sh
./theme_install.sh
```

​		then you can choose the theme in gnome-tweak-tool

> + see the [blog](https://blog.csdn.net/Acegem/article/details/126710645)
> + [Graphite theme](https://github.com/vinceliuice/Graphite-gtk-theme)
> + [dash-to-dock](https://blog.csdn.net/qq_45577269/article/details/124140555)
> + [McMojave icon](https://github.com/vinceliuice/McMojave-circle)





