# qz2_mujoco
mujoco simulation for qingzhui2

1. 目录结构

```
├── model                      // mujoco模型文件
│   ├── qz2_meshes             // mesh文件
│   ├── qingzhui2.xml          // mujoco模型描述
├── qz2_controller             // 脚本文件
│   ├── t1.py                  // mujoco-py 仿真接口
├── urdf                       // qingzhui urdf文件
│   ├── qz2_meshes
│   ├── qingzhui2.urdf
│   ├── qingzhui2.urdf.xacro
```

2. 环境依赖

* linux系统

* 安装mujoco仿真器：https://mujoco.org/
* 安装mujoco-py库：https://github.com/openai/mujoco-py/releases

3. 使用说明

* 模型定义，仿真参数设置：mujoco使用xml语言描述机器人模型，环境模型，以及仿真参数。可按需在qingzhui2.xml中调整仿真参数，加入传感器模型、环境地形结构等，具体方法参考mujoco文档：https://mujoco.readthedocs.io/en/latest/XMLreference.html
* 仿真脚本及接口：见 t1.py 文件。
