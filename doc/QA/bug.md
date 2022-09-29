# Record of bugs

+ after install [mujoco & mujoco_py](https://github.com/openai/mujoco-py/blob/master/README.md#install-mujoco), when run the python script, it comes up that **`PermissionError: [Errno 13] Permission denied: b'/usr/local/lib/python3.8/dist-packages/mujoco_py-2.1.2.14-py3.8.egg/mujoco_py/generated/mujocopy-buildlock'`**

  + add permission to the file

    ```sh
    sudo chmod a+rwx /usr/local/lib/python3.8/dist-packages/mujoco_py-2.1.2.14-py3.8.egg/mujoco_py/generated/mujocopy-buildlock
    ```


+ when running the mujoco sim render, no GUI came out, **`ERROR: GLEW initalization error: Missing GL version`**

  + ```sh
    sudo gedit ~/.bashrc
    export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so
    source ~/.bashrc
    ```

  + [blog](https://blog.csdn.net/qq_29176963/article/details/106485696)

  + if you use the **zsh**, edit the **~/.zshrc** instead

+ 