## [MuJoCo](https://github.com/deepmind/mujoco) setup:
1. Download the precompiled binaries and extrackt thme `tar -xvf mujoco*.tar.gz` to ~/.mujoco/.
2. Some depencies: `sudo apt-get install libglfw3 libglew2.1`
3. Setup X11Forwarding on you .ssh/config and on the server /etc/ssh/sshd_config.

### [dm_control](https://github.com/deepmind/dm_control) setup:
1. create python virtualenv: `virtualenv mujoco`
2. `source mujoco/bin/activate`
3. `pip install dm_control`
4. 
