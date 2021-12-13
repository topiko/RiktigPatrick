
### Jetson Config
1. [Write ssd disk](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit#write)
2. Setup ssh - in you router open port so outside access possible using: [duckdns](https://www.duckdns.org). [SECURE SSH!!!](https://www.howtogeek.com/443156/the-best-ways-to-secure-your-ssh-server/) 
3. Created more swap for buildin jax - [add swap space:](https://linuxize.com/post/how-to-add-swap-space-on-ubuntu-18-04/)
4. Check cuda version: 
	`/usr/local/cuda-8.0/bin/cuda-install-samples-8.0.sh .`
	`cd NVIDIA_CUDA-8.0_Samples/1_Utilities/deviceQuery`
	`make`
	`./bin/aarch64/linux/release/deviceQuery`
5. Then build jax -  [build jax:](https://forums.developer.nvidia.com/t/jax-on-jetson-nano/182593/9)
	1. Increasing swap space difficulty resolved by: 
	 	--> `mkswap /swapfile`
	2. At some point it will complain that no disutils for python3.9 (https://askubuntu.com/questions/1239829/modulenotfounderror-no-module-named-distutils-util)
		--> `sudo apt install python3.9-distutils`
	3. To change power mode: `sudo nvpmodel -m 1` (1=>5W, 0=>10W)
