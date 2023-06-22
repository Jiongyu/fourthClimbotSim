###  BIRL 双手爪攀爬机器人仿真

![image](https://github.com/Jiongyu/fourthClimbotSim/blob/main/video.gif)

#### 1. 安装

``` bash
# ~/ros/fourthClimbotSimulation/ 为用户自定义文件夹
mkdir -p ~/ros/fourthClimbotSimulation/src/ && cd ~/ros/fourthClimbotSimulation/src/  
git clone https://github.com/Jiongyu/fourthClimbotSim.git

cd  ~/ros/fourthClimbotSimulation/
catkin_make

sudo echo "source ~/ros/fourthClimbotSimulation/devel/setup.bash">> ~/.bashrc  
source ~/.bashrc
```

#### 2. 启动

`demo:`

``` bash
roslaunch mr_simulation start_new_climbot6d_simulation.launch
```

#### 3. 环境生成；机器人路径配置文件

仿真环境配置:`src/mr_simulation/config/climbot6d_simulation_config.yaml`

本工程使用不需要修改任意代码，仅通过修改环境生成配置文件（`src/mr_simulation/config/simulation_environment.yaml`）、
路径配置文件（`src/mr_simulation/config/climbot6d_path_list.txt`）,
然后启动`roslaunch mr_simulation start_new_climbot6d_simulation.launch`, 即可实现所期望的机器人仿真运动。
