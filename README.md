## 项目：基于ROS2 Humble TurtleBot3扩展多目标顺序导航+动态避障

本项目属于模式一课堂实例扩展，基于 ROS2 Humble、TurtleBot3 仿真平台，在课堂单点定点导航基础上做功能拓展，研究室内两轮差速机器人智能自主导航技术。

**核心功能**
1.基础原有功能：Gazebo 仿真环境、Cartographer 激光 SLAM 建图、AMCL 定位、Nav2 单点导航；
2.自主扩展创新功能：
多目标点顺序巡逻：读取配置坐标，机器人自动依次遍历全部点位并循环；
动态障碍物避障：依靠激光雷达 + DWA 算法，途中遇障碍自动绕行、重规划路径；
3.辅助功能：终端彩色文字实时播报导航运行状态，直观展示任务进度。

开发模式：模式1 课堂基础定点导航实例扩展
开发环境：Ubuntu22.04 + ROS2 Humble + Gazebo11 + RViz2
**技术栈**
分类	        技术	        用途	            版本
机器人操作系统	ROS2	        机器人应用开发框架	Humble
仿真环境       Gazebo	      3D物理仿真	        11.x
机器人平台	    TurtleBot3	  差速轮式移动机器人	Burger
编程语言	      Python	      业务逻辑开发	      3.10+
导航框架	      Nav2	        路径规划与导航	    Humble
建图工具	      SLAM Toolbox	同步定位与地图构建	Humble

**一、项目环境依赖完整清单**
1.ROS 2 核心包
ros-humble-desktop
2.TurtleBot3 仿真包
sudo apt install -y ros-humble-turtlebot3-simulations
sudo apt install -y ros-humble-turtlebot3-navigation2
sudo apt install -y ros-humble-slam-toolbox
sudo apt install -y ros-humble-gazebo-ros-pkgs
sudo apt install -y ros-humble-turtlebot3-teleop
3.Gazebo 相关依赖
sudo apt install -y gazebo
sudo apt install -y libgazebo11-dev
4.Python 依赖
pip3 install pyyaml
pip3 install setuptools
5.系统工具
sudo apt install -y python3-pip
sudo apt install -y git
sudo apt install -y wget
sudo apt install -y curl
6.语音功能相关（可选）
sudo apt install -y espeak          # 语音合成（网络限制）
sudo apt install -y espeak-ng       # 语音合成新版（网络限制）
sudo apt install -y festival        # 语音合成备选（网络限制）
pip3 install pyttsx3                # Python TTS 库（网络限制）

**二、编译步骤**
1. 新建工作空间文件夹
##创建ros_ws工作区，内部自动生成src源码目录
mkdir -p ~/ros_ws/src
##进入源码目录src
cd ~/ros_ws/src

2. 放入项目功能包
把完整的 multi_goal_nav 文件夹复制到 ~/ros_ws/src/ 路径下

3.返回工作区根目录编译
##回到ros_ws根目录
cd ~/ros_ws
##编译全部src下功能包
colcon build
##只编译本项目功能包（推荐，编译更快）
colcon build --packages-select multi_goal_nav

4. 刷新环境变量（每次新开终端必执行）
source install/setup.bash
##永久生效（可选，写入终端配置文件）
echo "source ~/ros_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

**三、完整启动流程（分3个终端）**
### 终端1：启动Gazebo仿真环境
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib:$GAZEBO_PLUGIN_PATH
export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
执行命令后完成：
加载 TurtleBot3 Burger 两轮差速机器人模型、激光雷达传感器；
打开 Gazebo 仿真世界，生成标准室内仿真场景，提供机器人运动、障碍物仿真环境；
发布里程计/odom、激光雷达/scan等底层感知话题，为导航提供仿真硬件数据源。
对应项目基础底层支撑功能，是机器人运行的仿真载体。

### 终端2：启动Nav2完整导航栈
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True
执行命令后启动整套导航框架，提供项目核心规划与定位能力：
AMCL 定位模块：融合激光与里程计，实时解算机器人在地图内全局位姿；
A * 全局规划器：根据目标点生成全局最优无碰撞路径；
DWA 局部控制器：实时读取激光雷达数据，动态避障、输出速度指令控制小车；
同步拉起 RViz2 可视化界面，展示地图、机器人轨迹、代价地图、规划路径；
为多目标导航、动态避障两大扩展功能提供算法底层支撑。

### 终端3：启动自研多目标点调度节点（项目创新点）
source ~/ros2_ws/install/setup.bash
ros2 run multi_goal_nav multi_goal_nav_node
基于前面两套环境，实现本项目独有扩展功能：
读取config/goals.yaml预设多组导航坐标，生成目标点执行队列；
通过 Nav2 动作客户端，自动依次下发导航目标；
监听导航完成 / 失败反馈：抵达目标后自动切换下一个点位，点位被障碍物完全封堵时自动跳过；
终端输出彩色文字播报运行状态（加载点位、前往目标、抵达提示、异常告警），替代原方案语音节点做状态反馈；
配合 Nav2 的 DWA 局部规划，在行驶途中遇 Gazebo 动态障碍物时，自动触发局部路径重规划、绕行避障，完整实现多目标点顺序巡逻 + 动态障碍物避障两大核心扩展功能。

***整体串联运行逻辑**
终端 1 提供仿真机器人硬件环境 → 终端 2 提供定位、路径规划、避障算法能力 → 终端 3 自研上层任务调度逻辑，在课堂单点导航基础上拓展多点自动巡逻任务，整套流程完整实现本项目全部设计功能。

***四、两大核心扩展功能实操流程**
1. 多目标点顺序自动导航实操
提前修改配置文件：进入multi_goal_nav/config/goals.yaml，自定义多组虚拟地图坐标，无需改动代码；
启动调度节点后程序自动读取全部目标点队列；
虚拟小车依据 A * 全局算法依次自主行驶至每个目标点位；
终端彩色文字实时播报：加载点位、正在前往、成功抵达提示；
全部点位遍历完成后自动循环巡逻；若目标被障碍物堵死，程序自动跳过当前点位执行下一个。

2. 动态障碍物虚拟避障实操
在 Gazebo 仿真界面插入第二台小车作为动态障碍物；
当行驶中的虚拟机器人激光雷达检测到前方障碍，DWA 局部规划器实时重规划局部路径；
虚拟小车自动减速、转向绕行障碍物，障碍消失后回归预设全局路线，继续前往目标点；
RViz 代价地图实时刷新障碍区域，直观查看虚拟避障运算过程。

**五、功能说明**
1. 基础扩展功能：单点导航升级为多目标点顺序巡逻导航
2. 动态避障：依托Nav2 DWA局部规划器实时规避激光检测障碍物
3. 状态反馈：终端彩色文字播报当前导航进度、到达提示、异常提示
4. 模块化设计：目标点坐标统一存放在yaml配置文件，无需修改代码即可更换巡逻点位

**六、成员分工**
尹毓莹（202334071103，组长/主讲，项目总负责、PPT制作、课堂展示与答辩）
毕巧玲 (202334071107,技术负责人,核心代码开发、导航调试、功能扩展)
曲欣雨 (202334071104,测试与辅助开发, 仿真配置、功能测试、Bug排查)
王满   (202334071112,文档与材料专员,项目说明书、README.md、测试报告)
