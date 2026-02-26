# UFACTORY LeRobot 数据采集、训练与推理使用说明

本文章仅提供数据采集方式，具体使用何种方式进行训练和推理客户可自行决定。

## ⚠️ 重要提示
对于gello数据采集方式：
- **一旦开始数据采集，机械臂与摄像头（D435 / D435i）的相对位置必须保持不变**。
- 推理时的摄像头位置必须与采集时相同。若机械臂或摄像头发生变化，**此前采集的数据将无效**。
- 如发生变化，必须重新执行完整流程：

```
重新采集数据 → 重新训练模型 → 重新推理运行
```

对于pika数据采集方式：
- **两个基站和机械臂相对位置没有要求，只需要保证采集时pika sense在基站范围内，但基站移动后需要重新校准**。
- 采集和推理时基站位置可不相同。


## 1. 硬件要求

### 1.1 推荐硬件配置（示例脚本）

- **机械臂**: [UFACTORY xArm7](https://www.ufactory.cc/xarm-collaborative-robot/) 
- **末端执行器**: [UFACTORY xArm 机械爪](https://www.ufactory.cc/product-page/ufactory-xarm-gripper/)
- **摄像头**: [Intel RealSense D435](https://www.realsenseai.com/products/stereo-depth-camera-d435/) 和 [Intel RealSense D435i](https://www.realsenseai.com/products/depth-camera-d435i//)
- **摄像头支架**: UFACTORY 提供 (可购买或 3D 打印)
  - 购买链接: [UFACTORY 摄像头支架](https://www.ufactory.cc/product-page/ufactory-xarm-camera-stand/)
  - 3D 文件下载: [Realsense_Camera_Stand.STEP](https://www.ufactory.cc/wp-content/uploads/2024/05/CameraStand_1300.zip)
- **遥操设备**： gello, [pika](https://global.agilex.ai/products/pika)

## 2. 遥操设备环境配置

### 2.1 gello采集方式

#### 安装librealsense库

参考：[librealsense](https://github.com/realsenseai/librealsense/blob/master/doc/distribution_linux.md)

##### 注册公钥
```bash
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.realsenseai.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
```

##### Https Support
```bash
sudo apt-get install apt-transport-https
```

##### 添加服务到仓库列表
```bash
echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update
```

##### 安装librealsense库
```bash
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
```

### 2.2 pika采集方式
请参考[pika安装和校准说明](https://github.com/xArm-Developer/ufactory_teleop/blob/main/pika_teleop/README_ZH.md)。  
首次使用Pika Sense 或者**基站位置变动**需要校准。

## 3. lerobot环境配置

### 3.1 创建Python虚拟环境
#### Linux(Using Conda)
```bash
conda create -y -n lerobot python=3.10
conda activate lerobot
```

#### 安装ffmpeg
```bash
conda install ffmpeg -c conda-forge
```

### 3.2 安装lerobot和依赖
#### 安装lerobot
```bash
git clone https://github.com/xArm-Developer/lerobot.git
cd lerobot
pip install -e .
```

#### 安装额外依赖
```bash
cd src/lerobot/ufactory_usage
pip install -r requirements_extra.txt
cd src/gello
# pip install -r requirements.txt
pip install -e third_party/DynamixelSDK/python
pip install numpy tyro
pip install -e .
pip install pyrealsense2  # 2.56.5.9235
pip install git+https://github.com/xArm-Developer/xArm-Python-SDK.git
```

## 4. 数据采集

### 4.1 Teleop控制测试
#### 使用gello(xArm7为例)
1. 修改配置文件这里举例使用xarm7_gello_record_config.yaml
* RobotConfig:
  * robot_ip: 要控制的机械臂IP
  * robot_dof: 机械臂轴数
  * start_joints: 机械臂初始位置
* TeleoperatorConfig:
  * start_joints: 机械臂初始位置
  
2. 控制移动gello各关节位置对应到机械臂初始位置
3. 启动脚本
```bash
python python uf_robot_teleop_test.py --config config/xarm7_gello_record_config.yaml
```
4. 按下回车开始控制
5. 按下'ESC'退出

#### 使用pika(xArm7为例)
1. 修改配置文件这里举例使用xarm7_pika_record_config.yaml
* RobotConfig:
  * robot_ip: 要控制的机械臂IP
  * robot_dof: 机械臂轴数
  * start_joints: 机械臂初始位置
* TeleoperatorConfig:
  * start_joints: 机械臂初始位置
  * gripper_port：pika gripper的串口，可以不修改，会默认识别
  * index_or_path：pika gripper上的鱼眼相机index
  * port：pika sense的串口，可以不修改，会默认识别

2. 将pika移动到操作的初始位置
3. 启动脚本
```bash
python python uf_robot_teleop_test.py --config config/xarm7_pika_record_config.yaml
```

4. 双击pika gripper开始和关闭控制
5. 按下'ESC'退出

### 4.2 采集
**用户可自定义采集方式，lerobot仅定义数据集的格式。**  

以下举例使用gello采集或程序生成随机点进行采集：  
每次采集完按右箭头保存，然后恢复环境，等待下一次采集；左箭头重新开始采集这一个episode；ESC停止采集,并退出程序。  
(根据输出操作: 每次重录或者保存后需要按下回车才继续)

#### 使用gello采集(遥操)
```bash
python uf_robot_record.py --config config/xarm7_gello_record_config.yaml
```

#### 使用程序生成随机目标点后, 程序自动生成数据来采集
```bash
python uf_robot_record.py --config config/xarm7_mock_record_config.yaml
```

#### 使用pika采集(遥操)
```bash
python uf_robot_record.py --config config/xarm7_pika_record_config.yaml
```

### 4.3 恢复采集
#### gello恢复采集
```bash
python uf_robot_record.py --config config/xarm7_gello_record_config.yaml --resume
```

#### 程序生成点恢复采集
```bash
python uf_robot_record.py --config config/xarm7_mock_record_config.yaml --resume
```

#### pika恢复采集
```bash
python uf_robot_record.py --config config/xarm7_pika_record_config.yaml --resume
```
    
## 5. 训练

### 5.1 初次训练

```bash
# 注意: repo_id就是采集时配置文件里面的repo_id
# 这里训练策略policy.type选用act，训练次数steps为80万次
# 训练过程每2万次保存一次结果，结果输出到output_dir对应目录中
python -m lerobot.scripts.lerobot_train \
  --dataset.repo_id=ufactory/xarm7_record_datas \
  --policy.type=act \
  --policy.device=cuda \
  --policy.repo_id=ufactory/xarm7_record_datas \
  --output_dir=outputs/train/xarm7_record_datas \
  --job_name=xarm7_record_datas \
  --steps=800000
```

### 5.2 恢复训练
```bash
python -m lerobot.scripts.lerobot_train \
  --dataset.repo_id=ufactory/xarm7_record_datas \
  --policy.type=act \
  --policy.device=cuda \
  --policy.repo_id=ufactory/xarm7_record_datas \
  --output_dir=outputs/train/xarm7_record_datas \
  --job_name=xarm7_record_datas \
  --steps=800000 \
  --batch_size=8 \
  --save_freq=20000 \
  --resume=true \
  --config_path=outputs/train/xarm7_record_datas/checkpoints/last/pretrained_model/train_config.json
```

## 6. 推理

### 指定模型进行推理
```bash
python uf_robot_eval.py \
  --config config/xarm7_gello_record_config.yaml \
  --policy.path=outputs/train/xarm7_record_datas/checkpoints/last/pretrained_model/
```

## 7. 数据集工具
提供一些数据集工具，方便对采集的数据集进行增删查操作。

### 查看某个索引的episode:
例如查看索引号为17的episode:
```bash
lerobot-dataset-viz \
  --repo-id ufactory/xarm7_record_datas \
  --display-compressed-images true \
  --episode-index 17
```

### 删除某些索引的episodes:
例如删除索引号为18和19的episode:
```bash
lerobot-edit-dataset \
  --repo_id ufactory/xarm7_record_datas \
  --operation.type delete_episodes \
  --operation.episode_indices "[18, 19]"
```

### 合并数据集
```python
from lerobot.datasets.aggregate import aggregate_datasets

aggregate_datasets(
  repo_ids=["ufactory/xarm7_record_datas_1", "ufactory/xarm7_record_datas_2"], # 待合并的数据集ID或本地路径
  aggr_repo_id="ufactory/xarm7_record_datas_merge_1_2", # 合并后数据集的名称
  # data_files_size_in_mb=200,         # 每个数据文件最大200MB
  # video_files_size_in_mb=500         # 每个视频文件最大500MB
)
```

### 8.重要提示
用户需要全面研究整个代码库，并了解相关的配置参数，因为代码中所写的配置并非适用于所有使用场景和设置，所以用户需要研究代码或相关理论，以获取相关知识，并自行进行修改和调整。特别是对于**扩散策略(diffusion policy)**，LeRobot 中的默认参数可能仅用于模拟，并未针对实际机器人场景进行优化。

### 9. 数据集实例
以下为开发测试时[采集的数据集](https://drive.google.com/drive/folders/1Ms25rd2YYGdh3tHPEsTTMU-m1fE7uNYY?usp=sharing)，**仅供参考，不可复用**，因为用户机械臂和摄像头位置和测试时不一致。
* xarm7_act_20260119: gello手工录制一次性抓取成功的点(**关节运动**)，60组；
* xarm7_act_20260127: 在xarm7_act_20260119基础上增加了失败重抓的点20组，共80组；
* xarm7_act_mock_20260126: 程序生成的一步到位的点，60组；
* xarm7_pika_datas_aa_merge_0208_0209_2pi：pika遥操采集的点(**笛卡尔-轴角**)，150组。
  * 提示：使用笛卡尔运动时，无论使用RPY还是轴角采集时，数据Roll或Rx会存在不连续跳变问题，通过`±2π`可解决该问题，示例脚本中已处理。

训练后效果：
* 使用xarm7_act_20260127进行训练，训练50w次。
[![演示视频](assets/inference-gello.png)](https://www.bilibili.com/video/BV12xFjzzEaX/?spm_id_from=333.1387.homepage.video_card.click)

* 使用xarm7_act_mock_20260126，训练50w次。
[![演示视频](assets/inference-scripts.png)](https://www.bilibili.com/video/BV14pf9BVEox/?spm_id_from=333.1387.homepage.video_card.click)

* 使用xarm7_pika_datas_aa_merge_0208_0209_2pi，训练40w次。
[![演示视频](assets/inference-pika.png)](https://www.bilibili.com/video/BV16ccizHE2P/?spm_id_from=333.1387.homepage.video_card.click)

* 失败案例
[![演示视频](assets/failure-cases.png)](https://www.bilibili.com/video/BV1Waf9BTEMr/?spm_id_from=333.1387.homepage.video_card.click)