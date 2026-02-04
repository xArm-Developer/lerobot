# UFACTORY Data Collection Guide(Imitation Learning)

## ⚠️ Important Notice
- Once data collection starts, **the relative position between the robot arm and the camera (D435/D435i) must not be changed**.  
- When making the inference, the position of the camera and the collection must be the same. Otherwise, the collected data will become invalid.  
- In case of change, the entire process must be re-performed:

```
Collect Dataset → Train the model → Perform Inference and Evaluation
```

## 1. Hardware Requirements

### Reference Hardware Setup
- **Robot Arm**: [UFACTORY xArm7](https://www.ufactory.cc/xarm-collaborative-robot/)
- **End Effector**: [UFACTORY xArm Gripper](https://www.ufactory.cc/product-page/ufactory-xarm-gripper/)
- **Camera**:  
  - [Intel RealSense D435](https://www.realsenseai.com/products/stereo-depth-camera-d435/)  
  - [Intel RealSense D435i](https://www.realsenseai.com/products/depth-camera-d435i/)
- **Camera Mount**: Provided by UFACTORY (purchase or 3D print)
  - Purchase: [UFACTORY Camera Stand](https://www.ufactory.cc/product-page/ufactory-xarm-camera-stand/)
  - 3D Model: [Realsense_Camera_Stand.STEP](https://www.ufactory.cc/wp-content/uploads/2024/05/CameraStand_1300.zip)
- **Teleoperation Device**: gello

---

## 2. Software Environment Setup

### 2.1 Install librealsense

Reference: [librealsense Linux Installation Guide](https://github.com/realsenseai/librealsense/blob/master/doc/distribution_linux.md)

#### Register Public Key
```bash
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.realsenseai.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
```

#### Install APT HTTPS Support
```bash
sudo apt-get install apt-transport-https
```

#### Add the server to the list of repository
```bash
echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update
```

#### Install librealsense
```bash
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
```

---

### 2.2 Create Python Virtual Environment

#### Linux (Conda)
```bash
conda create -y -n lerobot python=3.10
conda activate lerobot
```

#### Install ffmpeg
```bash
conda install ffmpeg -c conda-forge
```

---

### 2.3 Install LeRobot and Dependencies

#### Install LeRobot
```bash
git clone https://github.com/xArm-Developer/lerobot.git
cd lerobot
pip install -e .
```

#### Install Additional Dependencies
```bash
cd src/lerobot/ufactory_usage
pip install -r requirements_extra.txt

cd src/gello
pip install -e third_party/DynamixelSDK/python
pip install numpy tyro
pip install -e .
pip install pyrealsense2  # 2.56.5.9235
pip install git+https://github.com/xArm-Developer/xArm-Python-SDK.git
```

## 3. Data Collection

### 3.1 Teleoperation Test

#### Using gello (xArm7)
1. Modify the configuration file (example: `xarm7_gello_record_config.yaml`)
   - **RobotConfig**
     - `robot_ip`: IP address of the robot
     - `robot_dof`: Degrees of freedom
     - `start_joints`: Initial joint positions
   - **TeleoperatorConfig**
     - `start_joints`: Initial joint positions
2. Move the gello joints to match the robot's initial pose
3. Start the script:
```bash
python uf_robot_teleop_test.py --config config/xarm7_gello_record_config.yaml
```
4. Press **Enter** to begin teleoperation
5. Press **ESC** to exit

---

### 3.2 Data Recording
**Users may define their own data collection strategies. LeRobot only enforces the dataset format.**

**Keyboard control** for data recording:  
- "`->`" Exit early: Finish current episode, save and enter reset process for next episode preparation;  
- "`<-`" rerecord_episode + Exit early: Terminate current recording, reset and then re-record this episode;  
- "`Esc`" stop_recording + Exit early: Exit recording process;
- "`Enter`" : Continue

#### Recording with gello(teleoperation)
```bash
python uf_robot_record.py --config config/xarm7_gello_record_config.yaml
```

#### Recording with scripts(Randomly Generated Targets)
```bash
python uf_robot_record.py --config config/xarm7_mock_record_config.yaml
```

---

### 3.3 Resume Data Recording
```bash
# Resume gello-based recording
python uf_robot_record.py --config config/xarm7_gello_record_config.yaml --resume
```

```bash
# Resume random target recording
python uf_robot_record.py --config config/xarm7_mock_record_config.yaml --resume
```

---

## 4. Training

### 4.1 Initial Training(Use ACT)
```bash
# Note:
# repo_id must match the value used during data collection
# policy.type is set to 'act'
# steps = 800,000
# Model checkpoints are saved every 20,000 steps
python -m lerobot.scripts.lerobot_train \
  --dataset.repo_id=ufactory/xarm7_record_datas \
  --policy.type=act \
  --output_dir=outputs/train/xarm7_record_datas \
  --job_name=xarm7_record_datas \
  --policy.device=cuda \
  --policy.repo_id=ufactory/xarm7_record_datas \
  --steps=800000
```

### 4.2 Resume Training
```bash
python -m lerobot.scripts.lerobot_train \
  --dataset.repo_id=ufactory/xarm7_record_datas \
  --policy.type=act \
  --output_dir=outputs/train/xarm7_record_datas \
  --job_name=xarm7_record_datas \
  --policy.device=cuda \
  --policy.repo_id=ufactory/xarm7_record_datas \
  --steps=800000 \
  --batch_size=8 \
  --save_freq=20000 \
  --resume=true \
  --config_path=outputs/train/xarm7_record_datas/checkpoints/last/pretrained_model/train_config.json
```

## 5. Inference & Evaluation

### Run with a Specified Model
```bash
python uf_robot_eval.py --config config/xarm7_gello_record_config.yaml \
  --policy.path=outputs/train/xarm7_record_datas/checkpoints/last/pretrained_model/
```

## 6. Dataset Utilities

### Playback an Episode
Example: view episode index **17**
```bash
lerobot-dataset-viz --repo-id ufactory/xarm7_record_datas --episode-index 17
```

### Delete Episodes
Example: delete episodes **18** and **19**
```bash
lerobot-edit-dataset --repo_id ufactory/xarm7_record_datas \
  --operation.type delete_episodes \
  --operation.episode_indices "[18, 19]"
```

### Merge Datasets
```python
from lerobot.datasets.aggregate import aggregate_datasets

aggregate_datasets(
    repo_ids=[
        "ufactory/xarm7_record_datas_1",
        "ufactory/xarm7_record_datas_2"
    ],
    aggr_repo_id="ufactory/xarm7_record_datas_merge_1_2"
)
```


## 7. Important Notes
Users are expected to thoroughly study the codebase and configuration parameters.  
The provided configurations are **not guaranteed to work for all scenarios** and must be adjusted based on actual hardware setups and task requirements.

In particular, for **diffusion policies**, the default parameters in LeRobot are primarily designed for simulation and **are not optimized for real-world robots**.


## 8. Dataset Examples (Reference Only)
[Test datasets](https://drive.google.com/drive/folders/1Ms25rd2YYGdh3tHPEsTTMU-m1fE7uNYY?usp=sharing) used during development (not reusable):
- **xarm7_act_20260119**: 60 successful single-attempt grasps recorded via gello
- **xarm7_act_20260127**: 80 samples (added 20 failure-and-retry cases)
- **xarm7_act_mock_20260126**: 60 one-shot grasps generated programmatically

> These datasets are **for reference only**.  
> They cannot be reused because the robot–camera calibration differs between users.

