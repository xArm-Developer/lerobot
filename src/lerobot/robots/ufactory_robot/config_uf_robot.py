from dataclasses import dataclass, field
from typing import Tuple
import numpy as np
from lerobot.cameras import CameraConfig
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.cameras.realsense import RealSenseCameraConfig

from lerobot.robots import RobotConfig

@RobotConfig.register_subclass("uf_robot")
@dataclass
class UFRobotConfig(RobotConfig):
    # cameras
    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "overhead": RealSenseCameraConfig(
                serial_number_or_name="Intel RealSense D435I",
                fps=30,
                width=640, # 1280
                height=480, # 720
                # rotation=90,
            ),
            "tool": RealSenseCameraConfig(
                serial_number_or_name="Intel RealSense D435",
                fps=30,
                width=640, # 1280
                height=480, # 720
            ),
        }
    )

    robot_ip: str = "192.168.1.127"
    robot_dof: int | None = None  # Set it correctly if controlling in joint space!
    control_space: str = "joint"
    gripper_control: bool = True
    observe_joint_vel: bool = False # only effective in joint control mode
    start_joints: Tuple[float, ...] = (0, 0, 0, np.pi/2, 0, np.pi/2, 0)
