#!/usr/bin/env python

import time
import numpy as np
from lerobot.cameras.utils import make_cameras_from_configs

from lerobot.robots import Robot
from .config_uf_robot import UFRobotConfig

from xarm.wrapper import XArmAPI
from threading import Thread, Event, Lock
from .uf_report_utils import *

## Configurations:
MAX_LINEAR_VELOCITY_MM = 200
MAX_JOINT_VELOCITY_RAD = 1.6
INIT_SYNC_JOINT_VELOCITY_RAD = 0.2

CARTESIAN_OBS_KEYS = [
    "pose.x", "pose.y", "pose.z", "pose.rx", "pose.ry", "pose.rz",
    # un-comment if you need more features below:
    # "velo.x", "velo.y", "velo.z", "velo.rx", "velo.ry", "velo.rz",
]

CARTESIAN_ACTION_KEYS = [
    "pose.x", "pose.y", "pose.z", "pose.rx", "pose.ry", "pose.rz",
]


class UFRobot(Robot, Thread):

    config_class = UFRobotConfig
    name = "UFACTORY Robot"

    def __init__(self, config: UFRobotConfig):
        super().__init__(config)
        Thread.__init__(self)
        self.config = config
        self._dof = config.robot_dof 
        if self._dof == None or (not self._dof in (5,6,7)):
            raise ValueError(f"Please specify the correct DOF uf_robot!, got {self._dof}")
        
        self._control_space = self.config.control_space

        self.real_arm = None
        self.cameras = make_cameras_from_configs(config.cameras)

        self._is_connected = False
        self._is_calibrated =True

        self.logs = {}

        self.GRIPPER_OPEN = 800
        self.GRIPPER_CLOSE = 0

        self._cmd_cnt = 0

        self.report_stop_event = Event()
        self._rt_report_normal = False
        self._update_lock = Lock()
        self._use_rt_report = (self._control_space == "cartesian") # Cartesian observations must utilize rt_report
        self._cart_obs_has_vel = any('velo.' in key for key in CARTESIAN_OBS_KEYS)
        self._jnt_obs_has_vel = self.config.observe_joint_vel


    @property
    def _robot_state_features(self)-> dict:
        if self._control_space == "joint":
            state_features = {f"J{motor}.pos": float for motor in range(1, self._dof+1)}
            if self._jnt_obs_has_vel:
                state_features.update({f"J{motor}.vel": float for motor in range(1, self._dof+1)})
            if self.config.gripper_control:
                state_features.update({"gripper.pos": float})
        elif self._control_space == "cartesian":
            state_features = {key: float for key in CARTESIAN_OBS_KEYS}
            if self.config.gripper_control:
                state_features.update({"gripper.pos": float})
        else:
            raise ValueError(f"Please check the given control space of uf_robot! got {self._control_space}")
        return state_features

    @property
    # CHECK!! channel first or last?
    def _cam_features(self) -> dict:
        cam_ft = {}
        for cam_key, cam in self.cameras.items():
            cam_ft[f"{cam_key}"] = (cam.height, cam.width, 3)
        return cam_ft

    @property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._robot_state_features, **self._cam_features}

    @property
    def action_features(self)-> [str, type]:
        if self._control_space == "joint":
            action_ft = {f"J{motor}.pos": float for motor in range(1, self._dof+1)}
        elif self._control_space == "cartesian":
            action_ft = {key: float for key in CARTESIAN_ACTION_KEYS}
        else:
            raise ValueError(f"Please check the given control space of uf_robot! got {self._control_space}")
        # Consider adding velocity configuration ??
        if self.config.gripper_control:
            action_ft.update({"gripper.pos": float})
        return action_ft

    def connect(self, calibrate: bool = True) -> None:
        self.real_arm = XArmAPI(self.config.robot_ip)
        time.sleep(0.2)
        self._is_connected = self.real_arm.connected
        if not self._is_connected:
            print(f"UF Robot connection Failed, please check the hardware availability at ip: {self.config.robot_ip}")
            raise ConnectionError()

        if not self._dof == self.real_arm.axis:
            print(f"[ERROR: ] Real Robot DOF({self.real_arm.axis}) does not match configuration ({self._dof})!")
            self._is_connected = False
            raise ConnectionError()

        for cam in self.cameras.values():
            cam.connect()
            self._is_connected = self._is_connected and cam.is_connected

        if not self._is_connected:
            print("Could not connect to the cameras, check that all cameras are plugged-in.")
            raise ConnectionError()

        self.configure()
        if calibrate:  
            self.calibrate()

        self._is_connected = True

    def configure(self) -> None:
        self.real_arm.motion_enable()
        self.real_arm.clean_error()
        self.real_arm.set_mode(0)  # set to idle mode
        self.real_arm.set_state(0)  # set to start state
        time.sleep(0.5)
        self.real_arm.set_servo_angle(angle=self.config.start_joints, is_radian=True, wait=True)

        if self._control_space == "joint":
            self.real_arm.set_mode(6) 
        elif self._control_space == "cartesian":
            self.real_arm.set_mode(7)
        else:
            raise ValueError(f"Please check the given control space of uf_robot! got {self._control_space}")

        self.real_arm.set_state(0)

        if not self._get_arm_err() == 0:
            raise RuntimeError(f"Failed to set correct state to UF robot! Controller Error code: {self._get_arm_err()} !")

        if self.config.gripper_control:
            self.real_arm.set_gripper_enable(True)
            self.real_arm.set_gripper_mode(0)
            self.real_arm.set_gripper_speed(3000)
            self.real_arm.set_gripper_position(800)
            if not self._get_arm_err() == 0:
                raise RuntimeError(f"Failed to set correct state to Gripper! Controller Error code: {self._get_arm_err()} !")
        
        if self._use_rt_report:
            self.start()
        time.sleep(0.2)

    def calibrate(self) -> None:
        self._is_calibrated = True
        pass # CHECK! currently No-op

    def _get_arm_state(self):
        arm_state = self.real_arm.get_state()[1]
        # print(f"_get_arm_state() = {arm_state}")
        return arm_state

    def _get_arm_err(self) -> list:
        # return [error_code, warn_code]
        arm_err_warn = self.real_arm.get_err_warn_code()[1]
        # print(f"_get_arm_err() = {arm_err_warn}")
        return arm_err_warn[0]

    def get_observation(self) -> dict[str, np.ndarray]:
        obs_dict = {}

        # Read Stretch state
        before_read_t = time.perf_counter()
        if self._control_space == "joint":
            code, states = self.real_arm.get_joint_states(is_radian=True, num=3)
            pos_list = states[0].copy()
            obs_dict = {f"J{k+1}.pos": pos_list[k] for k in range(self._dof)}
            if self._jnt_obs_has_vel:
                vel_list = states[1].copy()
                obs_dict.update({f"J{k+1}.vel": vel_list[k] for k in range(self._dof)})
        elif self._control_space == "cartesian":
            if not self._rt_report_normal:
                raise ConnectionError("RT Report for target robot NOT READY! ")

            with self._update_lock:
                pos_list = self.rt_actual_tcp_pose.copy()
                vel_list = self.rt_actual_tcp_speed.copy()
                # pos_cmd_list = self.rt_cmd_tcp_pose.copy()
                # vel_cmd_list = self.rt_cmd_tcp_vel.copy()
                # jpos_fbk_list = self.rt_actual_joint_pos.copy()
                # jvel_fbk_list = self.rt_actual_joint_speed.copy()

            obs_dict = {"pose.x": pos_list[0],"pose.y": pos_list[1],"pose.z": pos_list[2],"pose.rx": pos_list[3],"pose.ry": pos_list[4],"pose.rz": pos_list[5]}
            if self._cart_obs_has_vel:
                obs_dict.update({"velo.x": vel_list[0], "velo.y": vel_list[1], "velo.z": vel_list[2], "velo.rx": vel_list[3], "velo.ry": vel_list[4], "velo.rz": vel_list[5]})
        else:
            ValueError(f"Please check the given control space of uf_robot! got {self._control_space}")
        
        if self.config.gripper_control:    
            # TODO: Add gripper pose
            code, grippos = self.real_arm.get_gripper_position()

            self.logs["read_pos_dt_s"] = time.perf_counter() - before_read_t
            # states[1] as joint velo
            grippos_norm = ( self.GRIPPER_OPEN - grippos ) / (self.GRIPPER_OPEN - self.GRIPPER_CLOSE)
            obs_dict["gripper.pos"] = grippos_norm

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            before_camread_t = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            self.logs[f"async_read_camera_{cam_key}_dt_s"] = time.perf_counter() - before_camread_t

        return obs_dict

    def send_action(self, action: dict) -> np.ndarray:
        if not self._is_connected:
            raise ConnectionError()
        if self.real_arm.error_code != 0:
            return action

        before_write_t = time.perf_counter()
        if self._control_space == "joint":
            # first sync with gello or other control device SLOWLY!
            jnt_spd = INIT_SYNC_JOINT_VELOCITY_RAD if self._cmd_cnt < 20 else MAX_JOINT_VELOCITY_RAD
            wait_ = True if self._cmd_cnt == 0 else False

            cmd_list = [0]*(self._dof+1)
            for i in range(self._dof):
                cmd_list[i] = action[f"J{i+1}.pos"]
            if self.config.gripper_control:
                cmd_list[self._dof] = action["gripper.pos"]

            # TODO: make mode 6 compatible with wait=True
            if wait_== False and self.real_arm.mode != 6:
                self.real_arm.set_mode(6)
                self.real_arm.set_state(0)
                time.sleep(0.1)
            elif wait_ and self.real_arm.mode != 0:
                self.real_arm.set_mode(0)
                self.real_arm.set_state(0)
                time.sleep(0.1)

            self.real_arm.set_servo_angle(angle=cmd_list[:self._dof], speed=jnt_spd, is_radian=True, wait=wait_)
            gripper_command = self.GRIPPER_OPEN + cmd_list[self._dof] * (self.GRIPPER_CLOSE - self.GRIPPER_OPEN)
        elif self._control_space == "cartesian": # unit: mm? 
            lin_spd = MAX_LINEAR_VELOCITY_MM
            
            if not self._rt_report_normal:
                raise ConnectionError("RT Report for target robot NOT READY! ")
            cmd_list = [action["pose.x"], action["pose.y"], action["pose.z"], action["pose.rx"], action["pose.ry"], action["pose.rz"]]
            self.real_arm.set_position_aa(axis_angle_pose=cmd_list, speed=lin_spd, is_radian=True, wait=False)
            if self.config.gripper_control:
                gripper_command = self.GRIPPER_OPEN + action["gripper.pos"] * (self.GRIPPER_CLOSE - self.GRIPPER_OPEN)

        if self._cmd_cnt < 99999:
            self._cmd_cnt += 1 # CHECK!! possibility of overflow?
        if self.config.gripper_control:
            self.real_arm.set_gripper_position(gripper_command, wait=False) # CHECK! the command unit
        self.logs["write_pos_dt_s"] = time.perf_counter() - before_write_t
        return action

    def print_logs(self) -> None:
        pass

    def disconnect(self) -> None:
        self.real_arm.set_state(4) # stop
        self.real_arm.set_mode(0)
        if self._use_rt_report:
            self.report_stop_event.set()
            self.join()
        self.real_arm.disconnect()
        # CHECK!! how about gripper? 

        for cam in self.cameras.values():
            cam.disconnect()

        self._is_connected = False

    def is_calibrated(self) -> bool:
        """Whether the robot is currently calibrated or not. Should be always `True` if not applicable"""
        return self._is_calibrated

    def is_connected(self) -> bool:
        """Whether the robot is currently calibrated or not. Should be always `True` if not applicable"""
        return self._is_connected

    def run(self):
        import socket
        
        robot_port = 30000 # DO NOT CHANGE
        # create socket connection
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setblocking(True)
        sock.settimeout(1)
        sock.connect((self.config.robot_ip, robot_port))

        buffer = sock.recv(4)
        print(buffer)
        while len(buffer) < 4:
            buffer += sock.recv(4 - len(buffer))
        size = bytes_to_u32(buffer[:4])
        print(f"UFACTORY Robot ({self.config.robot_ip}) RT Report Thread starts!! =======")
        while not self.report_stop_event.is_set():
            buffer += sock.recv(size - len(buffer))
            if len(buffer) < size:
                continue
            data = buffer[:size]
            buffer = buffer[size:]
            with self._update_lock:
                self.rt_actual_joint_pos = bytes_to_fp32_list(data[116:144])
                self.rt_actual_joint_speed = bytes_to_fp32_list(data[144:172])
                self.rt_cmd_tcp_pose = bytes_to_fp32_list(data[424:448])
                self.rt_cmd_tcp_vel = bytes_to_fp32_list(data[448:472])
                self.rt_actual_tcp_pose = bytes_to_fp32_list(data[472:496])
                self.rt_actual_tcp_speed = bytes_to_fp32_list(data[496:520])
            self._rt_report_normal = True

        self._rt_report_normal = False
        print(f"UFACTORY Robot ({self.config.robot_ip}) RT Report Thread Exit!! =======")
