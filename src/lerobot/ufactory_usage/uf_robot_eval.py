import yaml
import argparse
import importlib
import logging
import time
import torch
from dataclasses import asdict, dataclass
from pprint import pformat
from contextlib import nullcontext
from lerobot.scripts.lerobot_record import register_third_party_plugins
from lerobot.datasets.pipeline_features import aggregate_pipeline_dataset_features, create_initial_features
from lerobot.datasets.utils import build_dataset_frame, combine_feature_dicts
from lerobot.policies.utils import make_robot_action
from lerobot.policies.factory import make_policy, make_pre_post_processors
from lerobot.datasets.lerobot_dataset import LeRobotDatasetMetadata
from lerobot.utils.constants import OBS_STR
from lerobot.processor import (
    make_default_processors,
)
from lerobot.robots import (  # noqa: F401
    Robot,
    RobotConfig,
    make_robot_from_config,
    ufactory_robot
)
from lerobot.utils.control_utils import (
    is_headless,
    init_keyboard_listener,
    predict_action,
)
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import (
    get_safe_torch_device,
    init_logging,
)
from lerobot.configs import parser
from lerobot.configs.policies import PreTrainedConfig
from lerobot.scripts.lerobot_record import DatasetRecordConfig
from lerobot.datasets.lerobot_dataset import LeRobotDataset

@dataclass
class EvalConfig:
    robot: RobotConfig
    dataset: DatasetRecordConfig
    # Whether to control the robot with a policy
    policy: PreTrainedConfig | None = None
    n_episodes: int = 50
    single_task: str | None = "pick_place"

    def __post_init__(self):
        # HACK: We parse again the cli args here to get the pretrained path if there was one.
        policy_path = parser.get_path_arg("policy")
        if policy_path:
            cli_overrides = parser.get_cli_overrides("policy")
            self.policy = PreTrainedConfig.from_pretrained(policy_path, cli_overrides=cli_overrides)
            self.policy.pretrained_path = policy_path

        if self.policy is None:
            raise ValueError("Choose a policy to control the robot")
    
    @classmethod
    def __get_path_fields__(cls) -> list[str]:
        """This enables the parser to load config from the policy using `--policy.path=local/dir`"""
        return ["policy"]


# recursive call, inspired from gello_software:
def instantiate_from_dict(cfg):
    """Instantiate objects from configuration."""
    if isinstance(cfg, dict) and "_target_" in cfg:
        module_path, class_name = cfg["_target_"].rsplit(".", 1)
        cls = getattr(importlib.import_module(module_path), class_name)
        kwargs = {k: v for k, v in cfg.items() if k != "_target_"}
        pp_dict ={k: instantiate_from_dict(v) for k, v in kwargs.items()} 
        print(pp_dict)
        return cls(**{k: instantiate_from_dict(v) for k, v in kwargs.items()})
    elif isinstance(cfg, dict):
        return {k: instantiate_from_dict(v) for k, v in cfg.items()}
    elif isinstance(cfg, list):
        return [instantiate_from_dict(v) for v in cfg]
    else:
        return cfg


def eval(cfg: EvalConfig):
    init_logging()
    logging.info(pformat(asdict(cfg)))

    robot = make_robot_from_config(cfg.robot)

    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    try:
        dataset_metadata = LeRobotDatasetMetadata(repo_id=cfg.dataset.repo_id, root=cfg.dataset.root)
        dataset_features = dataset_metadata.features
        print("Loaded dataset metadata successfully.")
    except Exception:
        dataset_features = combine_feature_dicts(
            aggregate_pipeline_dataset_features(
                pipeline=teleop_action_processor,
                initial_features=create_initial_features(
                    action=robot.action_features
                ),  # TODO(steven, pepijn): in future this should be come from teleop or policy
                use_videos=cfg.dataset.video,
            ),
            aggregate_pipeline_dataset_features(
                pipeline=robot_observation_processor,
                initial_features=create_initial_features(observation=robot.observation_features),
                use_videos=cfg.dataset.video,
            ),
        )
        # Create empty dataset or load existing saved episodes
        dataset = LeRobotDataset.create(
            cfg.dataset.repo_id,
            cfg.dataset.fps,
            root=cfg.dataset.root,
            robot_type=robot.name,
            features=dataset_features,
            use_videos=cfg.dataset.video,
            image_writer_processes=cfg.dataset.num_image_writer_processes,
            image_writer_threads=cfg.dataset.num_image_writer_threads_per_camera * len(robot.cameras),
            batch_encoding_size=cfg.dataset.video_encoding_batch_size,
        )
        dataset_metadata = dataset.meta
        print("Created new dataset metadata successfully.")
    
    if cfg.dataset.fps != dataset_metadata.fps:
        raise ValueError(f"The dataset fps should be equal to requested fps ({dataset_metadata.fps} != {cfg.dataset.fps}).")
        

    policy = make_policy(cfg=cfg.policy, ds_meta=dataset_metadata)
    # policy.eval()

    # The inference device is automatically set to match the detected hardware, overriding any previous device settings from training to ensure compatibility.
    preprocessor_overrides = {
        "device_processor": {"device": str(policy.config.device)},
        "rename_observations_processor": {"rename_map": cfg.dataset.rename_map},
    }

    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=cfg.policy,
        pretrained_path=cfg.policy.pretrained_path,
        preprocessor_overrides=preprocessor_overrides,
        dataset_stats=dataset_metadata.stats
    )

    robot.connect()

    events = {"reset": False, "exit": False}
    listener = None

    if not is_headless():
        from pynput import keyboard

        def on_press(key):
            try:
                if key == keyboard.Key.right:
                    print("Right arrow key pressed. Resetting...")
                    events["reset"] = True
                elif key == keyboard.Key.left:
                    print("Left arrow key pressed. Resetting....")
                    events["reset"] = True
                elif key == keyboard.Key.esc:
                    print("Escape key pressed. Stopping ...")
                    events["exit"] = True
            except Exception as e:
                print(f"Error handling key press: {e}")

        listener, events = init_keyboard_listener(events=events, on_press=on_press)

    device = get_safe_torch_device(policy.config.device, log=True)
    sleep_time_s = 1 / dataset_metadata.fps

    print("\n********** Policy Eval Episode Loop Start **********")

    # with torch.no_grad(), torch.autocast(device_type=device.type) if cfg.policy.use_amp else nullcontext():
    while True:
        robot.configure()
        policy.reset()
        preprocessor.reset()
        postprocessor.reset()

        while True:
            start_loop_t = time.perf_counter()

            if events["reset"] or events["exit"]:
                events["reset"] = False
                print("\n********** Policy Eval Episode (Reset) **********")
                break

            # Get robot observation
            obs = robot.get_observation()

            # Applies a pipeline to the raw robot observation, default is IdentityProcessor
            obs_processed = robot_observation_processor(obs)

            observation_frame = build_dataset_frame(dataset_features, obs_processed, prefix=OBS_STR)

            action_values = predict_action(
                observation=observation_frame,
                policy=policy,
                device=device,
                preprocessor=preprocessor,
                postprocessor=postprocessor,
                use_amp=policy.config.use_amp,
                task=cfg.single_task,
                robot_type=robot.robot_type,
            )
            act_processed_policy = make_robot_action(action_values, dataset_features)
            robot_action_to_send = robot_action_processor((act_processed_policy, obs))
            robot.send_action(robot_action_to_send)
        
            dt_s = time.perf_counter() - start_loop_t
            precise_sleep(sleep_time_s - dt_s)

        if events["exit"]:
            break

    print("\n********** Policy Eval Loop Exit **********")
    if not is_headless() and listener is not None:
        listener.stop()

def main():
    parser = argparse.ArgumentParser(description='configuration args')
    parser.add_argument('-c', '--config', type=str, required=True, 
                       help='configuration file path, e.g.my_config.yaml')
    parser.add_argument('--policy.path', type=str, required=True, 
                       help='configuration file path, e.g.my_config.yaml')
    args = parser.parse_args()
    try:
        with open(args.config, 'r') as f:
            cfg = yaml.safe_load(f)
    except Exception as e:
        print(f"Error loading config yaml file: {e}")
    else:
        register_third_party_plugins()
        config = instantiate_from_dict(cfg)

        eval_cfg = EvalConfig(robot=config["RobotConfig"], dataset=config["DatasetRecordConfig"])
        eval(eval_cfg)


if __name__ == "__main__":
    main()
