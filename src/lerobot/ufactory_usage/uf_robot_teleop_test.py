import yaml
import argparse
import importlib
import logging
import time
from dataclasses import asdict, dataclass
from pprint import pformat
from lerobot.scripts.lerobot_record import register_third_party_plugins
from lerobot.processor import (
    make_default_processors,
)
from lerobot.robots import (  # noqa: F401
    Robot,
    RobotConfig,
    make_robot_from_config,
    ufactory_robot
)
from lerobot.teleoperators import (  # noqa: F401
    Teleoperator,
    TeleoperatorConfig,
    bi_so100_leader,
    homunculus,
    koch_leader,
    make_teleoperator_from_config,
    so100_leader,
    so101_leader,
    gello_xarm,
    space_mouse,
    ufactory_mock
)
from lerobot.utils.control_utils import (
    is_headless,
    init_keyboard_listener
)
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import (
    init_logging,
)

@dataclass
class EvalConfig:
    robot: RobotConfig
    teleop: TeleoperatorConfig
    fps: int = 30

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
    teleop = make_teleoperator_from_config(cfg.teleop)

    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    robot.connect()
    teleop.connect()

    events = {"exit": False}
    listener = None

    if not is_headless():
        from pynput import keyboard

        def on_press(key):
            try:
                if key == keyboard.Key.esc:
                    print("Escape key pressed. Stopping ...")
                    events["exit"] = True
            except Exception as e:
                print(f"Error handling key press: {e}")

        listener, events = init_keyboard_listener(events=events, on_press=on_press)

    sleep_time_s = 1 / cfg.fps

    print("\n********** Test Teleop With Robot **********")
    input('Enter to control robot with teleop >>> ')

    print("\n********** Teleop Control Loop Start **********")

    while not events["exit"]:
        start_loop_t = time.perf_counter()

        # Get robot observation
        obs = robot.get_observation()

        act = teleop.get_action()
        act_processed_teleop = teleop_action_processor((act, obs))

        robot_action_to_send = robot_action_processor((act_processed_teleop, obs))
        robot.send_action(robot_action_to_send)

        dt_s = time.perf_counter() - start_loop_t
        precise_sleep(sleep_time_s - dt_s)
    
    print("\n********** Teleop Control Loop Exit **********")
    if not is_headless() and listener is not None:
        listener.stop()

def main():
    parser = argparse.ArgumentParser(description='configuration args')
    parser.add_argument('-c', '--config', type=str, required=True, 
                       help='configuration file path, e.g.my_config.yaml')
    args = parser.parse_args()
    try:
        with open(args.config, 'r') as f:
            cfg = yaml.safe_load(f)
    except Exception as e:
        print(f"Error loading config yaml file: {e}")
    else:
        register_third_party_plugins()
        cfg['RobotConfig']['cameras'] = {}
        config = instantiate_from_dict(cfg)

        eval_cfg = EvalConfig(robot=config["RobotConfig"], teleop=config["TeleoperatorConfig"])
        eval(eval_cfg)


if __name__ == "__main__":
    main()
