import yaml
import argparse
import importlib
from lerobot.scripts.lerobot_record import *
from lerobot.teleoperators import (
    gello_xarm,
    space_mouse,
    ufactory_mock
)
from lerobot.teleoperators.ufactory_mock import UFactoryMockTeleop
from lerobot.robots import ufactory_robot


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


@parser.wrap()
def record(cfg: RecordConfig) -> LeRobotDataset:
    init_logging()
    logging.info(pformat(asdict(cfg)))
    if cfg.display_data:
        init_rerun(session_name="recording")

    robot = make_robot_from_config(cfg.robot)
    teleop = make_teleoperator_from_config(cfg.teleop) if cfg.teleop is not None else None

    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

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

    if cfg.resume:
        dataset = LeRobotDataset(
            cfg.dataset.repo_id,
            root=cfg.dataset.root,
            batch_encoding_size=cfg.dataset.video_encoding_batch_size,
        )

        if hasattr(robot, "cameras") and len(robot.cameras) > 0:
            dataset.start_image_writer(
                num_processes=cfg.dataset.num_image_writer_processes,
                num_threads=cfg.dataset.num_image_writer_threads_per_camera * len(robot.cameras),
            )
        sanity_check_dataset_robot_compatibility(dataset, robot, cfg.dataset.fps, dataset_features)
    else:
        # Create empty dataset or load existing saved episodes
        sanity_check_dataset_name(cfg.dataset.repo_id, cfg.policy)
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

    # Load pretrained policy
    policy = None if cfg.policy is None else make_policy(cfg.policy, ds_meta=dataset.meta)
    preprocessor = None
    postprocessor = None
    if cfg.policy is not None:
        preprocessor, postprocessor = make_pre_post_processors(
            policy_cfg=cfg.policy,
            pretrained_path=cfg.policy.pretrained_path,
            dataset_stats=rename_stats(dataset.meta.stats, cfg.dataset.rename_map),
            preprocessor_overrides={
                "device_processor": {"device": cfg.policy.device},
                "rename_observations_processor": {"rename_map": cfg.dataset.rename_map},
            },
        )

    robot.connect()
    if teleop is not None:
        teleop.connect()

    listener, events = init_keyboard_listener()

    print("\n********** Episode Record Loop Start **********")

    with VideoEncodingManager(dataset):
        recorded_episodes = 0
        while recorded_episodes < cfg.dataset.num_episodes and not events["stop_recording"]:
            if teleop is not None and isinstance(teleop, UFactoryMockTeleop):
                if events["stop_recording"]:
                    continue
                teleop.configure(events=events)
                if events["rerecord_episode"]:
                    events["rerecord_episode"] = False
                    events["exit_early"] = False
                    input('\nPress Enter to regenerate random target location >>>>> ')
                    continue
                if events["stop_recording"]:
                    continue

            log_say(f"Recording episode {dataset.num_episodes}", cfg.play_sounds)
            record_loop(
                robot=robot,
                events=events,
                fps=cfg.dataset.fps,
                teleop_action_processor=teleop_action_processor,
                robot_action_processor=robot_action_processor,
                robot_observation_processor=robot_observation_processor,
                teleop=teleop,
                policy=policy,
                preprocessor=preprocessor,
                postprocessor=postprocessor,
                dataset=dataset,
                control_time_s=cfg.dataset.episode_time_s,
                single_task=cfg.dataset.single_task,
                display_data=cfg.display_data,
            )

            if events["rerecord_episode"]:
                log_say("Re-record episode", cfg.play_sounds)
                events["rerecord_episode"] = False
                events["exit_early"] = False
                dataset.clear_episode_buffer()
                input('\nPress Enter to rerecord this episode >>>>> ')
                continue

            if not events['stop_recording']:
                dataset.save_episode()
                recorded_episodes += 1
                input('Press Enter to record at the next episode >>>>> ')

    print("\n********** Episode Record Loop Exit **********")

    robot.disconnect()
    if teleop is not None:
        teleop.disconnect()

    if not is_headless() and listener is not None:
        listener.stop()

    if cfg.dataset.push_to_hub:
        dataset.push_to_hub(tags=cfg.dataset.tags, private=cfg.dataset.private)

    log_say("Exiting", cfg.play_sounds)
    return dataset


def main():
    parser = argparse.ArgumentParser(description='configuration args')
    parser.add_argument('-c', '--config', type=str, required=True, 
                       help='configuration file path, e.g.my_config.yaml')
    parser.add_argument('-r', '--resume',
                       action='store_true', # specify --resume if resume needs to be True
                       default=False,
                       help='Whether contitue recording on existing dataset (default: False)')
    args = parser.parse_args()
    try:
        with open(args.config, 'r') as f:
            cfg = yaml.safe_load(f)
    except Exception as e:
        print(f"Error loading config yaml file: {e}")
    else:
        register_third_party_plugins()
        config = instantiate_from_dict(cfg)

        record_cfg = RecordConfig(resume=args.resume, play_sounds=False, robot=config["RobotConfig"], dataset=config["DatasetRecordConfig"], teleop=config["TeleoperatorConfig"])
        record(record_cfg)


if __name__ == "__main__":
    main()
