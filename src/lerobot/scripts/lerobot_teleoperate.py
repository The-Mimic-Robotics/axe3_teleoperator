# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Simple script to control a robot from teleoperation.

Example:
lerobot-calibrate \
    --teleop.type=axe3_leader \
    --teleop.port=/dev/ttyACM0 \
    --teleop.id=axe
```shell
lerobot-teleoperate \
    --robot.type=axe3_follower \
    --robot.cameras={} \
    --robot.udp_port=5005 \
    --teleop.type=axe3_leader \
    --teleop.port=/dev/ttyACM1 \
    --teleop.imu_port=5000 \
    --teleop.id=axe \
    --display_data=true 
```

Example teleoperation with bimanual so100:

```shell
lerobot-teleoperate \
  --robot.type=bi_so100_follower \
  --robot.left_arm_port=/dev/tty.usbmodem5A460851411 \
  --robot.right_arm_port=/dev/tty.usbmodem5A460812391 \
  --robot.id=bimanual_follower \
  --robot.cameras='{
    left: {"type": "opencv", "index_or_path": 0, "width": 1920, "height": 1080, "fps": 30},
    top: {"type": "opencv", "index_or_path": 1, "width": 1920, "height": 1080, "fps": 30},
    right: {"type": "opencv", "index_or_path": 2, "width": 1920, "height": 1080, "fps": 30}
  }' \
  --teleop.type=bi_so100_leader \
  --teleop.left_arm_port=/dev/tty.usbmodem5A460828611 \
  --teleop.right_arm_port=/dev/tty.usbmodem5A460826981 \
  --teleop.id=bimanual_leader \
  --display_data=true
```

"""

import logging
import time
from importlib import import_module
from dataclasses import asdict, dataclass
from pprint import pformat
from typing import Any

try:
    from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig  # noqa: F401
except Exception:
    OpenCVCameraConfig = None  # type: ignore[assignment]

try:
    from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig  # noqa: F401
except Exception:
    RealSenseCameraConfig = None  # type: ignore[assignment]
from lerobot.configs import parser
from lerobot.robots import Robot, RobotConfig, make_robot_from_config
from lerobot.teleoperators import Teleoperator, TeleoperatorConfig, make_teleoperator_from_config
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import init_logging, move_cursor_up


def _safe_import(module_name: str) -> None:
    try:
        import_module(module_name)
    except Exception as e:
        logging.debug(f"Skipping optional module '{module_name}': {e}")


for _module in [
    "lerobot.robots.bi_so100_follower",
    "lerobot.robots.earthrover_mini_plus",
    "lerobot.robots.hope_jr",
    "lerobot.robots.koch_follower",
    "lerobot.robots.omx_follower",
    "lerobot.robots.so100_follower",
    "lerobot.robots.so101_follower",
    "lerobot.robots.axe3_follower",
    "lerobot.robots.axe4_follower",
    "lerobot.teleoperators.bi_so100_leader",
    "lerobot.teleoperators.gamepad",
    "lerobot.teleoperators.homunculus",
    "lerobot.teleoperators.keyboard",
    "lerobot.teleoperators.koch_leader",
    "lerobot.teleoperators.omx_leader",
    "lerobot.teleoperators.so100_leader",
    "lerobot.teleoperators.so101_leader",
    "lerobot.teleoperators.axe3_leader",
    "lerobot.teleoperators.axe4_leader",
    "lerobot.teleoperators.axe_leader",
    "lerobot.teleoperators.bi_axe_leader",
]:
    _safe_import(_module)


@dataclass
class TeleoperateConfig:
    # TODO: pepijn, steven: if more robots require multiple teleoperators (like lekiwi) its good to make this possibele in teleop.py and record.py with List[Teleoperator]
    teleop: TeleoperatorConfig
    robot: RobotConfig
    # Limit the maximum frames per second.
    fps: int = 60
    teleop_time_s: float | None = None
    # Display all cameras on screen
    display_data: bool = False


def teleop_loop(
    teleop: Teleoperator,
    robot: Robot,
    fps: int,
    teleop_action_processor: Any,
    robot_action_processor: Any,
    robot_observation_processor: Any,
    display_data: bool = False,
    duration: float | None = None,
    log_rerun_data_fn=None,
):
    """
    This function continuously reads actions from a teleoperation device, processes them through optional
    pipelines, sends them to a robot, and optionally displays the robot's state. The loop runs at a
    specified frequency until a set duration is reached or it is manually interrupted.

    Args:
        teleop: The teleoperator device instance providing control actions.
        robot: The robot instance being controlled.
        fps: The target frequency for the control loop in frames per second.
        display_data: If True, fetches robot observations and displays them in the console and Rerun.
        duration: The maximum duration of the teleoperation loop in seconds. If None, the loop runs indefinitely.
        teleop_action_processor: An optional pipeline to process raw actions from the teleoperator.
        robot_action_processor: An optional pipeline to process actions before they are sent to the robot.
        robot_observation_processor: An optional pipeline to process raw observations from the robot.
    """

    display_len = max(len(key) for key in robot.action_features)
    start = time.perf_counter()

    while True:
        loop_start = time.perf_counter()

        # Get robot observation
        # Not really needed for now other than for visualization
        # teleop_action_processor can take None as an observation
        # given that it is the identity processor as default
        obs = robot.get_observation()

        # Get teleop action
        raw_action = teleop.get_action()

        # Process teleop action through pipeline
        teleop_action = teleop_action_processor((raw_action, obs))

        # Process action for robot through pipeline
        robot_action_to_send = robot_action_processor((teleop_action, obs))

        # Send processed action to robot (robot_action_processor.to_output should return dict[str, Any])
        _ = robot.send_action(robot_action_to_send)

        if display_data:
            # Process robot observation through pipeline
            obs_transition = robot_observation_processor(obs)

            if log_rerun_data_fn is not None:
                log_rerun_data_fn(
                observation=obs_transition,
                action=teleop_action,
                )

            print("\n" + "-" * (display_len + 10))
            print(f"{'NAME':<{display_len}} | {'NORM':>7}")
            # Display the final robot action that was sent
            for motor, value in robot_action_to_send.items():
                print(f"{motor:<{display_len}} | {value:>7.2f}")
            move_cursor_up(len(robot_action_to_send) + 3)

        dt_s = time.perf_counter() - loop_start
        precise_sleep(1 / fps - dt_s)
        loop_s = time.perf_counter() - loop_start
        print(f"Teleop loop time: {loop_s * 1e3:.2f}ms ({1 / loop_s:.0f} Hz)")
        move_cursor_up(1)

        if duration is not None and time.perf_counter() - start >= duration:
            return


@parser.wrap()
def teleoperate(cfg: TeleoperateConfig):
    init_logging()
    logging.info(pformat(asdict(cfg)))
    rr = None
    log_rerun_data_fn = None
    if cfg.display_data:
        try:
            import rerun as rr  # type: ignore[import-not-found]
            from lerobot.utils.visualization_utils import init_rerun, log_rerun_data
        except ImportError as e:
            raise ModuleNotFoundError(
                "display_data=true requires the optional 'rerun' package. "
                "Either install it (pip install rerun-sdk) or run with --display_data=false."
            ) from e

        log_rerun_data_fn = log_rerun_data
        init_rerun(session_name="teleoperation")

    teleop = make_teleoperator_from_config(cfg.teleop)
    robot = make_robot_from_config(cfg.robot)
    try:
        from lerobot.processor import make_default_processors

        teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()
    except ModuleNotFoundError as e:
        if e.name != "torch":
            raise

        logging.warning(
            "torch/processor stack not installed; using minimal passthrough processors for teleoperation."
        )

        def teleop_action_processor(x):
            return x[0]

        def robot_action_processor(x):
            return x[0]

        def robot_observation_processor(x):
            return x

    teleop.connect()
    robot.connect()

    try:
        teleop_loop(
            teleop=teleop,
            robot=robot,
            fps=cfg.fps,
            display_data=cfg.display_data,
            duration=cfg.teleop_time_s,
            teleop_action_processor=teleop_action_processor,
            robot_action_processor=robot_action_processor,
            robot_observation_processor=robot_observation_processor,
            log_rerun_data_fn=log_rerun_data_fn,
        )
    except KeyboardInterrupt:
        pass
    finally:
        if cfg.display_data and rr is not None:
            rr.rerun_shutdown()
        teleop.disconnect()
        robot.disconnect()


def main():
    register_third_party_plugins()
    teleoperate()


if __name__ == "__main__":
    main()
