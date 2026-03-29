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
Helper to recalibrate your device (robot or teleoperator).

Example:

```shell
lerobot-calibrate \
    --teleop.type=axe3_leader \
    --teleop.port=/dev/ttyACM0 \
    --teleop.id=axe
```
"""

import logging
from importlib import import_module
from dataclasses import asdict, dataclass
from pprint import pformat

import draccus

try:
    from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig  # noqa: F401
except Exception:
    OpenCVCameraConfig = None  # type: ignore[assignment]

try:
    from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig  # noqa: F401
except Exception:
    RealSenseCameraConfig = None  # type: ignore[assignment]

from lerobot.robots import Robot, RobotConfig, make_robot_from_config
from lerobot.teleoperators import Teleoperator, TeleoperatorConfig, make_teleoperator_from_config
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.utils import init_logging


def _safe_import(module_name: str) -> None:
    try:
        import_module(module_name)
    except Exception as e:
        logging.debug(f"Skipping optional module '{module_name}': {e}")


for _module in [
    "lerobot.robots.earthrover_mini_plus",
    "lerobot.robots.hope_jr",
    "lerobot.robots.koch_follower",
    "lerobot.robots.omx_follower",
    "lerobot.robots.so100_follower",
    "lerobot.robots.so101_follower",
    "lerobot.robots.axe3_follower",
    "lerobot.robots.axe4_follower",
    "lerobot.teleoperators.axe3_leader",
    "lerobot.teleoperators.axe4_leader",
    "lerobot.teleoperators.axe_leader",
    "lerobot.teleoperators.bi_axe_leader",
    "lerobot.teleoperators.bi_so100_leader",
    "lerobot.teleoperators.gamepad",
    "lerobot.teleoperators.homunculus",
    "lerobot.teleoperators.keyboard",
    "lerobot.teleoperators.koch_leader",
    "lerobot.teleoperators.omx_leader",
    "lerobot.teleoperators.so100_leader",
    "lerobot.teleoperators.so101_leader",
]:
    _safe_import(_module)


@dataclass
class CalibrateConfig:
    teleop: TeleoperatorConfig | None = None
    robot: RobotConfig | None = None

    def __post_init__(self):
        if bool(self.teleop) == bool(self.robot):
            raise ValueError("Choose either a teleop or a robot.")

        self.device = self.robot if self.robot else self.teleop


@draccus.wrap()
def calibrate(cfg: CalibrateConfig):
    init_logging()
    logging.info(pformat(asdict(cfg)))

    if isinstance(cfg.device, RobotConfig):
        device = make_robot_from_config(cfg.device)
    elif isinstance(cfg.device, TeleoperatorConfig):
        device = make_teleoperator_from_config(cfg.device)

    device.connect(calibrate=False)
    device.calibrate()
    device.disconnect()


def main():
    register_third_party_plugins()
    calibrate()


if __name__ == "__main__":
    main()
