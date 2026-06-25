from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig

from .config_rebot_arm_102_leader import RebotArm102LeaderArmConfig


@TeleoperatorConfig.register_subclass("bi_rebot_arm_102_leader")
@dataclass
class BiRebotArm102LeaderConfig(TeleoperatorConfig):
    """Configuration for a pair of reBot Arm 102 leader arms."""

    left_arm_config: RebotArm102LeaderArmConfig
    right_arm_config: RebotArm102LeaderArmConfig
