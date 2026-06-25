import logging
from functools import cached_property

from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.utils.decorators import check_if_not_connected
from lerobot.utils.errors import DeviceAlreadyConnectedError

from .config_bi_rebot_arm_102_leader import BiRebotArm102LeaderConfig
from .config_rebot_arm_102_leader import RebotArm102LeaderConfig
from .rebot_arm_102_leader import RebotArm102Leader

logger = logging.getLogger(__name__)


class BiRebotArm102Leader(Teleoperator):
    """Bimanual wrapper around two reBot Arm 102 leader arms."""

    config_class = BiRebotArm102LeaderConfig
    name = "bi_rebot_arm_102_leader"

    def __init__(self, config: BiRebotArm102LeaderConfig):
        if not config.id:
            raise ValueError("Bimanual reBot Arm 102 leader requires a non-empty id.")
        if config.left_arm_config.port == config.right_arm_config.port:
            raise ValueError("Bimanual reBot Arm 102 leader requires different left and right ports.")

        super().__init__(config)
        self.config = config

        left_arm_config = RebotArm102LeaderConfig(
            id=f"{config.id}_left" if config.id else None,
            calibration_dir=config.calibration_dir,
            port=config.left_arm_config.port,
            baudrate=config.left_arm_config.baudrate,
            joint_ids=config.left_arm_config.joint_ids,
            joint_ranges=config.left_arm_config.joint_ranges,
        )

        right_arm_config = RebotArm102LeaderConfig(
            id=f"{config.id}_right" if config.id else None,
            calibration_dir=config.calibration_dir,
            port=config.right_arm_config.port,
            baudrate=config.right_arm_config.baudrate,
            joint_ids=config.right_arm_config.joint_ids,
            joint_ranges=config.right_arm_config.joint_ranges,
        )

        self.left_arm = RebotArm102Leader(left_arm_config)
        self.right_arm = RebotArm102Leader(right_arm_config)

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {
            **{f"left_{key}": value for key, value in self.left_arm.action_features.items()},
            **{f"right_{key}": value for key, value in self.right_arm.action_features.items()},
        }

    @cached_property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.left_arm.is_connected and self.right_arm.is_connected

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        connected_arms = []
        try:
            self.left_arm.connect(calibrate)
            connected_arms.append(self.left_arm)
            self.right_arm.connect(calibrate)
            connected_arms.append(self.right_arm)
        except Exception:
            for arm in reversed(connected_arms):
                try:
                    arm.disconnect()
                except Exception:
                    pass
            raise

    @property
    def is_calibrated(self) -> bool:
        return self.left_arm.is_calibrated and self.right_arm.is_calibrated

    def calibrate(self) -> None:
        print("\n[BIMANUAL CALIBRATION] Calibrating LEFT reBot Arm 102 leader arm.")
        self.left_arm.calibrate()
        print("\n[BIMANUAL CALIBRATION] Calibrating RIGHT reBot Arm 102 leader arm.")
        self.right_arm.calibrate()

    def configure(self) -> None:
        self.left_arm.configure()
        self.right_arm.configure()

    @check_if_not_connected
    def get_action(self) -> dict[str, float]:
        action = {}
        action.update({f"left_{key}": value for key, value in self.left_arm.get_action().items()})
        action.update({f"right_{key}": value for key, value in self.right_arm.get_action().items()})
        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        raise NotImplementedError("Feedback is not implemented for the bi reBot Arm 102 leader.")

    def disconnect(self) -> None:
        disconnect_errors = []
        for arm in (self.right_arm, self.left_arm):
            if arm.is_connected:
                try:
                    arm.disconnect()
                except Exception as exc:
                    disconnect_errors.append(exc)

        if disconnect_errors:
            raise disconnect_errors[0]
