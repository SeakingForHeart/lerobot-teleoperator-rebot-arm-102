import math
import os

import pytest

from lerobot_teleoperator_rebot_arm_102 import (
    BiRebotArm102Leader,
    BiRebotArm102LeaderConfig,
    RebotArm102LeaderArmConfig,
)


def test_bimanual_rebot_arm_102_requires_id():
    with pytest.raises(ValueError, match="non-empty id"):
        BiRebotArm102Leader(
            BiRebotArm102LeaderConfig(
                id=None,
                left_arm_config=RebotArm102LeaderArmConfig(port="/dev/ttyUSB0"),
                right_arm_config=RebotArm102LeaderArmConfig(port="/dev/ttyUSB1"),
            )
        )


def test_bimanual_rebot_arm_102_requires_distinct_ports():
    with pytest.raises(ValueError, match="different left and right ports"):
        BiRebotArm102Leader(
            BiRebotArm102LeaderConfig(
                id="rebot_arm_102_dual",
                left_arm_config=RebotArm102LeaderArmConfig(port="/dev/ttyUSB0"),
                right_arm_config=RebotArm102LeaderArmConfig(port="/dev/ttyUSB0"),
            )
        )


def test_bimanual_rebot_arm_102_assigns_distinct_child_ids():
    leader = BiRebotArm102Leader(
        BiRebotArm102LeaderConfig(
            id="rebot_arm_102_dual",
            left_arm_config=RebotArm102LeaderArmConfig(port="/dev/ttyUSB0"),
            right_arm_config=RebotArm102LeaderArmConfig(port="/dev/ttyUSB1"),
        )
    )

    assert leader.left_arm.id == "rebot_arm_102_dual_left"
    assert leader.right_arm.id == "rebot_arm_102_dual_right"


# Hardware-only test.
# Run with:
#   REBOT_ARM_102_LEFT_PORT=/dev/ttyUSB1 \
#   REBOT_ARM_102_RIGHT_PORT=/dev/ttyUSB0 \
#   PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
#   pytest -q -s tests/test_bimanual_config_validation.py::test_bimanual_rebot_arm_102_connects_and_prints_joint_angles
# Optional:
#   REBOT_ARM_102_BAUDRATE=1000000
#   REBOT_ARM_102_DUAL_ID=rebot_arm_102_dual_test
@pytest.mark.hardware
def test_bimanual_rebot_arm_102_connects_and_prints_joint_angles():
    left_port = os.environ.get("REBOT_ARM_102_LEFT_PORT")
    right_port = os.environ.get("REBOT_ARM_102_RIGHT_PORT")
    if not left_port or not right_port:
        pytest.skip(
            "Set REBOT_ARM_102_LEFT_PORT and REBOT_ARM_102_RIGHT_PORT to run the hardware angle-read test."
        )

    baudrate = int(os.environ.get("REBOT_ARM_102_BAUDRATE", "1000000"))
    leader = BiRebotArm102Leader(
        BiRebotArm102LeaderConfig(
            id=os.environ.get("REBOT_ARM_102_DUAL_ID", "rebot_arm_102_dual_test"),
            left_arm_config=RebotArm102LeaderArmConfig(port=left_port, baudrate=baudrate),
            right_arm_config=RebotArm102LeaderArmConfig(port=right_port, baudrate=baudrate),
        )
    )

    try:
        leader.connect(calibrate=False)
        assert leader.is_connected

        action = leader.get_action()
        expected_keys = {
            f"left_{key}" for key in leader.left_arm.action_features
        } | {
            f"right_{key}" for key in leader.right_arm.action_features
        }
        assert set(action) == expected_keys

        print("\n[BIMANUAL REBOT ARM 102 JOINT ANGLES]")
        for side, arm in (("left", leader.left_arm), ("right", leader.right_arm)):
            raw_positions = arm._read_raw_positions()
            arm_action = arm.get_action()
            print(f"[{side.upper()}]")
            for motor_name in arm.motor_names:
                raw_value = raw_positions[motor_name]
                action_value = arm_action[f"{motor_name}.pos"]
                assert math.isfinite(raw_value)
                assert math.isfinite(action_value)
                print(f"  {motor_name:<16} raw={raw_value:8.2f} deg  action={action_value:8.2f} deg")
    finally:
        leader.disconnect()
