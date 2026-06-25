from lerobot_teleoperator_rebot_arm_102 import BiRebotArm102Leader, BiRebotArm102LeaderConfig, RebotArm102LeaderArmConfig


def test_bimanual_rebot_arm_102_calibration_prints_left_right_prompts(monkeypatch, capsys):
    leader = BiRebotArm102Leader(
        BiRebotArm102LeaderConfig(
            id="rebot_arm_102_dual",
            left_arm_config=RebotArm102LeaderArmConfig(port="/dev/ttyUSB0"),
            right_arm_config=RebotArm102LeaderArmConfig(port="/dev/ttyUSB1"),
        )
    )

    calls = []

    def left_calibrate():
        calls.append("left")

    def right_calibrate():
        calls.append("right")

    leader.left_arm.calibrate = left_calibrate
    leader.right_arm.calibrate = right_calibrate

    leader.calibrate()

    out = capsys.readouterr().out
    assert "[BIMANUAL CALIBRATION] Calibrating LEFT reBot Arm 102 leader arm." in out
    assert "[BIMANUAL CALIBRATION] Calibrating RIGHT reBot Arm 102 leader arm." in out
    assert calls == ["left", "right"]
