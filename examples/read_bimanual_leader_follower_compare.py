#!/usr/bin/env python

import argparse
import math
import os
import time

from lerobot_teleoperator_rebot_arm_102 import (
    BiRebotArm102Leader,
    BiRebotArm102LeaderConfig,
    RebotArm102LeaderArmConfig,
)
from lerobot_robot_seeed_b601 import (
    BiSeeedB601DMFollower,
    BiSeeedB601DMFollowerConfig,
    BiSeeedB601RSFollower,
    BiSeeedB601RSFollowerConfig,
    SeeedB601DMFollower,
    SeeedB601DMFollowerArmConfig,
    SeeedB601DMFollowerConfig,
    SeeedB601RSFollower,
    SeeedB601RSFollowerArmConfig,
    SeeedB601RSFollowerConfig,
)


class PassiveSeeedB601DMFollower(SeeedB601DMFollower):
    """Read-only DM follower variant for manual comparison/debugging."""

    def connect(self, calibrate: bool = False) -> None:
        super().connect(calibrate=calibrate)
        self.disable_torque()


class PassiveSeeedB601RSFollower(SeeedB601RSFollower):
    """Read-only RS follower variant for manual comparison/debugging."""

    def connect(self, calibrate: bool = False) -> None:
        super().connect(calibrate=calibrate)
        self.disable_torque()

    def get_observation(self):
        # The RobStride/motorbridge stack only refreshes angle feedback reliably
        # after a MIT command frame. For read-only debugging, send a zero-force
        # MIT command before polling so the arm stays passive while feedback updates.
        for motor in self.motors.values():
            motor.send_mit(0, 0, 0, 0, 0)
        return super().get_observation()


class PassiveBiSeeedB601DMFollower(BiSeeedB601DMFollower):
    """Read-only bimanual DM follower variant for manual comparison/debugging."""

    def __init__(self, config: BiSeeedB601DMFollowerConfig):
        super().__init__(config)
        self.left_arm = PassiveSeeedB601DMFollower(self.left_arm.config)
        self.right_arm = PassiveSeeedB601DMFollower(self.right_arm.config)
        self.cameras = {
            **{f"left_{key}": value for key, value in self.left_arm.cameras.items()},
            **{f"right_{key}": value for key, value in self.right_arm.cameras.items()},
        }


class PassiveBiSeeedB601RSFollower(BiSeeedB601RSFollower):
    """Read-only bimanual RS follower variant for manual comparison/debugging."""

    def __init__(self, config: BiSeeedB601RSFollowerConfig):
        super().__init__(config)
        self.left_arm = PassiveSeeedB601RSFollower(self.left_arm.config)
        self.right_arm = PassiveSeeedB601RSFollower(self.right_arm.config)
        self.cameras = {
            **{f"left_{key}": value for key, value in self.left_arm.cameras.items()},
            **{f"right_{key}": value for key, value in self.right_arm.cameras.items()},
        }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read bimanual reBot Arm 102 leader and B601 follower positions side by side."
    )
    parser.add_argument("--leader-left-port", required=True, help="Left reBot Arm 102 serial port, e.g. /dev/ttyUSB0")
    parser.add_argument("--leader-right-port", required=True, help="Right reBot Arm 102 serial port, e.g. /dev/ttyUSB1")
    parser.add_argument("--leader-id", default="rebot_arm_102_dual")
    parser.add_argument("--leader-baudrate", type=int, default=1000000)
    parser.add_argument("--follower-left-port", required=True, help="Left B601 CAN port, e.g. can0 or /dev/ttyACM0")
    parser.add_argument("--follower-right-port", required=True, help="Right B601 CAN port, e.g. can1 or /dev/ttyACM1")
    parser.add_argument("--follower-id", default="b601_dual")
    parser.add_argument("--follower-type", choices=["dm", "rs"], default="dm")
    parser.add_argument("--follower-left-can-adapter", default="socketcan", help="Left follower CAN adapter: damiao or socketcan")
    parser.add_argument("--follower-right-can-adapter", default="socketcan", help="Right follower CAN adapter: damiao or socketcan")
    parser.add_argument("--follower-dm-serial-baud", type=int, default=921600)
    parser.add_argument("--interval", type=float, default=0.2, help="Polling interval in seconds")
    return parser.parse_args()


def make_follower(args: argparse.Namespace):
    if args.follower_type == "dm":
        config = BiSeeedB601DMFollowerConfig(
            id=args.follower_id,
            left_arm_config=SeeedB601DMFollowerArmConfig(
                port=args.follower_left_port,
                can_adapter=args.follower_left_can_adapter,
                dm_serial_baud=args.follower_dm_serial_baud,
                cameras={},
            ),
            right_arm_config=SeeedB601DMFollowerArmConfig(
                port=args.follower_right_port,
                can_adapter=args.follower_right_can_adapter,
                dm_serial_baud=args.follower_dm_serial_baud,
                cameras={},
            ),
        )
        return PassiveBiSeeedB601DMFollower(config)

    config = BiSeeedB601RSFollowerConfig(
        id=args.follower_id,
        left_arm_config=SeeedB601RSFollowerArmConfig(
            port=args.follower_left_port,
            can_adapter=args.follower_left_can_adapter,
            dm_serial_baud=args.follower_dm_serial_baud,
            cameras={},
        ),
        right_arm_config=SeeedB601RSFollowerArmConfig(
            port=args.follower_right_port,
            can_adapter=args.follower_right_can_adapter,
            dm_serial_baud=args.follower_dm_serial_baud,
            cameras={},
        ),
    )
    return PassiveBiSeeedB601RSFollower(config)


def print_arm_table(
    side: str,
    leader_arm,
    follower_arm,
    raw_positions: dict[str, float],
    leader_action: dict[str, float],
    follower_obs: dict[str, float],
    previous_follower_obs: dict[str, float] | None,
) -> None:
    ranges = leader_arm.config.joint_ranges
    follower_directions = follower_arm.config.joint_directions

    print(f"[{side.upper()} ARM]")
    print(
        f"{'joint':<16} {'raw':>8} {'range':>13} {'leader':>8} "
        f"{'f.dir':>6} {'mapped':>8} {'follower':>9} {'f.step':>8} {'delta':>8}"
    )
    print(
        f"{'-' * 16} {'-' * 8} {'-' * 13} {'-' * 8} "
        f"{'-' * 6} {'-' * 8} {'-' * 9} {'-' * 8} {'-' * 8}"
    )

    for joint in leader_arm.motor_names:
        raw = raw_positions[joint]
        r_min, r_max = ranges[joint]
        leader_pos = leader_action[f"{joint}.pos"]
        follower_direction = follower_directions.get(joint, 1.0)
        mapped = leader_pos * follower_direction
        follower_pos = follower_obs[f"{joint}.pos"]
        previous_follower_pos = (
            previous_follower_obs.get(f"{joint}.pos") if previous_follower_obs is not None else None
        )
        follower_step = (
            follower_pos - previous_follower_pos if previous_follower_pos is not None else math.nan
        )
        delta = follower_pos - mapped
        range_str = f"[{r_min},{r_max}]"
        step_str = "--" if math.isnan(follower_step) else f"{follower_step:.2f}"

        print(
            f"{joint:<16} {raw:8.2f} {range_str:>13} {leader_pos:8.2f} "
            f"{follower_direction:6.1f} {mapped:8.2f} {follower_pos:9.2f} {step_str:>8} {delta:8.2f}"
        )

    print()


def safe_disconnect(arm) -> None:
    if not arm.is_connected:
        return

    try:
        arm.disconnect()
    except Exception as exc:
        print(f"[cleanup warning] failed to disconnect {arm}: {exc}")



def main() -> None:
    args = parse_args()

    leader = BiRebotArm102Leader(
        BiRebotArm102LeaderConfig(
            id=args.leader_id,
            left_arm_config=RebotArm102LeaderArmConfig(
                port=args.leader_left_port,
                baudrate=args.leader_baudrate,
            ),
            right_arm_config=RebotArm102LeaderArmConfig(
                port=args.leader_right_port,
                baudrate=args.leader_baudrate,
            ),
        )
    )
    follower = make_follower(args)

    try:
        leader.connect(calibrate=False)
        follower.connect(calibrate=False)

        if not leader.left_arm.is_calibrated or not leader.right_arm.is_calibrated:
            raise RuntimeError(
                "No reBot Arm 102 calibration file found for one or both leader arms. Calibrate both first."
            )

        print("Reading bimanual leader/follower positions side by side. Press Ctrl+C to stop.")
        previous_left_follower_obs = None
        previous_right_follower_obs = None
        while True:
            left_raw_positions = leader.left_arm._read_raw_positions()
            right_raw_positions = leader.right_arm._read_raw_positions()
            left_leader_action = leader.left_arm.get_action()
            right_leader_action = leader.right_arm.get_action()

            # Refresh follower observations on every loop. Read through the bimanual
            # wrapper so both follower arms are polled every frame before printing.
            follower_obs = observation_positions(follower.get_observation())
            left_follower_obs = strip_side_prefix(follower_obs, "left")
            right_follower_obs = strip_side_prefix(follower_obs, "right")

            os.system("clear")
            print("Reading bimanual leader/follower positions side by side. Press Ctrl+C to stop.")
            print("f.step shows how much the follower reading changed since the previous refresh.\n")
            print_arm_table(
                "left",
                leader.left_arm,
                follower.left_arm,
                left_raw_positions,
                left_leader_action,
                left_follower_obs,
                previous_left_follower_obs,
            )
            print_arm_table(
                "right",
                leader.right_arm,
                follower.right_arm,
                right_raw_positions,
                right_leader_action,
                right_follower_obs,
                previous_right_follower_obs,
            )

            previous_left_follower_obs = left_follower_obs
            previous_right_follower_obs = right_follower_obs
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        for arm in (follower.right_arm, follower.left_arm, leader.right_arm, leader.left_arm):
            safe_disconnect(arm)



def observation_positions(obs: dict[str, float]) -> dict[str, float]:
    return {key: value for key, value in obs.items() if key.endswith('.pos')}


def strip_side_prefix(obs: dict[str, float], side: str) -> dict[str, float]:
    prefix = f"{side}_"
    return {key.removeprefix(prefix): value for key, value in obs.items() if key.startswith(prefix)}


if __name__ == "__main__":
    main()
