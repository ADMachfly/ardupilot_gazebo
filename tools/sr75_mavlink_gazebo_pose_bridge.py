#!/usr/bin/env python3
"""SR-75 MAVLink to Gazebo pose bridge.

This bridge keeps Gazebo visual-only: it reads ArduPilot SITL MAVLink state and
sends pose updates to an existing Gazebo model through the set_pose service.
"""

import argparse
import math
import subprocess
import sys
import time


DEFAULT_MAVLINK = "udpin:0.0.0.0:14560"
DEFAULT_WORLD = "sr75_visual_follower_test"
DEFAULT_MODEL = "sr75_launch_stack_visual_follower"
DEFAULT_RATE_HZ = 5.0
DEFAULT_BASE_RPY = (1.5708, 0.0, 0.0)
DEFAULT_ORIGIN_OFFSET = (0.0, 0.0, 0.55)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Bridge ArduPilot/JSBSim MAVLink pose to a Gazebo visual model."
    )
    parser.add_argument(
        "--mavlink",
        default=DEFAULT_MAVLINK,
        help=f"MAVLink connection string. Default: {DEFAULT_MAVLINK}",
    )
    parser.add_argument(
        "--world",
        default=DEFAULT_WORLD,
        help=f"Gazebo world name. Default: {DEFAULT_WORLD}",
    )
    parser.add_argument(
        "--model",
        default=DEFAULT_MODEL,
        help=f"Gazebo model name. Default: {DEFAULT_MODEL}",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=DEFAULT_RATE_HZ,
        help=f"Maximum pose update rate in Hz. Default: {DEFAULT_RATE_HZ:g}",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Read MAVLink and print converted poses without calling Gazebo.",
    )
    parser.add_argument(
        "--origin-offset",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        default=DEFAULT_ORIGIN_OFFSET,
        help="Gazebo ENU origin offset in meters. Default: 0 0 0.55",
    )
    parser.add_argument(
        "--base-rpy",
        nargs=3,
        type=float,
        metavar=("ROLL", "PITCH", "YAW"),
        default=DEFAULT_BASE_RPY,
        help="Base Gazebo model alignment RPY in radians. Default: 1.5708 0 0",
    )
    parser.add_argument(
        "--yaw-offset",
        type=float,
        default=0.0,
        metavar="DEG",
        help="Yaw offset applied to MAVLink attitude before quaternion conversion. Default: 0",
    )
    parser.add_argument(
        "--invert-yaw",
        action="store_true",
        help="Invert MAVLink yaw after applying --yaw-offset.",
    )
    parser.add_argument(
        "--timeout",
        type=int,
        default=3000,
        help="Gazebo set_pose service timeout in milliseconds. Default: 3000",
    )
    return parser.parse_args()


def load_pymavlink():
    try:
        from pymavlink import mavutil
    except ImportError:
        print(
            "Missing pymavlink. Install with: python3 -m pip install pymavlink",
            file=sys.stderr,
        )
        sys.exit(1)
    return mavutil


def rpy_to_quaternion(roll, pitch, yaw):
    """Convert roll/pitch/yaw radians to a quaternion in w, x, y, z order."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return w, x, y, z


def multiply_quaternions(left, right):
    """Multiply two quaternions in w, x, y, z order."""
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


def corrected_attitude_rpy(attitude, yaw_offset_deg, invert_yaw):
    yaw = attitude.yaw + math.radians(yaw_offset_deg)
    if invert_yaw:
        yaw = -yaw
    return attitude.roll, attitude.pitch, yaw


def ned_to_gazebo_enu(local_position_ned, origin_offset):
    """Convert MAVLink LOCAL_POSITION_NED to Gazebo ENU with an optional offset."""
    east = local_position_ned.y
    north = local_position_ned.x
    up = -local_position_ned.z
    return (
        east + origin_offset[0],
        north + origin_offset[1],
        up + origin_offset[2],
    )


def make_pose_request(model_name, position, quaternion):
    x, y, z = position
    w, qx, qy, qz = quaternion
    return (
        f'name: "{model_name}", '
        f"position: {{x: {x:.6f}, y: {y:.6f}, z: {z:.6f}}}, "
        f"orientation: {{w: {w:.9f}, x: {qx:.9f}, y: {qy:.9f}, z: {qz:.9f}}}"
    )


def call_gazebo_set_pose(world_name, request, timeout_ms):
    service_name = f"/world/{world_name}/set_pose/blocking"
    cmd = [
        "gz",
        "service",
        "-s",
        service_name,
        "--reqtype",
        "gz.msgs.Pose",
        "--reptype",
        "gz.msgs.Boolean",
        "--timeout",
        str(timeout_ms),
        "--req",
        request,
    ]

    try:
        result = subprocess.run(
            cmd,
            check=False,
            capture_output=True,
            text=True,
            timeout=max(1.0, (timeout_ms / 1000.0) + 1.0),
        )
    except FileNotFoundError:
        return False, "gz command not found"
    except subprocess.TimeoutExpired:
        return False, "gz service command timed out"

    output = "\n".join(part for part in (result.stdout, result.stderr) if part)
    success = result.returncode == 0 and "data: true" in output.lower()
    return success, output.strip()


def format_rpy_degrees(rpy):
    return f"{math.degrees(rpy[0]):.2f} {math.degrees(rpy[1]):.2f} {math.degrees(rpy[2]):.2f}"


def print_startup(args):
    print("SR-75 MAVLink to Gazebo pose bridge", flush=True)
    print(f"  MAVLink:       {args.mavlink}", flush=True)
    print(f"  Gazebo world:  {args.world}", flush=True)
    print(f"  Gazebo model:  {args.model}", flush=True)
    print(f"  Rate:          {args.rate:g} Hz", flush=True)
    print(
        "  Origin offset: "
        f"{args.origin_offset[0]:.3f} {args.origin_offset[1]:.3f} {args.origin_offset[2]:.3f}",
        flush=True,
    )
    print(f"  Base RPY rad:  {args.base_rpy[0]:.4f} {args.base_rpy[1]:.4f} {args.base_rpy[2]:.4f}", flush=True)
    print(f"  Base RPY deg:  {format_rpy_degrees(args.base_rpy)}", flush=True)
    print(f"  Yaw offset:    {args.yaw_offset:g} deg", flush=True)
    print(f"  Invert yaw:    {args.invert_yaw}", flush=True)
    print(f"  Dry run:       {args.dry_run}", flush=True)


def print_debug(local_position_ned, gazebo_position, corrected_rpy, base_rpy, success):
    status = "true" if success else "false"

    print(
        "NED x/y/z "
        f"{local_position_ned.x:.3f} {local_position_ned.y:.3f} {local_position_ned.z:.3f} | "
        "Gazebo ENU x/y/z "
        f"{gazebo_position[0]:.3f} {gazebo_position[1]:.3f} {gazebo_position[2]:.3f} | "
        f"RPY deg {format_rpy_degrees(corrected_rpy)} | "
        f"base RPY deg {format_rpy_degrees(base_rpy)} | "
        f"set_pose success={status}",
        flush=True,
    )


def main():
    args = parse_args()
    if args.rate <= 0.0:
        print("--rate must be greater than zero", file=sys.stderr)
        return 2

    base_quaternion = rpy_to_quaternion(*args.base_rpy)

    print_startup(args)
    mavutil = load_pymavlink()
    print(f"Connecting MAVLink: {args.mavlink}", flush=True)
    mav = mavutil.mavlink_connection(args.mavlink)
    mav.wait_heartbeat()
    print(
        f"Heartbeat received. Bridging to /world/{args.world}/set_pose/blocking",
        flush=True,
    )

    latest_attitude = None
    latest_position = None
    next_update_time = 0.0
    last_debug_time = 0.0
    interval = 1.0 / args.rate
    last_success = False

    while True:
        msg = mav.recv_match(
            type=["ATTITUDE", "LOCAL_POSITION_NED"], blocking=True, timeout=1.0
        )
        if msg is None:
            continue

        msg_type = msg.get_type()
        if msg_type == "ATTITUDE":
            latest_attitude = msg
        elif msg_type == "LOCAL_POSITION_NED":
            latest_position = msg

        if latest_attitude is None or latest_position is None:
            continue

        now = time.monotonic()
        if now < next_update_time:
            continue
        next_update_time = now + interval

        gazebo_position = ned_to_gazebo_enu(latest_position, args.origin_offset)
        corrected_rpy = corrected_attitude_rpy(
            latest_attitude, args.yaw_offset, args.invert_yaw
        )
        attitude_quaternion = rpy_to_quaternion(*corrected_rpy)
        quaternion = multiply_quaternions(base_quaternion, attitude_quaternion)

        if args.dry_run:
            last_success = False
        else:
            request = make_pose_request(args.model, gazebo_position, quaternion)
            last_success, output = call_gazebo_set_pose(
                args.world, request, args.timeout
            )
            if not last_success and now - last_debug_time >= 1.0:
                print(f"set_pose failed: {output}", file=sys.stderr, flush=True)

        if now - last_debug_time >= 1.0:
            print_debug(
                latest_position,
                gazebo_position,
                corrected_rpy,
                args.base_rpy,
                last_success,
            )
            last_debug_time = now


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nBridge stopped.", flush=True)
