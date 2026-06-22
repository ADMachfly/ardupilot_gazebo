#!/usr/bin/env python3
"""SR-75 visual-only MAVLink HUD viewer."""

import argparse
import math
import sys
import time


DEFAULT_MAVLINK = "udpin:0.0.0.0:14560"
DEFAULT_WIDTH = 1280
DEFAULT_HEIGHT = 720
DEFAULT_RATE = 30.0
DEFAULT_BACKGROUND = "synthetic"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Draw a visual-only SR-75 HUD from MAVLink telemetry."
    )
    parser.add_argument(
        "--mavlink",
        default=DEFAULT_MAVLINK,
        help=f"MAVLink connection string. Default: {DEFAULT_MAVLINK}",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=DEFAULT_WIDTH,
        help=f"HUD width in pixels. Default: {DEFAULT_WIDTH}",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=DEFAULT_HEIGHT,
        help=f"HUD height in pixels. Default: {DEFAULT_HEIGHT}",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=DEFAULT_RATE,
        help=f"Maximum redraw rate in Hz. Default: {DEFAULT_RATE:g}",
    )
    parser.add_argument(
        "--background",
        choices=("synthetic", "gazebo"),
        default=DEFAULT_BACKGROUND,
        help=f"HUD background source. Default: {DEFAULT_BACKGROUND}",
    )
    parser.add_argument(
        "--gazebo-image-topic",
        default=None,
        help="Gazebo image topic for camera-backed HUD backgrounds.",
    )
    return parser.parse_args()


def load_dependencies():
    try:
        import cv2
        import numpy as np
        from pymavlink import mavutil
    except ImportError:
        print(
            "Missing dependency. Install with: "
            "python3 -m pip install opencv-python pymavlink",
            file=sys.stderr,
        )
        sys.exit(1)
    return cv2, np, mavutil


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def normalize_heading_deg(value):
    return value % 360.0


def format_value(value, fmt, fallback="--"):
    if value is None:
        return fallback
    return format(value, fmt)


class TelemetryState:
    def __init__(self):
        self.roll_rad = 0.0
        self.pitch_rad = 0.0
        self.heading_deg = None
        self.airspeed = None
        self.altitude = None
        self.climb_rate = None
        self.throttle = None
        self.flight_mode = "WAITING"
        self.aoa_deg = None
        self.battery_percent = None
        self.local_north = None
        self.local_east = None
        self.local_down = None
        self.global_alt_m = None
        self.last_heartbeat = None


class BackgroundSource:
    def __init__(self, mode, topic=None):
        self.mode = mode
        self.topic = topic
        self.label = f"CAM {topic}" if mode == "gazebo" and topic else "CAM synthetic"

    def get_frame(self, cv2, np, width, height):
        return synthetic_background(np, width, height)


class SyntheticBackgroundSource(BackgroundSource):
    def __init__(self):
        super().__init__("synthetic")


class GazeboBackgroundSource(BackgroundSource):
    def __init__(self, topic):
        super().__init__("gazebo", topic=topic)
        self._warned = False
        self._transport_available = False
        self._subscriber = None
        self._latest_frame = None
        self._init_transport()

    def _init_transport(self):
        try:
            import gz.transport13  # noqa: F401
            import gz.msgs10.image_pb2  # noqa: F401
        except ImportError:
            self._transport_available = False
            return
        self._transport_available = True
        # Python Gazebo transport bindings are not available in this environment.
        # Keep the structure so a real subscriber can be added without changing HUD flow.
        self._subscriber = None

    def _warn_once(self):
        if self._warned:
            return
        if not self._transport_available:
            print(
                "Gazebo image background requested, but Python Gazebo transport "
                "bindings are not installed. Falling back to synthetic background. "
                "Keep using: python3 -m pip install opencv-python pymavlink",
                flush=True,
            )
            print(
                "Additional Gazebo Python bindings would be required to subscribe "
                f"to {self.topic} directly. 'gz topic -e' image parsing is not "
                "implemented in this prototype.",
                flush=True,
            )
        else:
            print(
                "Gazebo image background requested, but image subscription is not "
                f"implemented yet for topic {self.topic}. Falling back to synthetic background.",
                flush=True,
            )
        self._warned = True

    def get_frame(self, cv2, np, width, height):
        if self._latest_frame is not None:
            return cv2.resize(self._latest_frame, (width, height), interpolation=cv2.INTER_LINEAR)
        self._warn_once()
        return synthetic_background(np, width, height)


def make_background_source(args):
    if args.background == "synthetic":
        return SyntheticBackgroundSource()
    return GazeboBackgroundSource(args.gazebo_image_topic)


def update_state(state, mavutil, msg):
    msg_type = msg.get_type()

    if msg_type == "ATTITUDE":
        state.roll_rad = msg.roll
        state.pitch_rad = msg.pitch
    elif msg_type == "VFR_HUD":
        state.airspeed = getattr(msg, "airspeed", None)
        state.heading_deg = getattr(msg, "heading", None)
        state.altitude = getattr(msg, "alt", None)
        state.climb_rate = getattr(msg, "climb", None)
        state.throttle = getattr(msg, "throttle", None)
    elif msg_type == "GLOBAL_POSITION_INT":
        if hasattr(msg, "relative_alt"):
            state.altitude = msg.relative_alt / 1000.0
        if hasattr(msg, "alt"):
            state.global_alt_m = msg.alt / 1000.0
        if hasattr(msg, "hdg") and msg.hdg != 65535:
            state.heading_deg = msg.hdg / 100.0
    elif msg_type == "LOCAL_POSITION_NED":
        state.local_north = getattr(msg, "x", None)
        state.local_east = getattr(msg, "y", None)
        state.local_down = getattr(msg, "z", None)
    elif msg_type == "AOA_SSA":
        aoa = getattr(msg, "AOA", None)
        if aoa is None:
            aoa = getattr(msg, "aoa", None)
        if aoa is not None:
            state.aoa_deg = math.degrees(aoa)
    elif msg_type == "BATTERY_STATUS":
        remaining = getattr(msg, "battery_remaining", None)
        if remaining is not None and remaining >= 0:
            state.battery_percent = remaining
    elif msg_type == "HEARTBEAT":
        state.last_heartbeat = time.time()
        state.flight_mode = mavutil.mode_string_v10(msg)


def synthetic_background(np, width, height):
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    horizon_split = int(height * 0.58)
    frame[:horizon_split, :, :] = (24, 24, 30)
    frame[horizon_split:, :, :] = (10, 10, 14)

    for y in range(0, height, 40):
        intensity = 20 if y < horizon_split else 14
        frame[y : y + 1, :, :] = (intensity, intensity, intensity)
    for x in range(0, width, 80):
        frame[:, x : x + 1, :] = (12, 12, 14)
    return frame


def draw_center_crosshair(cv2, frame, cx, cy, color):
    cv2.line(frame, (cx - 24, cy), (cx + 24, cy), color, 2, cv2.LINE_AA)
    cv2.line(frame, (cx, cy - 18), (cx, cy + 18), color, 2, cv2.LINE_AA)
    cv2.circle(frame, (cx, cy), 4, color, 1, cv2.LINE_AA)


def draw_artificial_horizon(cv2, frame, state, width, height, color):
    cx = width // 2
    cy = height // 2
    pitch_deg = math.degrees(state.pitch_rad)
    roll_rad = state.roll_rad
    pitch_pixels = pitch_deg * 6.0
    line_half = int(width * 0.22)

    def rotate_point(x, y):
        cos_r = math.cos(roll_rad)
        sin_r = math.sin(roll_rad)
        rx = x * cos_r - y * sin_r
        ry = x * sin_r + y * cos_r
        return int(cx + rx), int(cy + ry)

    p1 = rotate_point(-line_half, pitch_pixels)
    p2 = rotate_point(line_half, pitch_pixels)
    cv2.line(frame, p1, p2, color, 2, cv2.LINE_AA)

    for pitch_mark in range(-30, 31, 5):
        if pitch_mark == 0:
            continue
        mark_offset = pitch_pixels - (pitch_mark * 6.0)
        mark_half = 44 if pitch_mark % 10 == 0 else 24
        left = rotate_point(-mark_half, mark_offset)
        right = rotate_point(mark_half, mark_offset)
        cv2.line(frame, left, right, color, 1, cv2.LINE_AA)
        if pitch_mark % 10 == 0:
            label_left = rotate_point(-mark_half - 34, mark_offset + 4)
            label_right = rotate_point(mark_half + 10, mark_offset + 4)
            cv2.putText(
                frame,
                f"{abs(pitch_mark)}",
                label_left,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                f"{abs(pitch_mark)}",
                label_right,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
                cv2.LINE_AA,
            )


def draw_roll_scale(cv2, frame, state, width, height, color):
    cx = width // 2
    cy = int(height * 0.2)
    radius = int(min(width, height) * 0.14)
    cv2.ellipse(frame, (cx, cy), (radius, radius), 0, 200, 340, color, 1, cv2.LINE_AA)

    for angle in (-60, -45, -30, -20, -10, 10, 20, 30, 45, 60):
        rad = math.radians(angle - 90)
        outer = (
            int(cx + radius * math.cos(rad)),
            int(cy + radius * math.sin(rad)),
        )
        inner_len = 12 if angle % 30 == 0 else 8
        inner = (
            int(cx + (radius - inner_len) * math.cos(rad)),
            int(cy + (radius - inner_len) * math.sin(rad)),
        )
        cv2.line(frame, inner, outer, color, 1, cv2.LINE_AA)

    pointer_angle = state.roll_rad - math.pi / 2.0
    pointer_tip = (
        int(cx + radius * math.cos(pointer_angle)),
        int(cy + radius * math.sin(pointer_angle)),
    )
    pointer_left = (
        int(cx + (radius - 18) * math.cos(pointer_angle + 0.08)),
        int(cy + (radius - 18) * math.sin(pointer_angle + 0.08)),
    )
    pointer_right = (
        int(cx + (radius - 18) * math.cos(pointer_angle - 0.08)),
        int(cy + (radius - 18) * math.sin(pointer_angle - 0.08)),
    )
    cv2.polylines(
        frame,
        [np_points(pointer_tip, pointer_left, pointer_right)],
        True,
        color,
        2,
        cv2.LINE_AA,
    )


def np_points(*points):
    import numpy as np

    return np.array(points, dtype=np.int32)


def draw_text_block(cv2, frame, lines, x, y, color, align_right=False):
    line_height = 32
    for index, line in enumerate(lines):
        if align_right:
            size, _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.72, 2)
            x_pos = x - size[0]
        else:
            x_pos = x
        y_pos = y + index * line_height
        cv2.putText(
            frame,
            line,
            (x_pos, y_pos),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.72,
            color,
            2,
            cv2.LINE_AA,
        )


def draw_status_bar(cv2, frame, state, width, height, color):
    top_lines = [
        f"MODE {state.flight_mode}",
        f"HDG {format_value(state.heading_deg, '.1f')}",
        f"AOA {format_value(state.aoa_deg, '.1f')}",
        f"BAT {format_value(state.battery_percent, '.0f')}%",
    ]
    draw_text_block(cv2, frame, top_lines, 24, 36, color)

    left_lines = [
        f"ROLL {math.degrees(state.roll_rad):.1f} deg",
        f"PITCH {math.degrees(state.pitch_rad):.1f} deg",
        f"N {format_value(state.local_north, '.1f')} m",
        f"E {format_value(state.local_east, '.1f')} m",
    ]
    draw_text_block(cv2, frame, left_lines, 24, height - 112, color)

    right_lines = [
        f"SPD {format_value(state.airspeed, '.1f')} m/s",
        f"ALT {format_value(state.altitude, '.1f')} m",
        f"CLB {format_value(state.climb_rate, '.1f')} m/s",
        f"THR {format_value(state.throttle, '.0f')}%",
    ]
    draw_text_block(cv2, frame, right_lines, width - 24, height - 112, color, align_right=True)


def draw_heading_tape(cv2, frame, heading_deg, width, color):
    if heading_deg is None:
        heading_deg = 0.0

    center_x = width // 2
    top_y = 64
    span_px = 420
    px_per_deg = 6
    cv2.line(frame, (center_x - span_px, top_y), (center_x + span_px, top_y), color, 1, cv2.LINE_AA)

    for delta in range(-60, 61, 5):
        x = center_x + delta * px_per_deg
        tick_height = 16 if delta % 10 == 0 else 8
        cv2.line(frame, (x, top_y), (x, top_y + tick_height), color, 1, cv2.LINE_AA)
        if delta % 10 == 0:
            label = f"{int(normalize_heading_deg(heading_deg + delta)):03d}"
            cv2.putText(
                frame,
                label,
                (x - 18, top_y + 34),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
                cv2.LINE_AA,
            )

    cv2.rectangle(frame, (center_x - 34, 10), (center_x + 34, 42), color, 1)
    cv2.putText(
        frame,
        f"{int(normalize_heading_deg(heading_deg)):03d}",
        (center_x - 24, 34),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        color,
        2,
        cv2.LINE_AA,
    )


def draw_rate_marker(cv2, frame, value, label, x, y, height, color):
    cv2.rectangle(frame, (x - 26, y), (x + 26, y + height), color, 1)
    center = y + height // 2
    clamped = clamp(value if value is not None else 0.0, -20.0, 20.0)
    marker_y = int(center - (clamped / 20.0) * (height // 2 - 10))
    cv2.line(frame, (x - 18, center), (x + 18, center), color, 1, cv2.LINE_AA)
    cv2.rectangle(frame, (x - 18, marker_y - 6), (x + 18, marker_y + 6), color, 1)
    cv2.putText(
        frame,
        label,
        (x - 20, y - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        color,
        1,
        cv2.LINE_AA,
    )


def render_hud(cv2, np, state, width, height, background_source):
    frame = background_source.get_frame(cv2, np, width, height)
    hud_color = (0, 255, 160)
    center_x = width // 2
    center_y = height // 2

    draw_artificial_horizon(cv2, frame, state, width, height, hud_color)
    draw_center_crosshair(cv2, frame, center_x, center_y, hud_color)
    draw_roll_scale(cv2, frame, state, width, height, hud_color)
    draw_heading_tape(cv2, frame, state.heading_deg, width, hud_color)
    draw_status_bar(cv2, frame, state, width, height, hud_color)
    draw_rate_marker(cv2, frame, state.climb_rate, "V/S", width - 90, 180, 220, hud_color)
    draw_rate_marker(cv2, frame, state.aoa_deg, "AOA", 90, 180, 220, hud_color)

    if state.last_heartbeat is None:
        status_text = "WAITING FOR HEARTBEAT"
    else:
        status_text = "LINK OK"
    cv2.putText(
        frame,
        status_text,
        (width - 190, 36),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        hud_color,
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        background_source.label if background_source.mode == "gazebo" and background_source._latest_frame is not None else (
            f"CAM {background_source.topic}" if background_source.mode == "gazebo" and background_source.topic else "CAM synthetic"
        ),
        (24, 72),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        hud_color,
        2,
        cv2.LINE_AA,
    )

    return frame


def main():
    args = parse_args()
    if args.width <= 0 or args.height <= 0:
        print("--width and --height must be greater than zero", file=sys.stderr)
        return 2
    if args.rate <= 0:
        print("--rate must be greater than zero", file=sys.stderr)
        return 2
    if args.background == "gazebo" and not args.gazebo_image_topic:
        print(
            "--gazebo-image-topic is required when --background gazebo is used",
            file=sys.stderr,
        )
        return 2

    cv2, np, mavutil = load_dependencies()
    background_source = make_background_source(args)
    print("SR-75 HUD viewer", flush=True)
    print(f"  MAVLink: {args.mavlink}", flush=True)
    print(f"  Size:    {args.width}x{args.height}", flush=True)
    print(f"  Rate:    {args.rate:g} Hz", flush=True)
    print(f"  Background: {args.background}", flush=True)
    print(
        "  Camera topic: "
        f"{args.gazebo_image_topic if args.gazebo_image_topic else 'synthetic'}",
        flush=True,
    )
    print("Press q or Esc to close the HUD window.", flush=True)

    mav = mavutil.mavlink_connection(args.mavlink)
    mav.wait_heartbeat()
    print("Heartbeat received. Starting HUD.", flush=True)

    state = TelemetryState()
    frame_interval = 1.0 / args.rate
    next_frame_time = 0.0
    window_name = "SR75 HUD Viewer"

    while True:
        msg = mav.recv_match(
            type=[
                "ATTITUDE",
                "VFR_HUD",
                "GLOBAL_POSITION_INT",
                "LOCAL_POSITION_NED",
                "AOA_SSA",
                "BATTERY_STATUS",
                "HEARTBEAT",
            ],
            blocking=False,
        )
        while msg is not None:
            update_state(state, mavutil, msg)
            msg = mav.recv_match(
                type=[
                    "ATTITUDE",
                    "VFR_HUD",
                    "GLOBAL_POSITION_INT",
                    "LOCAL_POSITION_NED",
                    "AOA_SSA",
                    "BATTERY_STATUS",
                    "HEARTBEAT",
                ],
                blocking=False,
            )

        now = time.monotonic()
        if now >= next_frame_time:
            frame = render_hud(
                cv2,
                np,
                state,
                args.width,
                args.height,
                background_source,
            )
            cv2.imshow(window_name, frame)
            next_frame_time = now + frame_interval

        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            break
        time.sleep(0.001)

    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nHUD viewer stopped.", flush=True)
