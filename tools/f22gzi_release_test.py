"""F22-GZ-J: Gazebo-only rail-launch + detachable-release test.

Exercises the tracked sr75_launcher_carriage / sr75_uav_release two-model
launcher (worlds/sr75_f22_detachable_launch_test.sdf): confirms prelaunch
rail stability at the intended 20deg elevation / 315deg azimuth, applies a
5800N/3s RATO-equivalent thrust along the rail axis via ApplyLinkWrench,
and releases the UAV from the rail carriage via its native DetachableJoint
once along-rail travel reaches the 1.00-1.20m target window (well short of
the rail's 1.45m hard travel limit).

Usage (with the world already running, e.g.:
  GZ_SIM_RESOURCE_PATH=<repo>/models:<repo>/worlds \
  GZ_SIM_SYSTEM_PLUGIN_PATH=<repo>/build \
  gz sim -s -r worlds/sr75_f22_detachable_launch_test.sdf
):

  python3 tools/f22gzi_release_test.py
"""

import math
import threading
import time

import gz.transport13 as gzt
from gz.msgs10 import pose_v_pb2, entity_wrench_pb2, entity_pb2, empty_pb2

WORLD = "sr75_f22_detachable_launch_test"
UAV_MODEL = "sr75_uav_release"
LAUNCHER_MODEL = "sr75_launcher_carriage"
TOPIC = f"/world/{WORLD}/dynamic_pose/info"
DETACH_TOPIC = "/sr75_launcher_carriage/release/detach"

# Rail axis unit vector solved in F22-GZ-E: elevation=20deg, azimuth=315deg
# (measured clockwise from world +Y/North toward +X/East).
RAIL_AXIS = (-0.66446302, 0.66446302, 0.34202014)
FORCE_N = 5800.0
FX, FY, FZ = (RAIL_AXIS[0] * FORCE_N, RAIL_AXIS[1] * FORCE_N, RAIL_AXIS[2] * FORCE_N)
DETACH_MIN, DETACH_MAX = 1.00, 1.20

node = gzt.Node()
wrench_pub = node.advertise(f"/world/{WORLD}/wrench/persistent", entity_wrench_pb2.EntityWrench)
wrench_clear_pub = node.advertise(f"/world/{WORLD}/wrench/clear", entity_pb2.Entity)
detach_pub = node.advertise(DETACH_TOPIC, empty_pb2.Empty)

state = {
    "x0": None, "y0": None, "z0": None,
    "detached": False,
    "detach_wall_t": None, "detach_along": None, "detach_cross": None,
    "detach_speed": None, "detach_rpy": None,
    "force_start": None,
    "prev": None,
    "rows": [],
    "sample_count": 0,
}
lock = threading.Lock()


def quat_to_rpy(qx, qy, qz, qw):
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2 * (qw * qy - qz * qx)))
    pitch = math.asin(sinp)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def on_pose(msg):
    uav = None
    lch = None
    for p in msg.pose:
        if p.name == UAV_MODEL:
            uav = p
        elif p.name == LAUNCHER_MODEL:
            lch = p
    if uav is None:
        return
    t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
    x, y, z = uav.position.x, uav.position.y, uav.position.z

    with lock:
        state["sample_count"] += 1
        if state["x0"] is None:
            state["x0"], state["y0"], state["z0"] = x, y, z
            return
        if state["force_start"] is None:
            return

        dx, dy, dz = x - state["x0"], y - state["y0"], z - state["z0"]
        ax, ay, az_ = RAIL_AXIS
        along = dx * ax + dy * ay + dz * az_
        cross_vec = (dx - along * ax, dy - along * ay, dz - along * az_)
        cross = math.sqrt(sum(c * c for c in cross_vec))
        roll, pitch, yaw = quat_to_rpy(uav.orientation.x, uav.orientation.y, uav.orientation.z, uav.orientation.w)

        speed = None
        prev = state["prev"]
        if prev is not None:
            pt, px, py, pz = prev
            dt = t - pt
            if dt > 1e-6:
                speed = math.sqrt((x - px) ** 2 + (y - py) ** 2 + (z - pz) ** 2) / dt
        state["prev"] = (t, x, y, z)

        wall_dt = time.time() - state["force_start"]
        sep = None
        if lch is not None:
            sep = math.sqrt((x - lch.position.x) ** 2 + (y - lch.position.y) ** 2 + (z - lch.position.z) ** 2)
        state["rows"].append((wall_dt, t, along, cross, speed, math.degrees(roll), math.degrees(pitch), math.degrees(yaw), sep, state["detached"]))

        if not state["detached"] and DETACH_MIN <= along <= DETACH_MAX:
            detach_pub.publish(empty_pb2.Empty())
            state["detached"] = True
            state["detach_wall_t"] = wall_dt
            state["detach_along"] = along
            state["detach_cross"] = cross
            state["detach_speed"] = speed
            state["detach_rpy"] = (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))
            print(f"DETACH_FIRED wall_dt={wall_dt:.4f} along={along:.4f} speed={speed} sample_count={state['sample_count']}")


def main():
    node.subscribe(pose_v_pb2.Pose_V, TOPIC, on_pose)

    time.sleep(1.0)
    print("subscriber warmed up, sample_count=", state["sample_count"])

    wrench = entity_wrench_pb2.EntityWrench()
    wrench.entity.name = "wing"
    wrench.entity.type = entity_pb2.Entity.LINK
    wrench.wrench.force.x = FX
    wrench.wrench.force.y = FY
    wrench.wrench.force.z = FZ

    with lock:
        state["force_start"] = time.time()
    wrench_pub.publish(wrench)
    print("FORCE_APPLIED (world-frame vector along rail axis, 5800N) at", state["force_start"])

    time.sleep(3.6)

    clear_entity = entity_pb2.Entity()
    clear_entity.name = "wing"
    clear_entity.type = entity_pb2.Entity.LINK
    wrench_clear_pub.publish(clear_entity)
    print("FORCE_CLEARED")

    with lock:
        rows = list(state["rows"])
        detach_wall_t = state["detach_wall_t"]
        detach_along = state["detach_along"]
        detach_cross = state["detach_cross"]
        detach_speed = state["detach_speed"]
        detach_rpy = state["detach_rpy"]
        total_samples = state["sample_count"]

    print("total pose samples received:", total_samples)
    print(f"{'t_wall':>8} {'sim_t':>10} {'along':>8} {'cross':>8} {'speed':>8} {'roll':>8} {'pitch':>8} {'yaw':>8} {'sep':>8} {'det':>4}")
    for r in rows[::max(1, len(rows) // 60)]:
        speed_str = f"{r[4]:.2f}" if r[4] is not None else "  n/a"
        sep_str = f"{r[8]:.4f}" if r[8] is not None else "   n/a"
        print(f"{r[0]:8.4f} {r[1]:10.3f} {r[2]:8.3f} {r[3]:8.4f} {speed_str:>8} {r[5]:8.2f} {r[6]:8.2f} {r[7]:8.2f} {sep_str:>8} {int(r[9]):4d}")

    print("DETACH_T", detach_wall_t, "DETACH_ALONG", detach_along, "DETACH_CROSS", detach_cross,
          "DETACH_SPEED", detach_speed, "DETACH_RPY", detach_rpy)


if __name__ == "__main__":
    main()
