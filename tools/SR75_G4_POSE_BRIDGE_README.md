# SR-75 G4 MAVLink to Gazebo Pose Bridge

This phase keeps Gazebo visual-only. JSBSim remains the flight dynamics owner,
and ArduPilot SITL remains the autopilot, mission, and RATO state controller.
The bridge reads MAVLink `ATTITUDE` and `LOCAL_POSITION_NED`, converts local NED
position to Gazebo ENU, and sends pose updates to the Gazebo set_pose service.

## Terminal 1: start Gazebo server-only

```bash
cd /mnt/e/ardupilot_dev/ardupilot_gazebo
export GZ_SIM_RESOURCE_PATH=$PWD/models:$GZ_SIM_RESOURCE_PATH
gz sim -r -s worlds/sr75_visual_follower_test.sdf
```

## Terminal 2: start ArduPilot/JSBSim SITL

```bash
cd ~/ardupilot_clean
./Tools/autotest/sim_vehicle.py -v ArduPlane \
-f jsbsim:sr_75_6_dof \
--console \
--map \
-l 32.5378885,74.3661944,240.2,0 \
-w \
--add-param-file Tools/autotest/models/sr75.parm \
--out=udp:127.0.0.1:14560
```

## Terminal 3: run bridge

```bash
cd /mnt/e/ardupilot_dev/ardupilot_gazebo
python3 tools/sr75_mavlink_gazebo_pose_bridge.py \
--mavlink udpin:0.0.0.0:14560 \
--world sr75_visual_follower_test \
--model sr75_uav_visual_follower \
--rate 2 \
--position-scale 0.05 \
--invert-north
```

## Dry run

```bash
python3 tools/sr75_mavlink_gazebo_pose_bridge.py \
--mavlink udpin:0.0.0.0:14560 \
--dry-run
```

## Long-flight visualization

```bash
python3 tools/sr75_mavlink_gazebo_pose_bridge.py \
--mavlink udpin:0.0.0.0:14560 \
--world sr75_visual_follower_test \
--model sr75_uav_visual_follower \
--rate 2 \
--timeout 5000 \
--position-scale 0.05 \
--invert-north
```

Use `--position-scale 0.05` or `--position-scale 0.1` only for visualization.
It does not change JSBSim or ArduPilot physics.
Manual testing showed Gazebo `-Y` is forward along the SR75 launcher, so use
`--invert-north` for visual flights in this world.

## Long-flight visualization with CSV log

```bash
mkdir -p logs

python3 tools/sr75_mavlink_gazebo_pose_bridge.py \
--mavlink udpin:0.0.0.0:14560 \
--world sr75_visual_follower_test \
--model sr75_uav_visual_follower \
--rate 2 \
--timeout 5000 \
--position-scale 0.05 \
--invert-north \
--csv-log logs/sr75_g6_visual_pose_$(date +%Y%m%d_%H%M%S).csv
```

## Manual set_pose visual checks

Use the SR75 base alignment quaternion for visual checks:
`orientation: {w: 0.707105, x: 0.707108, y: 0, z: 0}`.

```bash
gz service -s /world/sr75_visual_follower_test/set_pose/blocking \
--reqtype gz.msgs.Pose \
--reptype gz.msgs.Boolean \
--timeout 3000 \
--req 'name: "sr75_uav_visual_follower", position: {x: 0, y: 20, z: 5}, orientation: {w: 0.707105, x: 0.707108, y: 0, z: 0}'
```

Manual `set_pose` with `orientation: {w: 1}` is only for raw pose testing and
will remove the SR75 model's 90-degree visual alignment. Use the base
quaternion for visual alignment.

## Expected result

Gazebo pose service returns `data: true`, and the SR75 UAV visual follower updates
according to ArduPilot/JSBSim motion.

## Warning

If the GUI crashes due to WSL OGRE/libd3d12core rendering, continue with
server-only testing. This is a rendering issue, not a pose bridge failure.
