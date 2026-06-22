# SR-75 G7 HUD Viewer

This HUD viewer is visual-only. It reads SR75 SITL MAVLink telemetry and draws
an OpenCV HUD window. It does not modify ArduPilot, JSBSim, Gazebo physics, or
the pose bridge.

## Dependencies

```bash
python3 -m pip install opencv-python pymavlink
```

If Gazebo camera background support is needed later through Python transport
bindings, that would require additional Gazebo Python packages. This G7B
prototype keeps synthetic HUD mode fully working and falls back clearly if
Gazebo image subscription is unavailable.

## Synthetic HUD

```bash
cd /mnt/e/ardupilot_dev/ardupilot_gazebo
python3 tools/sr75_hud_viewer.py \
--mavlink udpin:0.0.0.0:14570 \
--width 1280 \
--height 720 \
--rate 30
```

## Gazebo Camera HUD

```bash
cd /mnt/e/ardupilot_dev/ardupilot_gazebo
python3 tools/sr75_hud_viewer.py \
--mavlink udpin:0.0.0.0:14570 \
--background gazebo \
--gazebo-image-topic /sr75/chase_camera/image \
--width 1280 \
--height 720 \
--rate 30
```

## MAVLink inputs

The HUD listens for:

- `ATTITUDE`
- `VFR_HUD`
- `GLOBAL_POSITION_INT`
- `LOCAL_POSITION_NED`
- `AOA_SSA`
- `BATTERY_STATUS`
- `HEARTBEAT`

## Current HUD content

- Artificial horizon from roll and pitch
- Center crosshair
- Roll and pitch text
- Heading tape and heading readout
- Airspeed
- Altitude
- Climb rate
- Throttle
- Flight mode
- AOA if available
- Battery percentage if available
- Camera source label

## Background modes

- `--background synthetic` keeps the dark synthetic background used in G7A.
- `--background gazebo` requires `--gazebo-image-topic`.
- If Gazebo Python image subscription is unavailable, the viewer prints a clear
  message and falls back to the synthetic background without breaking the HUD.
