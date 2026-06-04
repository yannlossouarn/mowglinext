# Sensors

Dockerized ROS2 drivers for each supported sensor model. Located in `sensors/` in the monorepo.

## Supported Sensors

### GPS: u-blox ZED-F9P

| Property | Value |
|----------|-------|
| Directory | `sensors/gps/` |
| ROS2 Topic | `/ublox_gps_node/fix` (NavSatFix) |
| Connection | USB-CDC |
| RTK Support | Yes (via NTRIP) |
| Config | `sensors/gps/f9p_rover.yaml` |

The GPS container runs:
- `ublox_gps_node` — u-blox driver
- `ntrip_client` — RTK corrections from NTRIP caster
- `rtcm_serial_bridge.py` — forwards RTCM to GPS serial

**RTK Fixed/Float flicker:** under motion an F9P's reported carrier solution
(`carrSoln`) can toggle Fixed↔Float every epoch even while position σ stays
~4 mm — a pure classification flicker, not a position problem. Two pieces of
the ROS2 stack absorb this so it doesn't propagate downstream:
- `localization_monitor_node` debounces the published localization mode
  (`mode_debounce_sec`, default 1.0 s) — see
  [Architecture › localization_monitor_node](Architecture#3c-localization_monitor_node).
- The ublox GNSS diagnostics path treats `corrections_active` as following the
  carrier solution (a Fixed/Float solution implies corrections are active, since
  the receiver can't solve RTK without them), only falling back to the bursty
  transport RTCM freshness metric when the solution is not RTK. (The Unicore
  path is unchanged — it already uses the receiver's authoritative correction
  age.)

### LiDAR: LDRobot LD19

| Property | Value |
|----------|-------|
| Directory | `sensors/lidar/` |
| ROS2 Topic | `/scan` (LaserScan) |
| Connection | UART 230400 |
| Frame ID | `lidar_link` |

## Adding a New Sensor

1. Create `sensors/<type>-<model>/` (e.g., `sensors/lidar-rplidar/`)
2. Add a `Dockerfile` that builds the ROS2 driver
3. Add `ros2_entrypoint.sh` for ROS2 environment setup
4. Ensure the driver publishes the expected topic:
   - LiDAR: `/scan` (LaserScan)
   - GPS: `/ublox_gps_node/fix` or `/gps/fix` (NavSatFix)
5. Update `docker/docker-compose.yaml` to point the service at your new directory

## Building Locally

```bash
docker build -t mowgli-gps sensors/gps/
docker build -t mowgli-lidar --target runtime sensors/lidar/
```

## CI

Images are built automatically by `.github/workflows/sensors-docker.yml` for `linux/amd64` and `linux/arm64` on every push to `sensors/`.
