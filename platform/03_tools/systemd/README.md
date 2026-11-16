# ROS2 sensor service candidate

`Safina_sensor.service` is a reviewed candidate stored with source. Merely copying
this directory does not install, enable, or start a system service.

The entrypoint is `03_tools/config script/start_platform_sensors.sh`. It launches
full MAVROS, Gemini, independent USB camera, Livox and the RTSP image publisher.
It does not start localization, pose relay, task controllers, arming or flight.
The MAVROS command APIs are available; no command client is started here.

Selected-module preflight checks package availability, PX4 serial accessibility,
independent USB identity and Livox host IP. This is not a guarantee that cameras,
radar, RTSP data or PX4 fusion are healthy. Check actual topics and timestamps.
Any unexpected child exit fails the launch and stops the remaining children.
Ordinary SIGINT stops the group. The service intentionally has no restart loop.

With only the currently attached PX4 and Gemini, temporary ground testing uses:

```bash
bash '/home/orangepi/drone_platform/03_tools/config script/start_platform_sensors.sh' \
  enable_usb_camera:=false enable_lidar:=false enable_rtsp:=false
```

The all-enabled service requires an independent USB device binding, a connected
Livox interface with the configured host IP, and a reachable RTSP camera first.
No network settings are changed by the entrypoint. Runtime ROS logs go to
`/run/drone-sensors/ros-log` when run by this candidate service.

Installing/enabling the service or replacing an existing startup entry requires
user confirmation. Existing `start_sensors.sh` and `sensors.launch.py` remain
available. To abandon this candidate before activation, simply stop its owned
foreground launch with Ctrl-C; no system service change needs reverting.
