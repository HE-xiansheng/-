# Validation stages

1. eMMC root/boot, Wi-Fi/SSH and system clock: verified.
2. ROS2 base: install packages, record versions; validate C++ publisher to Python subscriber locally with isolated DDS discovery.
3. Drivers: connect actual devices, measure timestamps, message formats, frame IDs and rates.
4. FAST-LIO: coordinate/IMU conventions, TF, drift and latency; no controller starts.
5. MAVROS/PX4: vision reception and estimator fusion while disarmed and landed.
6. Planner and visual tasks: custom interfaces, QoS, TF, clock, parameters, launch; ground workflows.
7. Groundstation: remote start, commands, feedback, map/video and rosbag2.

Do not arm, take off or run missions automatically. OS, driver and production environment switches follow the user's confirmation requirements. Image full integrity tests/readback were skipped at the user's instruction; functional validation remains separate.
