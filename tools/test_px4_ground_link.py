#!/usr/bin/env python3
"""Check the physical wire policy without opening serial/network devices."""
import json
import math
import os
from pathlib import Path
import pty
import select
import socket
import sys
import termios
import time
from px4_ground_link import GroundLink, allowed_message, load_dialect

m = load_dialect(Path(sys.argv[1]))
now = 1700000000000000
checks = {}
def check(name, value):
    checks[name] = bool(value)
    assert value, name

v = m.MAVLink_vision_position_estimate_message(now-100000, 1, 2, 3, .1, .2, .3)
check('fresh_finite_pose_allowed', allowed_message(v, now))
v.usec = now-500001
check('stale_pose_refused', not allowed_message(v, now))
v.usec = now+100001
check('future_pose_refused', not allowed_message(v, now))
v.usec = now
v.x = math.nan
check('nonfinite_pose_refused', not allowed_message(v, now))
for command in (176, 400, 22, 21, 20, 179, 511, 246):
    msg = m.MAVLink_command_long_message(1, 1, command, 0, 1, 0, 0, 0, 0, 0, 0)
    check('nonquery_command_'+str(command)+'_refused', not allowed_message(msg, now))
check('mode_refused', not allowed_message(m.MAVLink_set_mode_message(1, 1, 6), now))
check('parameter_write_refused', not allowed_message(m.MAVLink_param_set_message(1, 1, b'EKF2_EV_CTRL', 3, 6), now))
check('parameter_query_allowed', allowed_message(m.MAVLink_param_request_read_message(1, 1, b'EKF2_EV_CTRL', -1), now))
check('firmware_query_allowed', allowed_message(m.MAVLink_command_long_message(1, 1, 512, 0, 148, 0, 0, 0, 0, 0, 0), now))
check('query_extra_arguments_refused', not allowed_message(m.MAVLink_command_long_message(1, 1, 512, 0, 148, 1, 0, 0, 0, 0, 0), now))
check('mavros_shell_refused', not allowed_message(m.MAVLink_serial_control_message(10, 6, 0, 0, 1, [10]+[0]*69), now))
check('setpoint_refused', not allowed_message(m.MAVLink_set_position_target_local_ned_message(0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0), now))
check('clock_sync_allowed', allowed_message(m.MAVLink_timesync_message(0, 123), now))
check('companion_active_heartbeat_is_not_arm_command', allowed_message(m.MAVLink_heartbeat_message(18, 8, 192, 0, 4, 3), now))
check('FCU_heartbeat_spoof_refused_from_ROS', not allowed_message(m.MAVLink_heartbeat_message(2, 12, 128, 0, 4, 3), now))
check('mission_read_list_allowed', allowed_message(m.MAVLink_mission_request_list_message(1, 1), now))
check('mission_clear_refused', not allowed_message(m.MAVLink_mission_clear_all_message(1, 1), now))
check('mission_upload_count_refused', not allowed_message(m.MAVLink_mission_count_message(1, 1, 3), now))
check('broadcast_firmware_query_reply_to_requester_allowed', allowed_message(m.MAVLink_command_long_message(0, 0, 512, 0, 148, 0, 0, 0, 0, 0, 1), now))
check('request_message_invalid_reply_selector_refused', not allowed_message(m.MAVLink_command_long_message(0, 0, 512, 0, 148, 0, 0, 0, 0, 0, 3), now))
check('home_query_allowed', allowed_message(m.MAVLink_command_long_message(1, 1, 410, 0, 0, 0, 0, 0, 0, 0, 0), now))
master, slave = pty.openpty()
device = os.ttyname(slave)
original = termios.tcgetattr(slave)
peer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
peer.bind(('127.0.0.1', 14650))
link = None
try:
    link = GroundLink(device, Path(sys.argv[1]))
    physical = m.MAVLink(None, srcSystem=1, srcComponent=1)
    ros = m.MAVLink(None, srcSystem=1, srcComponent=191)
    os.write(master, b'boot diagnostic\r\n'+m.MAVLink_heartbeat_message(2, 12, 0, 0, 3, 3).pack(physical))
    time.sleep(.01)
    link.step()
    check('PTY_real_heartbeat_forwarded', bool(select.select([peer], [], [], 1)[0]) and bool(peer.recv(65536)))
    check('PTY_disarmed_latch', link.disarmed())
    check('serial_noise_resync_without_forwarding', bool(link.serial_noise) and
          all(row['mavpackettype'] != 'BAD_DATA' for row in link.incoming))
    rejected = False
    try:
        link.send_query('vehicle_visual_odometry; reboot')
    except RuntimeError:
        rejected = True
    check('shell_injection_refused', rejected and not select.select([master], [], [], .02)[0])
    link.send_query('vehicle_visual_odometry')
    packet = os.read(master, 4096)
    decoded = m.MAVLink(None).parse_buffer(packet)[0]
    check('shell_query_exact_listener_only', decoded.get_type() == 'SERIAL_CONTROL' and
          bytes(decoded.data[:decoded.count]) == b'listener vehicle_visual_odometry 1\n')
    peer.sendto(m.MAVLink_set_mode_message(1, 1, 6).pack(ros), ('127.0.0.1', 14651))
    time.sleep(.01)
    rejected = False
    try:
        link.step()
    except RuntimeError:
        rejected = True
    check('wire_mode_packet_blocked_before_serial', rejected and not select.select([master], [], [], .02)[0])
    os.write(master, m.MAVLink_heartbeat_message(2, 12, 128, 0, 3, 3).pack(physical))
    time.sleep(.01)
    rejected = False
    try:
        link.step()
    except RuntimeError:
        rejected = True
    check('armed_physical_heartbeat_aborts', rejected and not link.disarmed())
finally:
    if link:
        link.close()
    peer.close()
    check('original_terminal_settings_restored', termios.tcgetattr(slave) == original)
    reopened = os.open(device, os.O_RDWR | os.O_NOCTTY)
    os.close(reopened)
    check('serial_exclusive_lock_released', True)
    os.close(master)
    os.close(slave)
print(json.dumps({'checks': checks, 'passed': all(checks.values()), 'scope': 'local policy and PTY/localhost, no physical FCU'}, indent=2))
