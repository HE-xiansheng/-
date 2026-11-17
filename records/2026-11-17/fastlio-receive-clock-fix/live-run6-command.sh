set -e
source /opt/ros/humble/setup.bash
source /home/orangepi/drone_platform/02_drivers/sensors/workspace/install/setup.bash
exec python3 /tmp/fastlio-receive-clock-fix-184504-792c7c/live-run6.py --root /tmp/fastlio-receive-clock-test-192539-5ae370 --binary /tmp/fastlio-receive-clock-fix-184504-792c7c/install-v2/lib/fast_lio/fastlio_mapping --config /tmp/fastlio-receive-clock-fix-184504-792c7c/config/lidar.yaml --candidate-rmw rmw_cyclonedds_cpp
