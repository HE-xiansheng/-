set -e
source /opt/ros/humble/setup.bash
source /home/orangepi/drone_platform/02_drivers/sensors/workspace/install/setup.bash
exec python3 /tmp/fastlio-receive-clock-fix-184504-792c7c/live-run8.py --root /tmp/fastlio-receive-clock-test-194622-1e14f8 --binary /tmp/fastlio-receive-clock-fix-184504-792c7c/install-v3/lib/fast_lio/fastlio_mapping --config /tmp/fastlio-receive-clock-fix-184504-792c7c/config/lidar.yaml --candidate-rmw rmw_fastrtps_cpp --candidate-fastdds-profile /tmp/fastlio-receive-clock-fix-184504-792c7c/config/fastdds_localization.xml
