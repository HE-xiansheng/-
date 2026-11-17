set -e
source /opt/ros/humble/setup.bash
source /home/orangepi/drone_platform/02_drivers/sensors/workspace/install/setup.bash
tar -xzf /tmp/fastlio-receive-clock-fix-184504-792c7c/candidate-v2-source.tar.gz -C /tmp/fastlio-receive-clock-fix-184504-792c7c
cmake -S /tmp/fastlio-receive-clock-fix-184504-792c7c/source/fast_lio -B /tmp/fastlio-receive-clock-fix-184504-792c7c/build -DCMAKE_INSTALL_PREFIX=/tmp/fastlio-receive-clock-fix-184504-792c7c/install-v2 -DCMAKE_BUILD_TYPE=Release -DFASTLIO_ROOT_DIR=/home/orangepi/drone_platform/01_apps/fast_lio/workspace/src/fast_lio
taskset -c 4-7 nice -n 10 cmake --build /tmp/fastlio-receive-clock-fix-184504-792c7c/build --target fastlio_mapping -j1
cmake --install /tmp/fastlio-receive-clock-fix-184504-792c7c/build
