set -e
source /opt/ros/humble/setup.bash
source /home/orangepi/drone_platform/02_drivers/sensors/workspace/install/setup.bash
python3 - <<'IDENTITY'
from pathlib import Path
import hashlib,json,subprocess
r=Path('/tmp/fastlio-receive-clock-fix-184504-792c7c')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
p=r/'install-v2/lib/fast_lio/fastlio_mapping'
ldd=subprocess.run(['ldd',str(p)],text=True,capture_output=True)
assert ldd.returncode==0 and 'not found' not in ldd.stdout
assert not any('/tmp/' in line for line in ldd.stdout.splitlines())
metadata={'event':'candidate_v2_identity','binary_sha256':sha(p),'source_sha256':sha(r/'source/fast_lio/src/laserMapping.cpp'),'ldd':ldd.stdout,'production_candidate_v1_sha256':sha(r/'install/lib/fast_lio/fastlio_mapping')}
print(json.dumps(metadata),flush=True)

IDENTITY
exec python3 /tmp/fastlio-receive-clock-fix-184504-792c7c/live-run5.py --root /tmp/fastlio-receive-clock-test-191807-52d782 --binary /tmp/fastlio-receive-clock-fix-184504-792c7c/install-v2/lib/fast_lio/fastlio_mapping --config /tmp/fastlio-receive-clock-fix-184504-792c7c/config/lidar.yaml --candidate-rmw rmw_fastrtps_cpp
