set -e
source /opt/ros/humble/setup.bash
source /home/orangepi/drone_platform/02_drivers/sensors/workspace/install/setup.bash
python3 - <<'META'
from pathlib import Path
import hashlib,json
r=Path('/tmp/fastlio-receive-clock-fix-184504-792c7c')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert sha(r/'source/fast_lio/src/laserMapping.cpp')=='323b7c5cbed2f95c16ec4e650a8639891161ca97bc11c18b43bd86b0dab87961'
assert sha(r/'install/lib/fast_lio/fastlio_mapping')=='17af7094f3191be589c8f04132ee86e46035f6f6fc7b51c7f25ad698c9750a97'
print(json.dumps({'event':'binary_identity','probe_sha256':sha(r/'fastlio_mapping-probe'),'source_restored':True}),flush=True)
META
exec python3 /tmp/fastlio-receive-clock-fix-184504-792c7c/live-probe.py --root /tmp/fastlio-receive-clock-test-191354-5865ac --binary /tmp/fastlio-receive-clock-fix-184504-792c7c/fastlio_mapping-probe --config /tmp/fastlio-receive-clock-fix-184504-792c7c/config/lidar.yaml --baseline-seconds 45 --skip-stall
