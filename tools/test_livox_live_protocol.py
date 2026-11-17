#!/usr/bin/env python3
"""No ROS/hardware: reject damaged and unbounded live transport frames."""
import io
import json
import livox_live_protocol as protocol
checks={}
def rejects(name,data):
    try:protocol.read_frame(io.BytesIO(data))
    except (ValueError,protocol.zlib.error):checks[name]=True
    else:checks[name]=False
buf=io.BytesIO();payload=bytes(range(256))*10
protocol.write_frame(buf,2,payload,123456789)
checks['payload_and_receipt_roundtrip']=protocol.read_frame(io.BytesIO(buf.getvalue()))==(2,payload,123456789)
checks['clean_EOF']=protocol.read_frame(io.BytesIO(b'')) is None
rejects('truncated_header',buf.getvalue()[:8])
rejects('truncated_body',buf.getvalue()[:-1])
bad=bytearray(buf.getvalue());bad[8]^=1;rejects('CRC_mismatch',bad)
bad=bytearray(buf.getvalue());bad[0]=40;rejects('unknown_message_type',bad)
rejects('oversize_encoded_frame',protocol.HEADER.pack(1,protocol.MAX_BYTES+1,0,0))
prior=protocol.MAX_BYTES;protocol.MAX_BYTES=32
raw=b'x'*1000;encoded=protocol.zlib.compress(raw)
rejects('decompression_limit',protocol.HEADER.pack(1,len(encoded),protocol.zlib.crc32(raw),0)+encoded)
protocol.MAX_BYTES=prior
print(json.dumps({'checks':checks,'passed':all(checks.values()),'scope':'local transport only; no ROS or physical hardware'},indent=2))
assert all(checks.values())
