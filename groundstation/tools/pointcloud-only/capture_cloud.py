#!/usr/bin/env python3
"""验证来自指定机载 IP 的完整 PCD1/LZ4 帧；不发送任何数据。"""
import argparse
import json
from pathlib import Path
import socket
import time
from cloud_sender import HEADER, MAGIC


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='192.168.1.235')
    parser.add_argument('--port', type=int, default=9000)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,4*1024*1024)
    sock.bind(('0.0.0.0',args.port))
    sock.settimeout(2.)
    pending = {}
    frames = []
    datagrams = 0
    deadline = time.monotonic()+15
    try:
        while time.monotonic()<deadline and len(frames)<3:
            try:
                packet,peer = sock.recvfrom(65535)
            except socket.timeout:
                continue
            if peer[0] != args.host or len(packet)<HEADER.size:
                continue
            magic,sequence,index,count,raw_size,compressed_size,length = HEADER.unpack_from(packet)
            assert magic == MAGIC and 0 <= index < count and length == len(packet)-HEADER.size and len(packet)<=1400
            datagrams += 1
            if sequence not in pending:
                pending = {sequence:{'count':count,'raw_size':raw_size,'compressed_size':compressed_size,'parts':{},'first_packet':packet}}
            frame = pending[sequence]
            assert (count,raw_size,compressed_size)==(frame['count'],frame['raw_size'],frame['compressed_size'])
            frame['parts'][index]=packet[HEADER.size:]
            if len(frame['parts']) == count:
                compressed = b''.join(frame['parts'][i] for i in range(count))
                assert len(compressed)==compressed_size and raw_size%12==0
                frames.append({'sequence':sequence,'points':raw_size//12,'packets':count,'source':peer[0]})
                if len(frames)==1:
                    (args.output/'first-packet.bin').write_bytes(frame['first_packet'])
                    (args.output/'frame.lz4').write_bytes(compressed)
                pending.clear()
        assert len(frames)==3,'未收到3个完整机载点云帧'
        result = {'passed':True,'frames':frames,'received_datagrams':datagrams,'port':args.port,'real_board':True}
        (args.output/'capture-result.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result,indent=2))
    finally:
        sock.close()


if __name__ == '__main__':
    main()
