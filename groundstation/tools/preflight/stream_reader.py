"""有期限的只读SSH行流；只保留当前批次最新包，避免把积压快照重新标成实时。"""
import json,os,selectors,time,zlib,base64,struct


def _packets_from_selector(process,stopped,idle_timeout=6,startup_timeout=15,alive_hint=None):
    selector=selectors.DefaultSelector();selector.register(process.stdout,selectors.EVENT_READ)
    buffer=b'';last=time.monotonic();first=True;offset=None
    try:
        while not stopped.is_set():
            events=selector.select(.25)
            if not events:
                if process.poll() is not None:return
                if time.monotonic()-last>(startup_timeout if first else idle_timeout):
                    if alive_hint and alive_hint():continue
                    raise TimeoutError('只读监视长时间无新采样，正在重新连接')
                continue
            chunk=os.read(process.stdout.fileno(),1024*1024)
            if not chunk:return
            buffer+=chunk
            if len(buffer)>2*1024*1024:raise ValueError('监视包超过长度上限')
            lines=buffer.split(b'\n');buffer=lines.pop();latest=None
            for line in lines:
                try:value=json.loads(line)
                except (ValueError,UnicodeError):continue
                if isinstance(value,dict):latest=value
            if latest is None:continue
            last=time.monotonic();first=False
            remote=latest.get('server_monotonic')
            if isinstance(remote,(int,float)):
                delta=last-remote;offset=delta if offset is None else min(offset,delta)
                latest['_transport_delay_s']=max(0.,delta-offset)
            yield latest
    finally:selector.close()


def decode_grid(grid):
    count=grid.get('count')
    if not isinstance(count,int) or not 0<=count<=18000:raise ValueError('栅格显示点数越界')
    value=base64.b64decode(grid['xyz'],validate=True);kind=grid.get('codec')
    if kind in ('zlib-f32','zlib-i16-half-grid','zlib-i16-delta-half-grid'):
        limit=18000*(6 if kind in ('zlib-i16-half-grid','zlib-i16-delta-half-grid') else 12)
        codec=zlib.decompressobj();raw=codec.decompress(value,limit+1)
        if not codec.eof or codec.unused_data or codec.unconsumed_tail:raise ValueError('栅格压缩长度错误')
    elif kind in (None,'raw-f32'):raw=value
    else:raise ValueError('未知栅格显示格式')
    if kind in ('zlib-i16-half-grid','zlib-i16-delta-half-grid'):
        import math,numpy as np
        res=grid.get('resolution')
        if len(raw)!=count*6 or not isinstance(res,(int,float)) or not math.isfinite(res) or res<=0:raise ValueError('栅格数据或分辨率错误')
        points=np.frombuffer(raw,dtype='<i2').reshape(count,3)
        if kind=='zlib-i16-delta-half-grid':points=np.cumsum(points,axis=0,dtype=np.int32)
        return (points.astype('<f4')*(res/2.)).astype('<f4').tobytes()
    if len(raw)!=count*12:raise ValueError('栅格显示帧长度错误')
    return raw


class StatusAssembler:
    """单向状态分片；重排可重组，缺片/超时/错误长度不产生状态。"""
    def __init__(self,magic=b'TLM1'):self.pending={};self.magic=magic
    def accept(self,raw):
        if not 16<=len(raw)<=1216:return None
        magic,seq,index,count,size=struct.unpack_from('<4sIHHI',raw)
        if magic!=self.magic or not 1<=count<=256 or index>=count or not 0<size<=512*1024:return None
        now=time.monotonic()
        self.pending={k:v for k,v in self.pending.items() if now-v[0]<1.5}
        if seq not in self.pending:
            if len(self.pending)>=8:self.pending.clear()
            self.pending[seq]=(now,count,size,{})
        _,n,length,parts=self.pending[seq]
        if n!=count or length!=size:return None
        parts[index]=raw[16:]
        if len(parts)!=count:return None
        del self.pending[seq]
        try:
            codec=zlib.decompressobj();data=codec.decompress(b''.join(parts[k] for k in range(count)),size+1)
            if len(data)!=size or not codec.eof or codec.unused_data or codec.unconsumed_tail:return None
            value=json.loads(data)
            if isinstance(value,dict):return value
        except (ValueError,zlib.error):pass
        return None


def packets_from_pipe(process,stopped,idle_timeout=6,startup_timeout=15,alive_hint=None):
    if os.name != 'nt':
        yield from _packets_from_selector(process,stopped,idle_timeout,startup_timeout,alive_hint)
        return
    # Windows select 仅支持 socket：专用读线程处理匿名管道，仍保留期限和最新包语义。
    import queue,threading
    chunks=queue.Queue(maxsize=32)
    def read():
        try:
            while not stopped.is_set():
                raw=os.read(process.stdout.fileno(),65536)
                while not stopped.is_set():
                    try:chunks.put(raw,timeout=.25);break
                    except queue.Full:pass
                if not raw:return
        except OSError:pass
        finally:
            try:chunks.put_nowait(b'')
            except queue.Full:pass
    threading.Thread(target=read,daemon=True).start()
    buffer=b'';last=time.monotonic();first=True;offset=None
    while not stopped.is_set():
        try:chunk=chunks.get(timeout=.25)
        except queue.Empty:
            if process.poll() is not None:return
            if time.monotonic()-last>(startup_timeout if first else idle_timeout):
                if alive_hint and alive_hint():continue
                raise TimeoutError('只读监视长时间无新采样，正在重新连接')
            continue
        if not chunk:return
        buffer+=chunk
        if len(buffer)>2*1024*1024:raise ValueError('监视包超过长度上限')
        lines=buffer.split(b'\n');buffer=lines.pop();latest=None
        for line in lines:
            try:value=json.loads(line)
            except (ValueError,UnicodeError):continue
            if isinstance(value,dict):latest=value
        if latest is None:continue
        last=time.monotonic();first=False
        remote=latest.get('server_monotonic')
        if isinstance(remote,(int,float)):
            delta=last-remote;offset=delta if offset is None else min(offset,delta)
            latest['_transport_delay_s']=max(0.,delta-offset)
        yield latest
