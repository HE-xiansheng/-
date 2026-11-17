# 原始机载档案分片

原始档案超过GitHub单文件限制，按40MiB分片，按文件名顺序拼接即可恢复。原文件及分片大小和SHA256见manifest.json；电脑保留完整原档和展开数据库。

在本目录执行：

```bash
cat evidence.tar.gz.part-* > ../evidence.tar.gz
python3 - <<'CHECK'
import hashlib,json
from pathlib import Path
m=json.loads(Path('manifest.json').read_text())
p=Path('../evidence.tar.gz')
h=hashlib.sha256()
with p.open('rb') as source:
    for block in iter(lambda: source.read(1024*1024), b''):
        h.update(block)
assert p.stat().st_size == m['original_size'] and h.hexdigest() == m['sha256']
CHECK
mkdir -p ../reassembled-evidence
tar -xzf ../evidence.tar.gz -C ../reassembled-evidence
```

原始结果为失败；不得修改或以诊断参数覆盖冒充正式验收通过。
