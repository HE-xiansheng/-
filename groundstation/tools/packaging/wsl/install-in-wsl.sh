#!/usr/bin/env bash
# 在WSL root下安装依赖，再以普通用户安装和运行地面站。
set -euo pipefail
GCS_PAYLOAD='' GCS_SHA='' GCS_BINARY_SHA='' GCS_VERSION='' GCS_USER='' GCS_NEW_DISTRO=0 GCS_RESULT='' GCS_MANIFEST_SHA='' GCS_UPGRADE_HELPER=''
while [[ $# -gt 0 ]]; do
    [[ $# -ge 2 ]] || { echo "参数缺少值：$1" >&2; exit 2; }
    case "$1" in
        --payload) GCS_PAYLOAD="$2";;
        --sha256) GCS_SHA="$2";;
        --binary-sha256) GCS_BINARY_SHA="$2";;
        --version) GCS_VERSION="$2";;
        --user) GCS_USER="$2";;
        --new-distro) GCS_NEW_DISTRO="$2";;
        --result-file) GCS_RESULT="$2";;
        --manifest-sha256) GCS_MANIFEST_SHA="$2";;
        --upgrade-helper) GCS_UPGRADE_HELPER="$2";;
        *) echo "未知参数：$1" >&2; exit 2;;
    esac
    shift 2
done
[[ "$GCS_SHA" =~ ^[a-f0-9]{64}$ && "$GCS_BINARY_SHA" =~ ^[a-f0-9]{64}$ && "$GCS_VERSION" =~ ^[a-zA-Z0-9._-]+$ ]] || { echo '安装清单参数无效' >&2; exit 2; }
[[ "$GCS_USER" =~ ^[a-z_][a-z0-9_-]*\$?$ && "$GCS_USER" != root && "$GCS_NEW_DISTRO" =~ ^[01]$ ]] || { echo '目标普通用户无效' >&2; exit 2; }
[[ -f "$GCS_PAYLOAD" && -n "$GCS_RESULT" ]] || { echo '缺少地面站安装包或结果路径' >&2; exit 2; }
[[ "$(uname -m)" == x86_64 && "$(id -u)" == 0 ]] || { echo '此步骤需要x64 WSL中的root用户' >&2; exit 2; }
[[ "$(sha256sum "$GCS_PAYLOAD" | cut -d' ' -f1)" == "$GCS_SHA" ]] || { echo 'Linux安装包校验失败' >&2; exit 2; }
# 仅支持本脚本选定的Ubuntu20.04，不修改其他发行版。
. /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 20.04 ]] || { echo '此一键包需要Ubuntu20.04发行版' >&2; exit 2; }
[[ -z "$GCS_MANIFEST_SHA" || "$GCS_MANIFEST_SHA" =~ ^[a-f0-9]{64}$ ]] || { echo '运行清单摘要无效' >&2; exit 2; }
[[ -z "$GCS_UPGRADE_HELPER" || -f "$GCS_UPGRADE_HELPER" ]] || { echo '升级助手缺失，请完整解压安装包' >&2; exit 2; }
export DEBIAN_FRONTEND=noninteractive
GCS_PACKAGES=(python3 python3-numpy openssh-client util-linux xdg-utils ca-certificates
    gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good
    gstreamer1.0-plugins-bad gstreamer1.0-libav fonts-noto-cjk
    libgl1 libegl1 libopengl0 libfontconfig1 libdbus-1-3 libxcb-cursor0
    libxkbcommon-x11-0 libxcb-icccm4 libxcb-image0 libxcb-keysyms1
    libxcb-render-util0 libxcb-xinerama0)
GCS_MISSING=()
for package in "${GCS_PACKAGES[@]}"; do
    if [[ "$(dpkg-query -W -f='${Status}' "$package" 2>/dev/null || true)" != 'install ok installed' ]]; then
        GCS_MISSING+=("$package")
    fi
done
if [[ ${#GCS_MISSING[@]} -gt 0 ]]; then
    echo '正在安装Ubuntu图形、视频和通信依赖；复用现有下载缓存，失败文件最多重试5次……'
    # 选项仅作用于本次命令，不改用户APT配置，不清理已下载的deb。
    GCS_APT_OPTIONS=(-o Acquire::Retries=5 -o Acquire::http::Timeout=30 -o Acquire::https::Timeout=30)
    apt-get "${GCS_APT_OPTIONS[@]}" update
    if apt-get "${GCS_APT_OPTIONS[@]}" install -y "${GCS_MISSING[@]}"; then
        echo 'Ubuntu依赖安装完成。'
    else
        GCS_APT_STATUS=$?
        echo "Ubuntu依赖安装失败（APT退出码$GCS_APT_STATUS），已下载文件仍保留在/var/cache/apt/archives。" >&2
        echo '若上方为Connection failed/超时，恢复网络后再次双击一键安装.cmd继续，无需清理缓存或卸载WSL。' >&2
        exit "$GCS_APT_STATUS"
    fi
fi
if ! id "$GCS_USER" >/dev/null 2>&1; then
    [[ "$GCS_USER" == gcs ]] || { echo '当前默认Linux用户不存在，未创建其他同名账号' >&2; exit 2; }
    useradd --create-home --shell /bin/bash --comment 'Research Flight Ground Station' "$GCS_USER"
    echo '已创建普通用户gcs用于地面站；没有配置密码或免密sudo。'
fi
[[ "$(id -u "$GCS_USER")" != 0 ]] || { echo '拒绝以root身份安装图形地面站' >&2; exit 2; }
GCS_USER_HOME="$(getent passwd "$GCS_USER" | cut -d: -f6)"
[[ "$GCS_USER_HOME" == /* && -d "$GCS_USER_HOME" && "$GCS_USER_HOME" != / ]] || { echo 'Linux用户主目录无效' >&2; exit 2; }
GCS_GROUP="$(id -gn "$GCS_USER")"
GCS_CACHE="$GCS_USER_HOME/.cache/research-flight-wsl-setup"
install -d -m 700 -o "$GCS_USER" -g "$GCS_GROUP" "$GCS_CACHE"
# 从Windows分区复制到Linux文件系统，避免权限及符号链接语义差异。
install -m 600 -o "$GCS_USER" -g "$GCS_GROUP" "$GCS_PAYLOAD" "$GCS_CACHE/groundstation.run"

matches_install() {
    python3 - "$1" "$GCS_VERSION" "$GCS_BINARY_SHA" "$GCS_MANIFEST_SHA" <<'PY'
import hashlib,json,sys,os
from pathlib import Path
p=Path(sys.argv[1])
try:
    receipt=json.loads((p/'.installation.json').read_text())
    raw=(p/'payload-manifest.json').read_bytes()
    manifest=json.loads(raw)
    binary=p/'build/linux/customgeometry'
    okay=(receipt.get('product')=='research-flight-groundstation' and manifest.get('version')==sys.argv[2]
          and hashlib.sha256(binary.read_bytes()).hexdigest()==sys.argv[3]
          and (not sys.argv[4] or hashlib.sha256(raw).hexdigest()==sys.argv[4]))
    # Qt主程序可能未变，通信脚本仍必须逐一核对，不能只按主程序判断复用。
    roots=('dianyunGCS-master/tools/preflight/','dianyunGCS-master/tools/pointcloud-only/',
           'dianyunGCS-master/tools/communication/','dianyunGCS-master/tools/desktop/')
    for name,digest in manifest['files'].items():
        if name.startswith(roots) or name=='start-groundstation.sh':
            if hashlib.sha256((p/name).read_bytes()).hexdigest()!=digest:okay=False
            if (name.endswith(('/gcs-askpass','/video-bin/gst-launch-1.0')) or name=='start-groundstation.sh') and not os.access(p/name,os.X_OK):okay=False
except (OSError,ValueError,KeyError,TypeError):okay=False
raise SystemExit(0 if okay else 1)
PY
}
GCS_PREFIX="$GCS_USER_HOME/.local/opt/research-flight-groundstation"
if [[ -e "$GCS_PREFIX" ]] && ! matches_install "$GCS_PREFIX"; then
    # 不覆盖已有版本及其日志；新版本在独立目录安装。
    GCS_PREFIX="$GCS_USER_HOME/.local/opt/research-flight-groundstation-wsl-$GCS_VERSION"
fi
if [[ -n "$GCS_UPGRADE_HELPER" ]]; then
    python3 "$GCS_UPGRADE_HELPER" --home "$GCS_USER_HOME" --user "$GCS_USER" --target "$GCS_PREFIX"
fi
if matches_install "$GCS_PREFIX"; then
    echo '同版本地面站及通信助手校验一致，复用现有安装。'
else
    [[ ! -e "$GCS_PREFIX" ]] || { echo "安装位置存在其他内容，未覆盖：$GCS_PREFIX" >&2; exit 2; }
    runuser -u "$GCS_USER" -- env HOME="$GCS_USER_HOME" bash "$GCS_CACHE/groundstation.run" --no-desktop --prefix "$GCS_PREFIX"
fi
matches_install "$GCS_PREFIX" || { echo '安装后的主程序校验失败' >&2; exit 2; }
if [[ "$GCS_NEW_DISTRO" == 1 ]]; then
    # 仅为本工具新建的发行版设置默认普通用户，保留其他wsl.conf配置。
    python3 - "$GCS_USER" <<'PY'
from pathlib import Path
import re,sys,shutil,datetime
p=Path('/etc/wsl.conf');old=p.read_text() if p.exists() else ''
lines=old.splitlines();out=[];inside=False;found=False;written=False
for line in lines:
    m=re.match(r'^\s*\[([^\]]+)\]',line)
    if m:
        if inside and not written:out.append('default='+sys.argv[1]);written=True
        inside=m.group(1).strip().lower()=='user';found=found or inside
    if inside and re.match(r'^\s*default\s*=',line):
        if not written:out.append('default='+sys.argv[1]);written=True
        continue
    out.append(line)
if not found:out+=['','[user]','default='+sys.argv[1]]
elif not written:out.append('default='+sys.argv[1])
new='\n'.join(out).strip()+'\n'
if old!=new:
    if p.exists():shutil.copy2(p,str(p)+'.gcs-backup-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
    p.write_text(new)
PY
fi
python3 - "$GCS_RESULT" "$GCS_USER" "$GCS_PREFIX" "$GCS_VERSION" <<'PY'
from pathlib import Path
import json,sys
p=Path(sys.argv[1]);p.write_text(json.dumps({'success':True,'user':sys.argv[2],'prefix':sys.argv[3],'version':sys.argv[4]},ensure_ascii=False),encoding='utf-8')
PY
echo "地面站安装完成：$GCS_PREFIX"
