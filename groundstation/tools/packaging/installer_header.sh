#!/usr/bin/env bash
# 科研飞行平台-地面站：带完整 Qt 运行时的用户目录安装包。
set -euo pipefail
GCS_INSTALL_TMP="$(mktemp -d "${TMPDIR:-/tmp}/research-flight-install.XXXXXX")"
trap 'rm -rf "$GCS_INSTALL_TMP"' EXIT
GCS_PAYLOAD_LINE="$(awk '/^__GCS_ARCHIVE_BELOW__$/ { print NR + 1; exit }' "$0")"
tail -n +"$GCS_PAYLOAD_LINE" "$0" > "$GCS_INSTALL_TMP/payload.tar.gz"
GCS_ACTUAL_SHA="$(sha256sum "$GCS_INSTALL_TMP/payload.tar.gz")"
if [[ "${GCS_ACTUAL_SHA%% *}" != "@PAYLOAD_SHA256@" ]]; then
    echo '安装包校验失败，请重新复制安装包。' >&2
    exit 1
fi
tar -xzf "$GCS_INSTALL_TMP/payload.tar.gz" -C "$GCS_INSTALL_TMP"
/usr/bin/python3 "$GCS_INSTALL_TMP/research-flight-groundstation/install_user.py" "$@"
exit 0
__GCS_ARCHIVE_BELOW__
