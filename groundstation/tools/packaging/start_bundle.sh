#!/usr/bin/env bash
set -euo pipefail
GCS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
unset QT_PLUGIN_PATH QT_QPA_PLATFORM_PLUGIN_PATH QML2_IMPORT_PATH QML_IMPORT_PATH
export QTDIR="$GCS_ROOT/.deps/Qt"
export LD_LIBRARY_PATH="$QTDIR/lib:$GCS_ROOT/.deps/runtime/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export QML_IMPORT_PATH="$QTDIR/qml"
export QSG_RHI_BACKEND="${QSG_RHI_BACKEND:-opengl}"
export PATH="$GCS_ROOT/dianyunGCS-master/tools/desktop/video-bin:$PATH"
cd "$GCS_ROOT/dianyunGCS-master"
if [[ "${1:-}" == "--live" ]]; then
    shift
    GCS_CLOUD_PORT="${1:-9000}"
    if [[ $# -gt 0 ]]; then shift; fi
    explicit_host=false
    for argument in "$@"; do
        case "$argument" in
            --host|--host=*) explicit_host=true; break ;;
        esac
    done
    connection="$GCS_ROOT/.pointcloud-link/connection.json"
    if [[ "$explicit_host" == false && -r "$connection" ]]; then
        # The compiled video controller initializes from --host at startup.
        saved_host="$(/usr/bin/python3 -c '
import json, sys
with open(sys.argv[1], encoding="utf-8") as config:
    host = json.load(config).get("host")
if isinstance(host, str):
    print(host.strip())
' "$connection" 2>/dev/null)" || saved_host=
        if [[ -n "$saved_host" && "$saved_host" != -* && "$saved_host" != *[[:space:]]* ]]; then
            set -- "$@" --host "$saved_host"
        fi
    fi
    exec "$GCS_ROOT/build/linux/customgeometry" --cloud-port "$GCS_CLOUD_PORT" "$@"
fi
exec "$GCS_ROOT/build/linux/customgeometry" "$@"
