#!/usr/bin/env bash
set -euo pipefail
GCS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
MINGW_USR="$GCS_ROOT/.deps/windows-cross/sysroot/usr"
export LD_LIBRARY_PATH="$MINGW_USR/lib/x86_64-linux-gnu:$GCS_ROOT/.deps/Qt/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PATH="$GCS_ROOT/.venv-build/bin:$MINGW_USR/bin:$PATH"
cmake -S "$GCS_ROOT/dianyunGCS-master" -B "$GCS_ROOT/build/windows" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE="$GCS_ROOT/.deps/windows-cross/toolchain.cmake" \
  -DCMAKE_PREFIX_PATH="$GCS_ROOT/.deps/windows-qt" \
  -DQT_HOST_PATH="$GCS_ROOT/.deps/Qt" \
  -DCMAKE_EXE_LINKER_FLAGS=-fstack-protector \
  -DGCS_LZ4_INCLUDE_DIR="$GCS_ROOT/.deps/windows-cross/codecs/lz4-1.9.4/lib" \
  -DGCS_LZ4_LIBRARY="$GCS_ROOT/.deps/windows-cross/codecs/lz4-1.9.4/lib/liblz4.a" \
  -DBZIP2_INCLUDE_DIR="$GCS_ROOT/.deps/windows-cross/codecs/bzip2-1.0.8" \
  -DBZIP2_LIBRARY_RELEASE="$GCS_ROOT/.deps/windows-cross/codecs/bzip2-1.0.8/libbz2.a"
cmake --build "$GCS_ROOT/build/windows" --parallel 3
# 旧版GNU PE链接器对Qt入口库的DWARF5节映射不正确；在链接完成后移除调试节。
for binary in customgeometry.exe radarsimulator.exe ssh-askpass.exe; do
  test -s "$GCS_ROOT/build/windows/$binary"
  "$MINGW_USR/bin/x86_64-w64-mingw32-strip" --strip-debug "$GCS_ROOT/build/windows/$binary"
done
