#!/usr/bin/env bash
set -euo pipefail

# patch_cubemx.sh
# Fix CubeMX-generated cmake/stm32cubemx/CMakeLists.txt that wrongly uses ${CMAKE_SOURCE_DIR}
# When CubeMX output is nested inside a larger repo, ${CMAKE_SOURCE_DIR} points to the repo root,
# so paths like ${CMAKE_SOURCE_DIR}/Core/Src/main.c break.
#
# This patch:
#   1) Defines CUBEMX_ROOT as the CubeMX project root (computed from this CMakeLists location)
#   2) Replaces ${CMAKE_SOURCE_DIR}/... with ${CUBEMX_ROOT}/...
#
# Usage:
#   ./patch_cubemx.sh <path-to-cubemx-root>
#
# Example:
#   ./patch_cubemx.sh nucleo_h7a3ziq/cubemx
#   ./patch_cubemx.sh platform/stm32/nucleo_h7a3ziq/

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 <board-root>"
  exit 2
fi

BOARD_ROOT="$(cd "$1" && pwd)"
FILE="$BOARD_ROOT/cmake/stm32cubemx/CMakeLists.txt"

if [[ ! -f "$FILE" ]]; then
  echo "ERROR: Not found: $FILE"
  echo "Expected: <board-root>/cmake/stm32cubemx/CMakeLists.txt"
  exit 1
fi

echo "[PATCH] TARGET = $FILE"
echo "[PATCH] BOARD_ROOT = $BOARD_ROOT"

## Deciding if patch is needed:
need_patch=0

# A) If CubeMX still uses ${CMAKE_SOURCE_DIR}/ anywhere, we must patch.
if grep -q '\${CMAKE_SOURCE_DIR}/' "$FILE"; then
  need_patch=1
fi

# B) If CUBEMX_ROOT line is missing or doesnt match the expected pattern, we must patch.
if ! grep -q 'get_filename_component(CUBEMX_ROOT' "$FILE"; then
  need_patch=1
else
  # Extract the path from the CUBEMX_ROOT line and check if it looks sane
  if ! grep -q 'get_filename_component(CUBEMX_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)' "$FILE"; then
    need_patch=1
  fi
fi

if [[ "$need_patch" -eq 0 ]]; then
  echo "[PATCH] Already OK (no \${CMAKE_SOURCE_DIR}/ and CUBEMX_ROOT looks sane)."
  exit 0
fi

echo "[PATCH] Patching required."

tmp="$(mktemp)"

# 1) Remove ANY existing CUBEMX_ROOT definition line (good/bad/ugly)
grep -vE '^[[:space:]]*get_filename_component[[:space:]]*\([[:space:]]*CUBEMX_ROOT\b' "$FILE" > "$tmp"

# 2) Insert correct definition right after cmake_minimum_required(...)
awk '
  BEGIN { inserted=0 }
  { print }
  (!inserted && $0 ~ /^cmake_minimum_required\(/) {
    print ""
    print "# ---- patch_cubemx.sh (manual patch; rerun after CubeMX regen)"
    print "get_filename_component(CUBEMX_ROOT \"${CMAKE_CURRENT_LIST_DIR}/../..\" ABSOLUTE)"
    print "# ---- end patch"
    inserted=1
  }
' "$tmp" > "$FILE"

rm -f "$tmp"

# 3) Rewrite ${CMAKE_SOURCE_DIR}/... -> ${CUBEMX_ROOT}/...
sed -i 's|${CMAKE_SOURCE_DIR}/|${CUBEMX_ROOT}/|g' "$FILE"

echo "[PATCH] DONE."