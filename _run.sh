#!/bin/bash
# dialout 그룹이 현재 셸에 아직 적용되지 않았으면 sg로 감싸서 재실행한다.
if ! id -nG | tr ' ' '\n' | grep -qx dialout; then
  exec sg dialout -c "bash $(printf '%q ' "$0" "$@")"
fi
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$DIR/env.sh"
