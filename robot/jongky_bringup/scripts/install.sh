#!/usr/bin/env bash
# 젯슨에서 한 번 돌린다. ~/bin 에 심링크를 걸고 PATH 를 잡는다.
set -euo pipefail

HERE="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
mkdir -p "$HOME/bin"

for f in jmap jdrive jsave jcheck jstop jview jbot; do
  ln -sf "$HERE/$f" "$HOME/bin/$f"
  echo "  $f"
done

LINE='export PATH="$HOME/bin:$PATH"'
if ! grep -qxF "$LINE" "$HOME/.bashrc" 2>/dev/null; then
  echo "$LINE" >> "$HOME/.bashrc"
  echo "PATH 를 .bashrc 에 추가했다"
fi

# 7인치 화면에서 기본 폰트가 너무 작다. 콘솔 폰트를 키운다.
if [ -w /etc/default/console-setup ] 2>/dev/null; then
  echo "콘솔 폰트를 키우려면:  sudo dpkg-reconfigure console-setup  (Terminus 16x32 권장)"
fi

cat <<'MSG'

설치 완료. 새 터미널을 열거나:

    source ~/.bashrc

쓰는 법:

    jcheck        맵핑 전 점검 (/scan /odom /map TF)
    jmap 10f      SLAM + rosbag 기동
    jdrive 10f    텔레옵 (다른 터미널에서)
    jsave 10f     지도 저장
    jstop         정리

MSG
