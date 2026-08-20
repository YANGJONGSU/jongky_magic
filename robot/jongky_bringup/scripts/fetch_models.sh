#!/usr/bin/env bash
# 인터넷이 되는 자리에서 미리 돌린다. 현장에서는 못 받는다.
#
# [왜 이게 필요한가]
# follow_service.py 의 탐지기는 torchvision 가중치를 **첫 실행 때 인터넷에서
# 받는다** (`M(weights=W.COCO_V1)` → download.pytorch.org). 저장소에도 없고
# 컨테이너 이미지에도 없다 — follow_service.py 는 컨테이너가 아니라 젯슨
# 호스트에서 돌기 때문에 이미지에 굽는다고 해결되지도 않는다.
#
# 건물 WiFi 는 층마다 서브넷이 갈려 있고 11층에서는 밖으로 못 나간다.
# 그 자리에서 처음 띄우면 다운로드가 타임아웃으로 실패하고 서비스가 아예
# 안 뜬다. 개발실에서 한 번 돌려 두면 그 뒤로는 캐시에서 로드한다.
#
# [반드시 follow_service.py 를 돌릴 그 계정으로 실행할 것]
# 캐시가 ~/.cache/torch 라 계정이 다르면 받아 놓고도 못 찾는다.
# sudo 로 돌리면 /root 밑에 받힌다 — 그게 대표적인 실패다.
#
#   ./fetch_models.sh           받는다 (이미 있으면 건너뛴다)
#   ./fetch_models.sh --check   받지 않고 있는지만 본다 (현장 나가기 전 점검)
set -euo pipefail

# torchvision SSDLite320 MobileNetV3-Large, COCO_V1.
# 파일명 뒤 8자리는 torch.hub 가 쓰는 sha256 앞자리다. 받은 파일이 온전한지
# 여기서 같은 방식으로 확인한다 — 반쯤 받힌 .pth 는 로드할 때 알 수 없는
# 역직렬화 에러로 터져서 원인을 찾기 어렵다.
WEIGHTS_FILE="ssdlite320_mobilenet_v3_large_coco-a79551df.pth"
WEIGHTS_URL="https://download.pytorch.org/models/${WEIGHTS_FILE}"
WEIGHTS_SHA_PREFIX="a79551df"

CHECK_ONLY=0
DEST=""
while [ $# -gt 0 ]; do
  case "$1" in
    --check) CHECK_ONLY=1 ;;
    --dest)  DEST="${2:-}"; shift ;;
    -h|--help) sed -n '2,26p' "$0"; exit 0 ;;
    *) echo "모르는 인자: $1" >&2; exit 2 ;;
  esac
  shift
done

# 캐시 경로는 torch 가 정한다. torch 가 있으면 직접 물어보는 게 확실하다 —
# TORCH_HOME / XDG_CACHE_HOME 을 우리가 다시 해석하면 어긋날 수 있다.
if [ -z "$DEST" ]; then
  DEST="$(python3 -c 'import torch.hub,os;print(os.path.join(torch.hub.get_dir(),"checkpoints"))' 2>/dev/null || true)"
fi
if [ -z "$DEST" ]; then
  DEST="${TORCH_HOME:-${XDG_CACHE_HOME:-$HOME/.cache}/torch}/hub/checkpoints"
  echo "torch 를 임포트하지 못했다 — 경로를 추정한다: $DEST" >&2
fi

TARGET="$DEST/$WEIGHTS_FILE"

# 온전한지 확인한다. 크기만 보면 중간에 끊긴 파일을 통과시킨다.
verify() {
  [ -f "$TARGET" ] || return 1
  local got
  got="$(sha256sum "$TARGET" | cut -c1-8)"
  [ "$got" = "$WEIGHTS_SHA_PREFIX" ]
}

if verify; then
  echo "SSDLite 가중치 있음:  $TARGET"
  exit 0
fi

if [ -f "$TARGET" ]; then
  echo "가중치가 있으나 해시가 안 맞는다 (덜 받혔거나 깨졌다): $TARGET" >&2
  [ "$CHECK_ONLY" = "1" ] && exit 1
  rm -f "$TARGET"
fi

if [ "$CHECK_ONLY" = "1" ]; then
  echo "SSDLite 가중치 없음:  $TARGET" >&2
  echo "  인터넷 되는 자리에서 ./fetch_models.sh 를 돌릴 것." >&2
  echo "  이대로 현장에 나가면 follow_service.py 가 안 뜬다." >&2
  exit 1
fi

mkdir -p "$DEST"
echo "받는 중: $WEIGHTS_URL"
# 임시 파일에 받고 검증 뒤에 옮긴다. 받다 끊긴 파일이 캐시에 남으면
# 다음 실행이 그걸 진짜로 알고 로드하다 죽는다.
TMP="$TARGET.part"
trap 'rm -f "$TMP"' EXIT
if command -v curl >/dev/null 2>&1; then
  curl -fL --retry 3 --connect-timeout 10 -o "$TMP" "$WEIGHTS_URL"
elif command -v wget >/dev/null 2>&1; then
  wget -q --tries=3 --timeout=30 -O "$TMP" "$WEIGHTS_URL"
else
  echo "curl 도 wget 도 없다" >&2; exit 1
fi

GOT="$(sha256sum "$TMP" | cut -c1-8)"
if [ "$GOT" != "$WEIGHTS_SHA_PREFIX" ]; then
  echo "해시 불일치: 기대 $WEIGHTS_SHA_PREFIX, 받은 것 $GOT" >&2
  exit 1
fi
mv "$TMP" "$TARGET"
trap - EXIT

echo "완료:  $TARGET  ($(du -h "$TARGET" | cut -f1))"
echo
echo "확인:  python3 follow_service.py --fake   # 가중치는 --fake 에서 안 읽는다"
echo "       python3 follow_service.py          # 카메라까지 함께 확인"
