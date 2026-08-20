#!/usr/bin/env python3
"""cuDNN 이 실제로 도는지, 딸린 nvidia-* 패키지가 torch 와 맞는지 본다.

    python3 diag_cudnn.py

## 왜 필요한가

`CUDNN_STATUS_NOT_INITIALIZED` 는 원인을 안 알려주는 메시지다. 셋 중 하나다.

  1. torch 휠이 들고 온 `nvidia-cudnn-cu12` 버전이 torch 와 안 맞는다
  2. 메모리가 모자라서 cuDNN 이 작업공간을 못 잡는다
  3. 드라이버/장치 문제

이 파드는 torch 를 2.2 → 2.13 → 2.4.1 → 2.5.1 로 갈아치웠다. torch 본체는
제약으로 묶었지만 `nvidia-*` 딸림 패키지는 안 묶어서, 다른 torch 가 끌고 온
판이 남아 있을 수 있다. 1번이 가장 유력하다.
"""
import importlib.metadata as md
import sys

print("=" * 60)
print(" 설치된 nvidia-* / torch")
print("=" * 60)
rows = []
for dist in md.distributions():
    name = dist.metadata["Name"] or ""
    if name.startswith("nvidia-") or name in ("torch", "torchvision", "torchaudio", "triton"):
        rows.append((name, dist.version))
for n, v in sorted(rows):
    print("  %-28s %s" % (n, v))

import torch
print()
print("=" * 60)
print(" cuDNN")
print("=" * 60)
print("  torch            ", torch.__version__)
print("  torch 가 빌드된 cudnn", torch.backends.cudnn.version())
print("  cudnn available  ", torch.backends.cudnn.is_available())
print("  cudnn enabled    ", torch.backends.cudnn.enabled)

print()
print("=" * 60)
print(" 실제 합성곱 — 여기가 터진 자리다")
print("=" * 60)
ok = True
# 토크나이저가 쓰는 건 Conv3d 다. 2d 도 같이 봐서 어느 쪽만 깨졌는지 가른다.
for label, mk, shape in (
    ("Conv2d", lambda: torch.nn.Conv2d(3, 8, 3, padding=1), (1, 3, 64, 64)),
    ("Conv3d", lambda: torch.nn.Conv3d(3, 8, 3, padding=1), (1, 3, 8, 64, 64)),
):
    try:
        m = mk().cuda()
        y = m(torch.randn(*shape, device="cuda"))
        print("  %-8s OK  %s" % (label, tuple(y.shape)))
    except Exception as e:
        ok = False
        print("  %-8s 실패: %s: %s" % (label, type(e).__name__, e))

print()
if ok:
    print(" cuDNN 은 정상이다. 그러면 생성 중 실패는 메모리 쪽일 가능성이 크다.")
    free, total = torch.cuda.mem_get_info()
    print(" VRAM 여유 %.1f / %.1f GB" % (free / 1e9, total / 1e9))
else:
    print(" cuDNN 이 깨졌다. torch 를 --force-reinstall 로 다시 깔아")
    print(" nvidia-* 딸림 패키지를 한 벌로 맞춰야 한다.")
    sys.exit(1)
