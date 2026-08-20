#!/usr/bin/env python3
"""T5 인코더 로딩이 왜 실패하는지 스택까지 본다.

    cd /workspace/Cosmos1GP && python3 /workspace/jongky_magic/deploy/runpod/diag_t5.py

## 왜 따로 필요한가

`t5_text_encoder.py` 는 로딩을 try/except 로 감싸고 실패하면
`log.warning(... {e})` 로 넘긴 뒤 폴백을 시도한다. 그래서 화면에 뜨는 건
폴백이 실패한 traceback 이고, **진짜 원인은 str(e) 한 조각으로 뭉개진다.**
KeyError 면 키 이름만 남는다 — 어느 파일 어느 줄에서 났는지가 사라진다.

여기서는 같은 호출을 그대로 하되 아무것도 삼키지 않는다.
"""
import json
import os
import struct
import sys
import traceback

CKPT = os.environ.get("CKPT", "checkpoints")
ENC = os.path.join(CKPT, "text_encoder")
NAME = os.environ.get("T5", "T5XXLEncoder_11B_quanto_int8.safetensors")
PATH = os.path.join(ENC, NAME)


def line(t):
    print("\n" + "=" * 60 + "\n " + t + "\n" + "=" * 60)


line("1. 버전")
import importlib.metadata as md
for p in ("torch", "numpy", "transformers", "mmgp", "optimum-quanto",
          "safetensors", "accelerate"):
    try:
        print("  %-16s %s" % (p, md.version(p)))
    except md.PackageNotFoundError:
        print("  %-16s (없음)" % p)

line("2. 파일")
print("  cwd      ", os.getcwd())
print("  경로     ", PATH)
print("  있음     ", os.path.isfile(PATH))
if not os.path.isfile(PATH):
    sys.exit("  !! 파일이 없다. cd /workspace/Cosmos1GP 에서 실행할 것.")
print("  크기     %.2f GB" % (os.path.getsize(PATH) / 1e9))
for f in sorted(os.listdir(ENC)):
    print("           ", f)

line("3. safetensors 헤더 메타데이터")
# mmgp 는 metadata['quantization_map'] 을 읽어서 양자화 구조를 복원한다.
# 그게 없거나 형식이 다르면 그 아래에서 KeyError 가 난다.
with open(PATH, "rb") as fh:
    n = struct.unpack("<Q", fh.read(8))[0]
    hdr = json.loads(fh.read(n))
meta = hdr.get("__metadata__", {})
print("  텐서 수  ", len(hdr) - (1 if "__metadata__" in hdr else 0))
print("  메타 키  ", list(meta.keys()))
qm = meta.get("quantization_map")
print("  quantization_map 있음:", qm is not None)
if qm:
    try:
        d = json.loads(qm) if isinstance(qm, str) else qm
        print("  항목 수  ", len(d))
        for k in list(d)[:3]:
            print("    ", k, "->", d[k])
    except Exception as e:
        print("  파싱 실패:", e)

# 문제의 이름이 파일 안에서 어떤 키로 저장돼 있는지 그대로 본다
line("4. 'SelfAttention.q' 관련 실제 키")
pref = "encoder.block.0.layer.0.SelfAttention.q"
ks = [k for k in hdr if k.startswith(pref)]
for k in sorted(ks):
    print("   ", k, hdr[k].get("dtype"), hdr[k].get("shape"))
if not ks:
    print("    (없음 — 접두어 자체가 다르다)")
    print("    앞쪽 키 몇 개:")
    for k in list(hdr)[:8]:
        if k != "__metadata__":
            print("     ", k)

line("5. 토크나이저")
try:
    from transformers import T5TokenizerFast
    tok = T5TokenizerFast.from_pretrained(ENC)
    print("  OK", type(tok).__name__, "· vocab", tok.vocab_size)
except Exception:
    traceback.print_exc()

line("6. 인코더 로딩 — 여기가 삼켜지던 곳")
try:
    from mmgp import offload
    m = offload.fast_load_transformers_model(PATH)
    print("  OK", type(m).__name__)
except Exception:
    traceback.print_exc()
    print("\n  ^ 이 스택이 진짜 원인이다.")
