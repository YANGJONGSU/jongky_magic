#!/usr/bin/env python3
"""씨앗 × 조건 조합을 순서대로 돌려서 합성 클립을 쌓는다.

    cd /workspace/Cosmos1GP
    nohup python3 /workspace/jongky_magic/deploy/runpod/run_batch.py \
        --seeds-dir /workspace/jongky_magic/deploy/runpod/seeds \
        > /workspace/batch.log 2>&1 &

    tail -f /workspace/batch.log

## 순서를 왜 이렇게 잡았나

조건을 바깥 고리, 씨앗을 안쪽 고리로 돈다. 즉 **조건 1을 모든 지점에 대해
먼저** 끝내고 조건 2로 넘어간다. 중간에 멈춰도 노선 전체가 한 번은 덮이도록
하기 위해서다. 반대로 하면 한 지점만 조건 4개를 갖고 나머지는 아무것도 없다.

강화학습 데이터에서 중요한 건 한 곳의 깊이가 아니라 상황의 넓이다.

## 재시작

이미 만든 조합은 manifest 를 보고 건너뛴다. 끊겼다 다시 켜도 이어서 간다.
"""
import argparse
import fnmatch
import glob
import json
import os
import re
import time

from gradio_client import Client, handle_file

NEG = (
    "The video captures a series of frames showing ugly scenes, static with no motion, "
    "motion blur, over-saturation, shaky footage, low resolution, grainy texture, "
    "pixelated images, poorly lit areas, underexposed and overexposed scenes, poor color "
    "balance, washed out colors, choppy sequences, jerky movements, low frame rate, "
    "artifacting, color banding, unnatural transitions, outdated special effects, fake "
    "elements, unconvincing visuals, poorly edited content, jump cuts, visual noise, and "
    "flickering. Overall, the video is of poor quality."
)


def seeds_for(cond_path, all_seeds):
    """이 조건을 어느 씨앗에 적용할지.

    조건 파일 옆에 같은 이름의 `.seeds` 가 있으면 거기 적힌 무늬에 맞는
    씨앗에만 적용한다. 없으면 전부.

    왜 필요한가 — 엘리베이터나 사물함처럼 **특정 지점에만 있는 것**을 조건으로
    쓰면, 그게 안 보이는 복도 씨앗에서는 모델이 없던 구조를 만들어낸다.
    벽을 뚫어 엘리베이터를 그리는 순간 "복도 폭과 벽 위치는 안 변한다" 가
    깨지고, 그 클립은 학습에 못 쓴다. depth 와 액션의 관계가 거짓이 되기 때문이다.
    """
    side = os.path.splitext(cond_path)[0] + ".seeds"
    if not os.path.isfile(side):
        return all_seeds
    pats = [ln.strip() for ln in open(side, encoding="utf-8")
            if ln.strip() and not ln.startswith("#")]
    out = [s for s in all_seeds
           if any(fnmatch.fnmatch(os.path.basename(s), p) for p in pats)]
    if not out:
        raise SystemExit("%s 의 무늬에 맞는 씨앗이 없다: %s"
                         % (os.path.basename(side), ", ".join(pats)))
    return out


def load_done(path):
    done = set()
    if os.path.isfile(path):
        with open(path, encoding="utf-8") as fh:
            for ln in fh:
                try:
                    r = json.loads(ln)
                except ValueError:
                    continue
                if r.get("ok"):
                    done.add((r["seed_file"], r["cond"]))
    return done


def main():
    p = argparse.ArgumentParser(description="Cosmos 배치 생성")
    p.add_argument("--seeds-dir", required=True)
    p.add_argument("--seed-glob", default="corridor_1*f_[0-9]*.mp4")
    p.add_argument("--cond-glob", default="cond_*.txt")
    p.add_argument("--url", default="http://localhost:7860")
    p.add_argument("--outputs", default="/workspace/Cosmos1GP/outputs",
                   help="서버가 결과를 쓰는 폴더. 절대경로라 실행 위치와 무관하다")
    p.add_argument("--manifest", default="/workspace/batch_manifest.jsonl")
    p.add_argument("--resolution", default="832x624", help="4:3 만")
    p.add_argument("--length", type=int, default=121)
    p.add_argument("--steps", type=int, default=20)
    p.add_argument("--guidance", type=float, default=7.0)
    p.add_argument("--max-frames", type=int, default=9)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--limit", type=int, default=0, help="0 이면 전부")
    a = p.parse_args()

    seeds = sorted(glob.glob(os.path.join(a.seeds_dir, a.seed_glob)))
    conds = sorted(glob.glob(os.path.join(a.seeds_dir, a.cond_glob)))
    if not seeds:
        raise SystemExit("씨앗을 못 찾겠다: %s" % a.seed_glob)
    if not conds:
        raise SystemExit("조건 프롬프트를 못 찾겠다: %s" % a.cond_glob)

    done = load_done(a.manifest)
    # 조건이 바깥, 씨앗이 안쪽 — 멈춰도 노선 전체가 덮이도록
    jobs = [(s, c) for c in conds for s in seeds_for(c, seeds)
            if (os.path.basename(s), os.path.basename(c)) not in done]
    if a.limit:
        jobs = jobs[:a.limit]

    total = sum(len(seeds_for(c, seeds)) for c in conds)
    print("씨앗 %d · 조건 %d · 전체 %d조합 · 남은 작업 %d개"
          % (len(seeds), len(conds), total, len(jobs)), flush=True)
    for c in conds:
        n = len(seeds_for(c, seeds))
        print("   %-28s 씨앗 %d개%s" % (os.path.basename(c), n,
              "" if n == len(seeds) else "  (일부에만 적용)"), flush=True)
    print("해상도 %s · %d프레임 · %d스텝" % (a.resolution, a.length, a.steps), flush=True)
    if not jobs:
        print("할 게 없다."); return

    c = Client(a.url, verbose=False)
    times = []
    for i, (sv, cf) in enumerate(jobs, 1):
        # 줄바꿈을 반드시 없앤다. gradio_server_v2w.py:428 이
        #     prompts = prompt.replace("\r", "").split("\n")
        # 로 프롬프트를 줄 단위로 쪼개고 **한 줄당 영상을 하나씩** 만든다.
        # 여러 줄로 보내면 각 클립이 문장 조각만 받는다 — 첫 배치에서
        # "...low objects left on the floor along the" 에서 끊긴 줄이
        # 그대로 프롬프트가 돼서, 무엇을 놓으라는 건지 말하지도 못했다.
        prompt = re.sub(r"\s+", " ", open(cf, encoding="utf-8").read()).strip()
        before = set(glob.glob(os.path.join(a.outputs, "*.mp4")))
        t0 = time.time()
        eta = ""
        if times:
            avg = sum(times) / len(times)
            eta = " · 남은 예상 %.1f시간" % (avg * (len(jobs) - i + 1) / 3600)
        print("[%d/%d] %s × %s%s" % (i, len(jobs), os.path.basename(sv),
                                     os.path.basename(cf), eta), flush=True)
        rec = {"seed_file": os.path.basename(sv), "cond": os.path.basename(cf),
               "resolution": a.resolution, "steps": a.steps}
        try:
            c.predict(
                prompt=prompt, neg_prompt=NEG, resolution=a.resolution,
                video_length=a.length, seed=a.seed, num_inference_steps=a.steps,
                embedded_guidance_scale=a.guidance, repeat_generation=1, tea_cache=0,
                image_to_continue=None,
                video_to_continue=handle_file(os.path.abspath(sv)),
                max_frames=a.max_frames, api_name="/generate_video",
            )
            dt = time.time() - t0
            times.append(dt)
            new = sorted(set(glob.glob(os.path.join(a.outputs, "*.mp4"))) - before)
            rec.update(ok=True, seconds=round(dt, 1), out=[os.path.basename(x) for x in new])
            print("      %.1f분 · %s" % (dt / 60, ", ".join(rec["out"]) or "(새 파일 없음)"),
                  flush=True)
        except Exception as e:
            rec.update(ok=False, error="%s: %s" % (type(e).__name__, e))
            print("      실패: %s" % rec["error"], flush=True)
        with open(a.manifest, "a", encoding="utf-8") as fh:
            fh.write(json.dumps(rec, ensure_ascii=False) + "\n")

    if times:
        print("\n완료 %d개 · 평균 %.1f분 · 합계 %.1f시간"
              % (len(times), sum(times) / len(times) / 60, sum(times) / 3600))


if __name__ == "__main__":
    main()
