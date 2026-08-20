#!/usr/bin/env python3
"""Cosmos Video2World 로 씨앗 영상의 변형본을 만든다.

    python gen_clip.py --seed-video seeds/corridor_10f.mp4 \
                       --prompt-file seeds/prompt_v1.txt \
                       --resolution 1104x832

한 번 호출에 4~5초(121프레임) 클립 하나가 나온다. 20분짜리 원본을 넣어
20분을 받는 물건이 아니다 — 씨앗 몇 초를 조건으로 새 클립을 생성한다.

**기하를 지켜야 한다.** 복도 폭·벽 위치·카메라 높이·FOV 가 바뀌면 depth 와
액션의 관계가 깨져서 "이 상황에서 이 액션은 안전하다" 는 거짓 신호가 된다.
그래서
  · 해상도는 씨앗과 같은 화면비를 쓴다 (640x480 → 1104x832 또는 832x624, 둘 다 4:3)
  · 프롬프트 앞부분에 씨앗 장면을 그대로 서술하고 "폭·벽·카메라가 안 바뀐다" 를 명시
  · 조명·사람·재질만 바꾼다
  · 길게 뽑지 않는다 (길수록 기하가 흘러간다)
"""
import argparse
import os
import time

from gradio_client import Client, handle_file

# Cosmos 기본 네거티브. 저품질·흔들림·정지 화면을 막는다.
NEG = (
    "The video captures a series of frames showing ugly scenes, static with no motion, "
    "motion blur, over-saturation, shaky footage, low resolution, grainy texture, "
    "pixelated images, poorly lit areas, underexposed and overexposed scenes, poor color "
    "balance, washed out colors, choppy sequences, jerky movements, low frame rate, "
    "artifacting, color banding, unnatural transitions, outdated special effects, fake "
    "elements, unconvincing visuals, poorly edited content, jump cuts, visual noise, and "
    "flickering. Overall, the video is of poor quality."
)


def main():
    p = argparse.ArgumentParser(description="Cosmos Video2World 클립 생성")
    p.add_argument("--seed-video", required=True, help="조건으로 쓸 씨앗 mp4")
    p.add_argument("--prompt-file", required=True, help="프롬프트 텍스트 파일")
    p.add_argument("--url", default="http://localhost:7860")
    p.add_argument("--resolution", default="1104x832",
                   help="씨앗과 같은 화면비로. 4:3 은 1104x832(720p) / 832x624(540p)")
    p.add_argument("--length", type=int, default=121, choices=[121, 242],
                   help="121=5초, 242=10초. 길수록 기하가 흘러간다")
    p.add_argument("--steps", type=int, default=25)
    p.add_argument("--guidance", type=float, default=7.0)
    p.add_argument("--seed", type=int, default=42, help="-1 이면 무작위")
    p.add_argument("--max-frames", type=int, default=9,
                   help="씨앗에서 조건으로 쓸 프레임 수")
    p.add_argument("--repeat", type=int, default=1, help="같은 설정으로 몇 개 뽑을지")
    p.add_argument("--log", default="/workspace/v2w.log",
                   help="실패했을 때 들여다볼 서버 로그")
    a = p.parse_args()

    prompt = open(a.prompt_file).read().strip()
    print("프롬프트 %d자" % len(prompt))
    print("씨앗 %s · %s · %d프레임 · %d스텝" %
          (os.path.basename(a.seed_video), a.resolution, a.length, a.steps))

    c = Client(a.url, verbose=False)
    t0 = time.time()
    print("생성 시작", time.strftime("%H:%M:%S"), flush=True)
    try:
        r = call(c, a, prompt)
    except Exception as e:
        # gradio 서버가 show_error=True 없이 뜨면 클라이언트에는
        # "예외가 났다" 는 사실만 오고 내용이 안 온다. 진짜 traceback 은
        # 서버 stdout, 즉 로그 파일에 있다. 사람이 다시 찾아 들어가지 않도록
        # 여기서 바로 꺼내 보여준다.
        print("\n실패:", type(e).__name__, e)
        print("\n--- 서버 로그 마지막 40줄 (%s) ---" % a.log)
        try:
            with open(a.log, encoding="utf-8", errors="replace") as fh:
                for ln in fh.read().splitlines()[-40:]:
                    print("  " + ln)
        except OSError as oe:
            print("  로그를 못 읽는다:", oe)
        raise SystemExit(1)
    dt = time.time() - t0
    print("완료 %s · 소요 %.1f분 (%.0f초) · 클립당 %.1f분" %
          (time.strftime("%H:%M:%S"), dt / 60, dt, dt / 60 / max(a.repeat, 1)), flush=True)
    print("반환:", r, flush=True)
    print("결과는 outputs/ 에 쌓인다")


def call(c, a, prompt):
    return c.predict(
        prompt=prompt,
        neg_prompt=NEG,
        resolution=a.resolution,
        video_length=a.length,
        seed=a.seed,
        num_inference_steps=a.steps,
        embedded_guidance_scale=a.guidance,
        repeat_generation=a.repeat,
        tea_cache=0,
        image_to_continue=None,
        video_to_continue=handle_file(os.path.abspath(a.seed_video)),
        max_frames=a.max_frames,
        api_name="/generate_video",
    )


if __name__ == "__main__":
    main()
