#!/usr/bin/env python3
"""실주행 bag → DreamerV3 리플레이 에피소드 (.npz).

시뮬과 **같은 보상함수**(reward_spec.py)로 실물 로그에 보상을 다시 매긴다.
D3(실물 리플레이 미세조정)의 데이터가 여기서 나온다. Cosmos 리스타일(갈래 A)
도 같은 기계를 쓴다 — 씨앗 구간의 액션·보상을 이걸로 뽑고 픽셀만 바꾼다.

    # 카메라가 든 bag → 에피소드 생성
    python3 relabel_bag.py BAG.mcap --out /root/labels_out/episodes

    # 카메라 없는 bag (예: bags_0821) → 액션·보상 통계만 검증
    python3 relabel_bag.py BAG.mcap --dry-run

## 정렬 규칙

- 프레임 시계는 이미지 토픽의 mcap log_time 이다 (드라이-런은 /scan).
- 액션 = 프레임 시각 직전의 /cmd_vel 을 zero-order hold.
  **header stamp 가 아니라 log_time 을 쓴다** — 이 로봇의 TwistStamped 에는
  stamp 가 0 인 메시지가 섞여 있다 (bags_0821 실측).
- 액션은 [v/V_MAX, ω/OMEGA_MAX] 로 정규화해 적는다. 시뮬 정책 출력과 같은
  좌표계다 (scale_action 의 역방향).
- proprio = odom 의 (v, ω) 를 같은 정규화로. 시뮬 어댑터의 proprio 정의와
  같다 (dreamer_env.py observation_space 주석).

## 보상 항별 실물 근거

- 진행: odom 포즈 → 목표까지 거리 감소.
  목표는 hindsight — **그 에피소드가 실제로 끝난 지점**을 목표로 삼는다.
  teleop 주행에는 목표가 없지만, "이 궤적은 이 지점으로 가는 시연이었다"
  로 다시 읽으면 진행 보상이 성립한다 (goal relabeling).
- 근접: /scan 전방 ±90° 최소거리 − 로봇 반폭 → reward_spec.proximity_penalty.
  시뮬은 벽까지 측방 기하 이격이라 정의가 완전히 같지는 않다 (스캔은 전방도
  본다). 복도에서는 벽이 지배적이라 차이가 작다 — reward_spec 주석 참조.
- 충돌: 스캔 최소거리 < SCAN_COLLISION_RANGE 를 접촉으로 라벨하고
  is_terminal=True. 범퍼 센서가 없어서 이게 최선이다.
- 회전·시간: 시뮬과 동일 계수.

## 에피소드 절단

이미지 간격이 --gap 초를 넘으면 자르고, --max-len 프레임(기본 60초분)을
넘어도 자른다. 순변위 --min-disp 미만인 조각(제자리 대기)은 버린다.
"""
import argparse
import bisect
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import reward_spec as R  # noqa: E402

import mcap_lite as M  # noqa: E402


def load_streams(bag, image_topic, odom_topic, cmd_topic, scan_topic):
    """bag 을 한 번 훑어 토픽별 시계열을 만든다. 시각은 전부 log_time 초."""
    cmd_t, cmd_v = [], []
    odo_t, odo = [], []
    scan_t, scan_clear = [], []
    img_t, img = [], []
    topics = {cmd_topic, odom_topic, scan_topic}
    if image_topic:
        topics.add(image_topic)
    for topic, lt, d in M.iter_messages(bag, topics):
        t = lt / 1e9
        if topic == cmd_topic:
            try:
                _, v, w = M.decode_twist_stamped(d)
            except Exception:
                v, w = M.decode_twist(d)      # 혹시 Twist 로 기록된 bag
            cmd_t.append(t)
            cmd_v.append((v, w))
        elif topic == odom_topic:
            _, x, y, yaw, v, w = M.decode_odometry(d)
            odo_t.append(t)
            odo.append((x, y, yaw, v, w))
        elif topic == scan_topic:
            _, a0, ai, ranges = M.decode_laserscan(d)
            n = len(ranges)
            ang = a0 + ai * np.arange(n)
            front = np.abs(np.arctan2(np.sin(ang), np.cos(ang))) <= R.SCAN_FRONT_HALF_ANGLE
            r = ranges[front]
            mn = np.nanmin(r) if np.isfinite(r).any() else np.nan
            scan_t.append(t)
            scan_clear.append(mn)
        elif image_topic and topic == image_topic:
            _, im, enc = M.decode_image(d)
            if im is None:
                continue                       # 잘린 프레임
            img_t.append(t)
            img.append(_to64(im, enc))
    return (np.array(cmd_t), np.array(cmd_v, dtype=np.float32).reshape(-1, 2),
            np.array(odo_t), np.array(odo, dtype=np.float64).reshape(-1, 5),
            np.array(scan_t), np.array(scan_clear, dtype=np.float32),
            np.array(img_t), img)


def _to64(im, enc):
    import cv2
    if enc == "bgr8":
        im = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
    # 640x480(4:3) → 64x64. 세로가 눌린다. 시뮬 카메라와 이 왜곡이 같은지는
    # 열린 문제다 — 문서 '카메라 화면비' 절 참조.
    return cv2.resize(im, (64, 64), interpolation=cv2.INTER_AREA)


def hold(ts, xs, t):
    """t 직전 표본 (zero-order hold). 표본 이전 시각이면 첫 값."""
    i = bisect.bisect_right(ts.tolist(), t) - 1
    return xs[max(i, 0)]


def cut_segments(frame_t, gap, max_len):
    segs, start = [], 0
    for i in range(1, len(frame_t)):
        if frame_t[i] - frame_t[i - 1] > gap or i - start >= max_len:
            segs.append((start, i))
            start = i
    if len(frame_t) - start > 1:
        segs.append((start, len(frame_t)))
    return segs


def relabel_segment(frame_t, cmd_t, cmd_v, odo_t, odo, scan_t, scan_clear,
                    fragment=False):
    """한 에피소드 구간의 (액션, proprio, 보상, is_terminal, 진단) 계산.

    fragment=True 는 씨앗/리스타일 조각용: 에피소드가 아니라 긴 주행의
    일부이므로 ①hindsight 도달 절단을 하지 않고 ②도달 보너스도 주지 않는다
    (조각 끝 = 목표라는 건 인위적 설정이라, +50 을 흩뿌리면 보상 헤드가
    '아무 데서나 +50' 을 배운다). 진행·근접·시간·회전 항만 남는다.
    """
    T = len(frame_t)
    act = np.zeros((T, 2), dtype=np.float32)
    prop = np.zeros((T, 2), dtype=np.float32)
    pose = np.zeros((T, 3), dtype=np.float64)
    clear = np.zeros(T, dtype=np.float32)
    for i, t in enumerate(frame_t):
        v, w = hold(cmd_t, cmd_v, t)
        act[i] = (np.clip(v / R.V_MAX, -1, 1), np.clip(w / R.OMEGA_MAX, -1, 1))
        x, y, yaw, ov, ow = hold(odo_t, odo, t)
        pose[i] = (x, y, yaw)
        prop[i] = (np.clip(ov / R.V_MAX, -1, 1), np.clip(ow / R.OMEGA_MAX, -1, 1))
        c = hold(scan_t, scan_clear, t)
        clear[i] = (c - R.ROBOT_HALF_WIDTH) if np.isfinite(c) else 10.0

    goal = pose[-1, :2]                        # hindsight 목표
    dist = np.linalg.norm(pose[:, :2] - goal, axis=1)

    # 시뮬은 도달 즉시 종료라 보너스가 한 번만 나온다. 여기서도 첫 도달
    # 프레임에서 에피소드를 자른다 — 안 자르면 목표 반경 안의 모든 프레임이
    # +50 을 받아 보상 분포가 시뮬과 갈린다 (bags_0821 드라이런에서 확인).
    hit = np.nonzero(dist < R.GOAL_RADIUS)[0]
    if fragment:
        hit = hit[:0]                          # 절단·보너스·종료 모두 끔
    k = int(hit[0]) + 1 if len(hit) else T
    T = k
    act, prop, pose, clear, dist = act[:k], prop[:k], pose[:k], clear[:k], dist[:k]
    frame_t = frame_t[:k]

    progress = np.zeros(T, dtype=np.float32)
    progress[1:] = R.clamp_progress((dist[:-1] - dist[1:]) * R.REW_PROGRESS, np)
    reached = np.zeros(T, dtype=np.float32)
    if len(hit):
        reached[-1] = R.REW_GOAL
    contact = clear + R.ROBOT_HALF_WIDTH < R.SCAN_COLLISION_RANGE
    collision = contact.astype(np.float32) * R.REW_COLLISION
    ang = prop[:, 1] * R.OMEGA_MAX
    spin = np.abs(ang).astype(np.float32) * R.REW_SPIN
    proximity = R.proximity_penalty(clear, np).astype(np.float32)

    reward = progress + reached + collision + spin + proximity + R.REW_TIME
    # 진짜 종료 = 접촉, 그리고 목표 도달 (시뮬 _get_dones 와 같은 의미론 —
    # 둘 다 terminated 로 취급된다). 시간·길이 절단은 is_terminal 이 아니다.
    is_terminal = contact.copy()
    if len(hit):
        is_terminal[-1] = True
    diag = {"disp": float(np.linalg.norm(pose[-1, :2] - pose[0, :2])),
            "contact_frames": int(contact.sum()),
            "min_clear": float(clear.min()),
            "mean_reward": float(reward.mean())}
    return act, prop, reward, is_terminal, dist, diag


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--out", default="/root/labels_out/episodes")
    ap.add_argument("--image-topic", default="/camera/rgb/image_raw")
    ap.add_argument("--odom-topic", default="/odometry/filtered",
                    help="없으면 --odom-topic /odom 으로")
    ap.add_argument("--cmd-topic", default="/cmd_vel")
    ap.add_argument("--scan-topic", default="/scan")
    ap.add_argument("--gap", type=float, default=0.5, help="이 간격[s] 넘으면 에피소드 절단")
    ap.add_argument("--max-len", type=int, default=1800, help="에피소드 최대 프레임 (60s@30Hz)")
    ap.add_argument("--min-disp", type=float, default=1.0, help="순변위[m] 미만 조각은 버림")
    ap.add_argument("--dry-run", action="store_true",
                    help="이미지 없이 /scan 시계로 액션·보상 통계만 출력")
    a = ap.parse_args()

    (cmd_t, cmd_v, odo_t, odo, scan_t, scan_clear,
     img_t, imgs) = load_streams(a.bag, None if a.dry_run else a.image_topic,
                                 a.odom_topic, a.cmd_topic, a.scan_topic)
    if len(odo_t) == 0:
        sys.exit("odom 이 없다: %s (--odom-topic /odom 을 시도해 볼 것)" % a.odom_topic)
    frame_t = scan_t if a.dry_run else img_t
    if len(frame_t) == 0:
        sys.exit("프레임 시계가 비었다 — 이미지 토픽이 이 bag 에 있는지 확인: %s" % a.image_topic)

    print("스트림: cmd %d · odom %d · scan %d · frame %d"
          % (len(cmd_t), len(odo_t), len(scan_t), len(frame_t)))

    os.makedirs(a.out, exist_ok=True)
    kept = skipped = 0
    for s, e in cut_segments(frame_t, a.gap, a.max_len):
        seg_t = frame_t[s:e]
        act, prop, rew, term, dist, diag = relabel_segment(
            seg_t, cmd_t, cmd_v, odo_t, odo, scan_t, scan_clear)
        if diag["disp"] < a.min_disp:
            skipped += 1
            continue
        kept += 1
        T = len(rew)                          # 도달에서 잘렸을 수 있다
        e = s + T
        tag = "%s_ep%03d" % (os.path.splitext(os.path.basename(a.bag))[0], kept)
        print("%s: %d프레임 변위 %.1fm 이격최소 %.2fm 접촉 %d 평균보상 %+.3f"
              % (tag, T, diag["disp"], diag["min_clear"],
                 diag["contact_frames"], diag["mean_reward"]))
        if a.dry_run:
            continue
        is_first = np.zeros(T, dtype=bool)
        is_first[0] = True
        is_last = np.zeros(T, dtype=bool)
        is_last[-1] = True
        ep = {
            "image": np.stack(imgs[s:e]),
            "proprio": prop,
            "action": act,
            "reward": rew,
            "is_first": is_first,
            "is_last": is_last,
            "is_terminal": term,
            # dreamer train_eps 실물 규격의 나머지 두 키 (pack_episodes 참조)
            "discount": (1.0 - term.astype(np.float32)),
            "logprob": np.zeros(T, dtype=np.float32),
        }
        np.savez_compressed(os.path.join(a.out, tag + ".npz"), **ep)
        json.dump({"bag": a.bag, "t0": float(seg_t[0]), "t1": float(seg_t[-1]),
                   "odom_topic": a.odom_topic, **diag},
                  open(os.path.join(a.out, tag + ".json"), "w"))
    print("에피소드 %d개 저장, %d개 버림(변위<%.1fm)%s"
          % (kept, skipped, a.min_disp, " [dry-run: 저장 안 함]" if a.dry_run else ""))


if __name__ == "__main__":
    main()
