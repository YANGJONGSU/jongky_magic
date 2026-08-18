#!/usr/bin/env python3
"""지도 저장 도구 (Nav2 map_saver_cli 래퍼).

사용법:
  # 기본 이름(map_YYYYMMDD_HHMMSS)으로 현재 디렉토리에 저장
  ros2 run jongky_navigation save_map.py

  # 특정 디렉토리 및 이름으로 저장
  ros2 run jongky_navigation save_map.py --name my_lab --dir /path/to/maps
"""
import argparse
from datetime import datetime
import os
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description='Jongky Map Saver Utility')
    parser.add_argument(
        '--name',
        type=str,
        default='',
        help='저장할 맵 이름 (기본값: map_YYYYMMDD_HHMMSS)',
    )
    parser.add_argument(
        '--dir',
        type=str,
        default='',
        help='저장할 디렉토리 경로 (기본값: 현재 디렉토리)',
    )
    parser.add_argument(
        '--free-thresh',
        type=float,
        default=0.25,
        help='Free threshold (기본값: 0.25)',
    )
    parser.add_argument(
        '--occ-thresh',
        type=float,
        default=0.65,
        help='Occupied threshold (기본값: 0.65)',
    )
    parser.add_argument(
        '--timeout',
        type=float,
        default=20.0,
        help='/map 구독 대기 시간 [s] (기본값: 20). map_saver_cli 기본값 2초는 짧다',
    )

    args = parser.parse_args()

    target_dir = args.dir if args.dir else os.getcwd()
    map_name = args.name if args.name else f"map_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

    os.makedirs(target_dir, exist_ok=True)
    target_path = os.path.join(target_dir, map_name)

    print(f"[*] /map 토픽으로부터 지도를 저장합니다...")
    print(f"    저장 경로: {target_path}.yaml / {target_path}.pgm")

    # save_map_timeout 을 늘린다. 기본 2초로는 /map 구독이 붙기 전에 포기하고
    # "Failed to spin map subscription" 으로 죽는다 — 지도는 멀쩡히 발행 중인데도.
    cmd = [
        'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
        '-f', target_path,
        '--free', str(args.free_thresh),
        '--occ', str(args.occ_thresh),
        '--ros-args', '-p', f'save_map_timeout:={args.timeout}',
    ]

    try:
        res = subprocess.run(cmd, check=True)
        print(f"[+] 성공적으로 지도가 저장되었습니다: {target_path}.yaml")
    except subprocess.CalledProcessError as e:
        print(f"[-] 지도 저장 실패 (종료 코드 {e.returncode})", file=sys.stderr)
        sys.exit(e.returncode)
    except FileNotFoundError:
        print("[-] 'ros2' 명령어를 찾을 수 없습니다. ROS 2 환경을 source 했는지 확인하세요.", file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
