#!/usr/bin/env python3
"""뎁스 카메라의 최소 측정거리를 실측한다.

구조광 카메라는 일정 거리보다 가까우면 깊이값이 아예 안 나온다.
스펙시트 값보다 실측이 정확하므로 직접 확인한다.

쓰는 법
  1. 카메라 드라이버를 띄운다.
  2. ros2 run jongky_description check_depth_min_range.py
     (토픽 이름이 다르면)  ... check_depth_min_range.py /내/뎁스/토픽
  3. 평평한 물체(책 표지)를 카메라 정면 1m 쯤에서 천천히 가까이 가져간다.
  4. "값 없음"으로 바뀌는 순간 직전의 거리가 최소 측정거리다.

화면 가운데 20x20 픽셀만 본다.
"""
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class DepthMin(Node):
    def __init__(self, topic):
        super().__init__('check_depth_min_range')
        self.create_subscription(Image, topic, self.cb, 10)
        self.best = None
        print('토픽: %s' % topic)
        print('물체를 카메라 정면에서 천천히 가까이 가져가세요. Ctrl-C 로 종료.\n')

    def cb(self, msg):
        if msg.encoding == '16UC1':
            a = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width).astype(float)
        elif msg.encoding == '32FC1':
            a = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width).astype(float) * 1000.0
        else:
            print('지원 안 하는 인코딩: %s' % msg.encoding)
            raise SystemExit

        h, w = a.shape
        patch = a[h // 2 - 10:h // 2 + 10, w // 2 - 10:w // 2 + 10]
        valid = patch[(patch > 0) & np.isfinite(patch)]
        ratio = len(valid) / patch.size

        if ratio < 0.3:
            print('  가운데 값 없음   (유효 픽셀 %3.0f%%)   <- 최소거리보다 가깝다' % (ratio * 100))
        else:
            d = float(np.median(valid))
            if self.best is None or d < self.best:
                self.best = d
            print('  거리 %7.0f mm   (유효 픽셀 %3.0f%%)   지금까지 최소 %.0f mm'
                  % (d, ratio * 100, self.best))


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else '/camera/depth/image_raw'
    rclpy.init()
    node = DepthMin(topic)
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    if node.best is not None:
        print('\n실측 최소 측정거리 ≈ %.0f mm' % node.best)
        print('(값이 나온 것 중 가장 가까운 거리. 이보다 가까우면 깊이가 안 나온다)')
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
