#!/usr/bin/env python3
"""Lay anh tu camera de cham ROI.

Anh luu ra la NGUYEN BAN tu /camXHP/image_raw -- dung he toa do ma YOLO node
publish bbox. Khong resize, khong dung /camXHP/image_overlay (anh do 640x360,
bi bop doc, cham len do se lech).

Dung:
    python3 grab_roi_frames.py                 # ca 2 cam
    python3 grab_roi_frames.py --cam cam0      # rieng cam0
    python3 grab_roi_frames.py --out ~/roi     # thu muc khac
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import Image

TOPICS = {
    'cam0': '/cam0HP/image_raw',   # input tray  -> 5 row
    'cam1': '/cam1HP/image_raw',   # output tray -> 10 slot
}


def rosimg_to_bgr(msg):
    """Chi ho tro cac encoding camera nay thuc su publish."""
    buf = np.frombuffer(msg.data, np.uint8)
    if msg.encoding == 'bgr8':
        return buf.reshape(msg.height, msg.width, 3)
    if msg.encoding == 'rgb8':
        return cv2.cvtColor(buf.reshape(msg.height, msg.width, 3), cv2.COLOR_RGB2BGR)
    if msg.encoding == 'mono8':
        return cv2.cvtColor(buf.reshape(msg.height, msg.width), cv2.COLOR_GRAY2BGR)
    raise ValueError(f'encoding chua ho tro: {msg.encoding}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cam', choices=list(TOPICS) + ['all'], default='all')
    ap.add_argument('--out', default=os.path.expanduser('~/Pictures/roi'))
    ap.add_argument('--timeout', type=float, default=20.0)
    args = ap.parse_args()

    wanted = list(TOPICS) if args.cam == 'all' else [args.cam]
    os.makedirs(args.out, exist_ok=True)

    rclpy.init()
    node = rclpy.create_node('grab_roi_frames')
    got = {}

    def make_cb(name):
        def cb(msg):
            if name not in got:
                got[name] = (rosimg_to_bgr(msg).copy(), msg.width, msg.height, msg.encoding)
        return cb

    for name in wanted:
        node.create_subscription(Image, TOPICS[name], make_cb(name), 1)

    print(f'Cho frame tu: {", ".join(TOPICS[n] for n in wanted)} ...')
    t0 = time.time()
    while len(got) < len(wanted) and time.time() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.5)

    rc = 0
    for name in wanted:
        if name not in got:
            print(f'  [LOI] {name}: khong nhan duoc frame tu {TOPICS[name]} sau {args.timeout:.0f}s')
            print('        -> camera launch dang chay chua? ros2 topic hz ' + TOPICS[name])
            rc = 1
            continue
        img, w, h, enc = got[name]
        path = os.path.join(args.out, f'{name}.png')
        cv2.imwrite(path, img)
        # Ban chinh (cam0.png) la thu roi_pick.py doc; ban timestamp de cac lan
        # chup thu cong khong de mat anh cu.
        stamp = time.strftime('%Y%m%d_%H%M%S')
        archive = os.path.join(args.out, f'{name}_{stamp}.png')
        cv2.imwrite(archive, img)
        print(f'  {name}: {w}x{h} {enc} -> {path}')
        print(f'        (ban luu: {archive})')
        # ROI se duoc so thang voi bbox center, nen kich thuoc nay CHINH LA
        # he toa do phai cham. Ghi ra de roi_pick.py doc lai va chot ref_*.
        if (w, h) != (640, 480):
            print(f'        [CANH BAO] mong doi 640x480, nhan duoc {w}x{h}.')
            print('        Kich thuoc nay van la he bbox dung -- nhung hay bao Claude')
            print('        vi image_width/image_height trong vision_decision_node phai khop.')

    rclpy.shutdown()
    if rc == 0:
        print('\nBuoc tiep: python3 roi_pick.py --cam cam0   (roi --cam cam1)')
    return rc


if __name__ == '__main__':
    sys.exit(main())
