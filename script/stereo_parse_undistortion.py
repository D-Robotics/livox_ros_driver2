#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
stereo_nv12_rectify.py
假设输入 NV12 为垂直拼接：上=左目，下=右目
fov_scale 从 calib.yaml 读取
"""

import os
import glob
import argparse
import numpy as np
import cv2
import yaml


def load_yaml_calib(yaml_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    s = data['stereo0']
    c0, c1 = s['cam0'], s['cam1']

    def parse_cam(cam):
        fx, fy, cx, cy = cam['intrinsics']
        w, h = cam['resolution']
        K = np.array([[fx, 0, cx],
                      [0, fy, cy],
                      [0,  0,  1]], dtype=np.float64)
        dist_model = cam.get('distortion_model', 'radtan').lower()
        d = np.array(cam.get('distortion_coeffs', []), dtype=np.float64).reshape(-1,)
        if dist_model in ('radtan', 'rational_polynomial'):
            D = d
            model = 'pinhole'
        elif dist_model == 'equidistant':
            D = d[:4] if d.size >= 4 else np.zeros(4, dtype=np.float64)
            model = 'fisheye'
        else:
            raise ValueError(f'Unsupported distortion_model: {dist_model}')
        return K, D, (w, h), model

    K0, D0, size0, model0 = parse_cam(c0)
    K1, D1, size1, model1 = parse_cam(c1)
    if size0 != size1:
        raise ValueError(f'Resolution mismatch: {size0} vs {size1}')
    if model0 != model1:
        raise ValueError(f'Model mismatch: cam0={model0}, cam1={model1}')
    T = np.array(c1['T_cn_cnm1'], dtype=np.float64)
    R = T[:3, :3].copy()
    t = T[:3, 3].reshape(3, 1).copy()

    # 从配置读取 fov_scale，默认 1.0
    fov_scale = float(c1.get('fov_scale', 1.0))

    return K0, D0, K1, D1, size0, R, t, model0, fov_scale


def build_rectify_maps(K0, D0, K1, D1, size, R, t, model,
                       alpha=0.0, balance=0.0, fov_scale=1.0):
    w, h = size
    image_size = (w, h)

    if model == 'fisheye':
        try:
            R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(
                K0, D0, K1, D1, image_size, R, t,
                flags=cv2.CALIB_ZERO_DISPARITY,
                newImageSize=image_size,
                balance=balance,
                fov_scale=fov_scale
            )
        except TypeError:
            R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(
                K0, D0, K1, D1, image_size, R, t,
                flags=cv2.CALIB_ZERO_DISPARITY,
                newImageSize=image_size,
                balance=balance
            )
            if abs(fov_scale - 1.0) > 1e-6:
                for P in (P1, P2):
                    P[0, 0] *= fov_scale
                    P[1, 1] *= fov_scale

        map1x, map1y = cv2.fisheye.initUndistortRectifyMap(
            K0, D0, R1, P1, image_size, cv2.CV_32FC1
        )
        map2x, map2y = cv2.fisheye.initUndistortRectifyMap(
            K1, D1, R2, P2, image_size, cv2.CV_32FC1
        )
    else:
        R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
            K0, D0, K1, D1, image_size, R, t,
            flags=cv2.CALIB_ZERO_DISPARITY,
            alpha=float(alpha)
        )
        if abs(fov_scale - 1.0) > 1e-6:
            for P in (P1, P2):
                P[0, 0] *= fov_scale
                P[1, 1] *= fov_scale

        map1x, map1y = cv2.initUndistortRectifyMap(
            K0, D0, R1, P1, image_size, cv2.CV_32FC1
        )
        map2x, map2y = cv2.initUndistortRectifyMap(
            K1, D1, R2, P2, image_size, cv2.CV_32FC1
        )
    return (map1x, map1y), (map2x, map2y)


def nv12_to_bgr(nv12_bytes, w, h):
    y_size = w * h
    uv_size = w * h // 2
    if len(nv12_bytes) != y_size + uv_size:
        raise ValueError(f'NV12 size mismatch: got {len(nv12_bytes)}, expected {y_size + uv_size}')
    y = np.frombuffer(nv12_bytes[:y_size], dtype=np.uint8).reshape((h, w))
    uv = np.frombuffer(nv12_bytes[y_size:], dtype=np.uint8).reshape((h // 2, w))
    yuv = np.vstack((y, uv))
    return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)


def read_nv12_vstack(path, w, h):
    W, H = w, 2 * h
    with open(path, 'rb') as f:
        raw = f.read()
    if len(raw) != W * H + (W * H) // 2:
        raise ValueError(f'NV12 size mismatch: expected {int(1.5 * W * H)}, got {len(raw)}')
    bgr = nv12_to_bgr(raw, W, H)
    return bgr[:h, :, :].copy(), bgr[h:, :, :].copy()


def rectify_and_save(left_bgr, right_bgr, maps_left, maps_right, out_prefix,
                     stereo_dir, cam0_dir, cam1_dir, cam_combine_dir, num):
    (map1x, map1y) = maps_left
    (map2x, map2y) = maps_right
    rect_l = cv2.remap(left_bgr, map1x, map1y, interpolation=cv2.INTER_LINEAR)
    rect_r = cv2.remap(right_bgr, map2x, map2y, interpolation=cv2.INTER_LINEAR)
    combine = np.vstack((rect_l, rect_r))

    cv2.imwrite(os.path.join(stereo_dir, f'left{num:06d}.png'), rect_l)
    cv2.imwrite(os.path.join(stereo_dir, f'right{num:06d}.png'), rect_r)

    cv2.imwrite(os.path.join(cam0_dir, f'{out_prefix}.png'), rect_l)
    cv2.imwrite(os.path.join(cam1_dir, f'{out_prefix}.png'), rect_r)
    cv2.imwrite(os.path.join(cam_combine_dir, f'{out_prefix}.png'), combine)

    print(f'[OK] {out_prefix}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--yaml', required=True, help='calibration yaml path')
    ap.add_argument('--input_dir', required=True, help='folder with vertically stacked NV12 files')
    ap.add_argument('--out_dir', required=True, help='output folder for PNG')
    args = ap.parse_args()

    K0, D0, K1, D1, size, R, t, model, fov_scale = load_yaml_calib(args.yaml)
    maps_left, maps_right = build_rectify_maps(K0, D0, K1, D1, size, R, t, model,
                                               alpha=0.0, balance=0.0, fov_scale=fov_scale)
    out_dir = args.out_dir
    stereo_dir = os.path.join(out_dir, 'stereo')
    cam0_dir = os.path.join(out_dir, 'cam0/data/')
    cam1_dir = os.path.join(out_dir, 'cam1/data/')
    cam_combine_dir = os.path.join(out_dir, 'cam_combine/data/')

    os.makedirs(stereo_dir, exist_ok=True)
    os.makedirs(cam0_dir, exist_ok=True)
    os.makedirs(cam1_dir, exist_ok=True)
    os.makedirs(cam_combine_dir, exist_ok=True)

    w, h = size
    files = sorted(glob.glob(os.path.join(args.input_dir, '*.bin')))
    num = 1
    for p in files:
        prefix = os.path.splitext(os.path.basename(p))[0]
        left_bgr, right_bgr = read_nv12_vstack(p, w, h)
        rectify_and_save(left_bgr, right_bgr, maps_left, maps_right, prefix,
                         stereo_dir, cam0_dir, cam1_dir, cam_combine_dir,
                         num)
        num = num + 1

    print('[DONE]')


if __name__ == '__main__':
    main()
