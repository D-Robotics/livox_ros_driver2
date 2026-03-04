#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
stereo_nv12_rectify.py
fov_scale 从 calib.yaml 读取
"""

import os
import glob
import argparse
import numpy as np
import cv2
import yaml
import shutil
import math
from tqdm import tqdm
import json
import struct
from datetime import datetime, timezone
from pathlib import Path

def get_isoformat_time_by_nanosecond(timestamp):
    return datetime.fromtimestamp(timestamp / 1_000_000_000).isoformat(sep='_').replace(':', '-')

def get_isoformat_T_time_by_nanosecond(timestamp):
    return datetime.fromtimestamp(timestamp / 1_000_000_000).isoformat(sep='T')

def get_fov(target_width, target_height, camera_fx, camera_fy):
    h = 2 * math.atan2(target_width, 2 * camera_fx) * 180.0 / math.pi
    v = 2 * math.atan2(target_height, 2 * camera_fy) * 180.0 / math.pi
    d = 2 * math.atan2(
        math.sqrt(target_width * target_width + target_height * target_height),
        2 * math.sqrt(camera_fx * camera_fy)
    ) * 180.0 / math.pi
    return h, v, d

def load_yaml_calib(yaml_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    if "stereo0" in data:
        s = data['stereo0']
    else:
        s = data
    c0, c1 = s['cam0'], s['cam1']

    def parse_cam(cam):
        fx, fy, cx, cy = cam['intrinsics']
        w, h = cam['resolution']
        K = np.array([[fx, 0, cx],
                      [0, fy, cy],
                      [0, 0, 1]], dtype=np.float64)
        dist_model = cam.get('distortion_model', 'radtan').lower()
        d = np.array(cam.get('distortion_coeffs', []), dtype=np.float64).reshape(-1, )
        if dist_model in ('radtan', 'rational_polynomial', 'pinhole'):
            D = d
            model = 'pinhole'
        elif dist_model in ('equidistant', 'fisheye'):
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

    fov_scale = float(c1.get('fov_scale', 1.0))
    print(f"fov_scale: {fov_scale}")

    return K0, D0, K1, D1, size0, R, t, model0, fov_scale


def build_rectify_maps(K0, D0, K1, D1, size, target_image_size, R, t, model,
                       alpha=-1.0, balance=0.0, fov_scale=1.0, out_dir = './'):
    w, h = size
    image_size = (w, h)
    if model == 'fisheye':
        R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(
            K0, D0, K1, D1, image_size, R, t,
            flags=cv2.CALIB_ZERO_DISPARITY,
            newImageSize=target_image_size,
            balance=balance,
            fov_scale=fov_scale
        )

        map1x, map1y = cv2.fisheye.initUndistortRectifyMap(
            K0, D0, R1, P1, target_image_size, cv2.CV_32FC1
        )
        map2x, map2y = cv2.fisheye.initUndistortRectifyMap(
            K1, D1, R2, P2, target_image_size, cv2.CV_32FC1
        )
    else:
        R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
            K0, D0, K1, D1, image_size, R, t,
            flags=cv2.CALIB_ZERO_DISPARITY,
            newImageSize=target_image_size,
            alpha=float(alpha)
        )

        map1x, map1y = cv2.initUndistortRectifyMap(
            K0, D0, R1, P1, target_image_size, cv2.CV_32FC1
        )
        map2x, map2y = cv2.initUndistortRectifyMap(
            K1, D1, R2, P2, target_image_size, cv2.CV_32FC1
        )

    print("R1:\n", R1)
    print("R2:\n", R2)
    print("P1:\n", P1)
    print("P2:\n", P2)
    print("Q:\n", Q)

    camera_fx = Q[2, 3]
    camera_fy = Q[2, 3]
    camera_cx = -Q[0, 3]
    camera_cy = -Q[1, 3]
    base_line = abs(1 / Q[3, 2])
    h, v, d = get_fov(target_image_size[0], target_image_size[1], camera_fx, camera_fy)
    print(f'fov [h, v, d]: [{h}, {v}, {d}]')
    print('# fx fy cx cy baseline(m)')
    print(f'{camera_fx:.6f} {camera_fy:.6f} {camera_cx:.6f} {camera_cy:.6f} {base_line:.6f}')
    with open(out_dir + '/camera_intrinsic.txt', 'w', encoding='utf-8') as f:
        f.write('# fx fy cx cy baseline(m)\n')
        f.write(f'{camera_fx:.6f} {camera_fy:.6f} {camera_cx:.6f} {camera_cy:.6f} {base_line:.6f}')
    return (map1x, map1y), (map2x, map2y), (camera_fx, camera_fy, base_line)


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
                     stereo_dir, cam0_dir, cam1_dir, cam_combine_dir, num, enable_view):
    (map1x, map1y) = maps_left
    (map2x, map2y) = maps_right
    rect_l = cv2.remap(left_bgr, map1x, map1y, interpolation=cv2.INTER_AREA)
    rect_r = cv2.remap(right_bgr, map2x, map2y, interpolation=cv2.INTER_AREA)
    if enable_view:
        small = cv2.resize(np.hstack((left_bgr, right_bgr)), None, fx=0.5, fy=0.5,
                           interpolation=cv2.INTER_AREA)
        cv2.imshow("stereo", small)
        cv2.waitKey(10)
    else:
        save_bgr_json(rect_l, rect_r, out_prefix, stereo_dir, cam0_dir, cam1_dir, cam_combine_dir, num)


def save_bgr(left_bgr, right_bgr, out_prefix, stereo_dir, cam0_dir, cam1_dir, cam_combine_dir, num):
    combine = np.vstack((left_bgr, right_bgr))
    cv2.imwrite(os.path.join(stereo_dir, f'left{num:06d}.png'), left_bgr)
    cv2.imwrite(os.path.join(stereo_dir, f'right{num:06d}.png'), right_bgr)

    cv2.imwrite(os.path.join(cam0_dir, f'{out_prefix}.png'), left_bgr)
    cv2.imwrite(os.path.join(cam1_dir, f'{out_prefix}.png'), right_bgr)
    cv2.imwrite(os.path.join(cam_combine_dir, f'{out_prefix}.png'), combine)
    #print(f'[OK] {out_prefix}')

def save_bgr_json(left_bgr, right_bgr, out_prefix, stereo_dir, cam0_dir, cam1_dir, cam_combine_dir, num):
    combine = np.vstack((left_bgr, right_bgr))
    cv2.imwrite(os.path.join(cam_combine_dir,
                             get_isoformat_time_by_nanosecond(int(out_prefix)) + '.png'), combine)

def save_pcd(image_path, pcd_seq_dir, pcd_ts_dir, num):
    ts = os.path.splitext(os.path.basename(image_path))[0]
    parent_dir = os.path.dirname(os.path.dirname(image_path))
    pcd_dir = os.path.join(parent_dir, "pcd")
    pcd_file = os.path.join(pcd_dir, ts + ".pcd")

    if os.path.exists(pcd_file):
        dst_pcd_seq_file = os.path.join(pcd_seq_dir, f'pcd{num:06d}.pcd')
        dst_pcd_ts_file = os.path.join(pcd_ts_dir, ts + ".pcd")
        shutil.copy(pcd_file, dst_pcd_seq_file)
        shutil.copy(pcd_file, dst_pcd_ts_file)
        print(f'[OK] {pcd_file} {dst_pcd_seq_file} {dst_pcd_ts_file}')
        return True
    else:
        print(f'[FATAL] {pcd_file} is not exist!!')
        return False

def create_json_entry(timestamp, width, height, fx, fy, bl):
    entry = {
        "filename": get_isoformat_time_by_nanosecond(timestamp) + ".png",
        "width": width,
        "height": height,
        "capture_timestamp": get_isoformat_T_time_by_nanosecond(timestamp),
        "latitude": None,
        "longitude": None,
        "trigger_reason": "timer",
        "camera_mode": "day",
        "camera_parameters": {
            "fx": fx,
            "fy": fy,
            "base_line_m": bl
        }
    }
    return entry

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--yaml', required=False, help='calibration yaml path')
    ap.add_argument('--input_dir', required=True, help='folder with NV12 files')
    ap.add_argument('--out_dir', required=True, help='output folder for PNG')
    ap.add_argument('--target_size', required=False, help='target size of the image for rectification')
    ap.add_argument('--sync_with_pcd', required=False, help='if true, images have no corresponding pcd will be discarded')
    ap.add_argument('--enable_view', required=False, help='show stereo images')

    args = ap.parse_args()

    target_image_size = [1280, 1088]
    sync_with_pcd = False

    if args.target_size is not None:
        target_image_size = tuple(int(x) for x in args.target_size.split('x'))

    if args.sync_with_pcd is not None and args.sync_with_pcd is True:
        sync_with_pcd = True

    out_dir = args.out_dir
    stereo_dir = os.path.join(out_dir, 'stereo_seq/')
    cam0_dir = os.path.join(out_dir, 'stereo_ts/cam0/data/')
    cam1_dir = os.path.join(out_dir, 'stereo_ts/cam1/data/')
    cam_combine_dir = os.path.join(out_dir, 'stereo_ts/cam_combine/data/')

    enable_view = args.enable_view
    video = None
    os.makedirs(stereo_dir, exist_ok=True)
    os.makedirs(cam0_dir, exist_ok=True)
    os.makedirs(cam1_dir, exist_ok=True)
    os.makedirs(cam_combine_dir, exist_ok=True)

    pcd_seq_dir = os.path.join(out_dir, 'pcd_seq')
    pcd_ts_dir = os.path.join(out_dir, 'pcd_ts')
    os.makedirs(pcd_seq_dir, exist_ok=True)
    os.makedirs(pcd_ts_dir, exist_ok=True)

    num = 1

    files = sorted(glob.glob(os.path.join(args.input_dir, '**', '*.yuv'), recursive=True))
    failed_count = 0
    if args.yaml is not None:
        K0, D0, K1, D1, size, R, t, model, fov_scale = load_yaml_calib(args.yaml)
        maps_left, maps_right, (fx, fy, bl) = build_rectify_maps(K0, D0, K1, D1, size, target_image_size, R, t, model,
                                                   balance=0.0, fov_scale=fov_scale, out_dir=out_dir)
        w, h = size
        json_entries = []
        for p in tqdm(files, desc="Processing YUV files"):
            prefix = os.path.splitext(os.path.basename(p))[0]

            if sync_with_pcd and save_pcd(p, pcd_seq_dir, pcd_ts_dir, num) == False :
                continue
            try:
                left_bgr, right_bgr = read_nv12_vstack(p, w, h)
                rectify_and_save(left_bgr, right_bgr, maps_left, maps_right, prefix,
                                 stereo_dir, cam0_dir, cam1_dir, cam_combine_dir,
                                 num, enable_view)

                entry = create_json_entry(int(prefix), target_image_size[0], target_image_size[1] * 2, fx, fy, bl)
                json_entries.append(entry)
            except Exception as e:
                failed_count = failed_count + 1
                print(f"error: {type(e).__name__}: {e}, filename: {p}, failed count: {failed_count}")

            num = num + 1

        json_result = {
            "generated_timestamp": datetime.now().isoformat(),
            "data": json_entries
        }

        json_file = os.path.join(cam_combine_dir, '../metadata.json')
        with open(json_file, 'w', encoding='utf-8') as f:
            json.dump(json_result, f, indent=2, ensure_ascii=False)
            print("json has been save to: ", json_file)
    else:
        w, h = target_image_size
        for p in tqdm(files, desc="Processing YUV files"):
            prefix = os.path.splitext(os.path.basename(p))[0]
            #print("prefix:", prefix)

            if sync_with_pcd and save_pcd(p, pcd_seq_dir, pcd_ts_dir, num) == False :
                continue
            try:
                left_bgr, right_bgr = read_nv12_vstack(p, w, h)
                if enable_view:
                    small = cv2.resize(np.hstack((left_bgr, right_bgr)), None, fx=0.5, fy=0.5,
                                       interpolation=cv2.INTER_AREA)
                    cv2.imshow("stereo", small)
                    cv2.waitKey(10)
                    if video is None:
                        height, width, channels = small.shape
                        fourcc = cv2.VideoWriter_fourcc(*'mp4v') # Codec for .mp4
                        video = cv2.VideoWriter('output_video.mp4', fourcc, 10, (width, height))
                    video.write(small)
                else:
                    save_bgr(left_bgr, right_bgr, prefix,
                             stereo_dir, cam0_dir, cam1_dir, cam_combine_dir, num)
            except Exception as e:
                failed_count = failed_count + 1
                print(f"error: {type(e).__name__}: {e}, filename: {p}, failed count: {failed_count}")
            num = num + 1

    print('[DONE]')


if __name__ == '__main__':
    main()