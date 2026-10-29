#!/usr/bin/env python3
"""Smoke test for data_collection_node output (legacy + kalibr_bagcreater formats).

Publishes a fake combined stereo NV12 image (top=left, bottom=right, both W x H_HALF)
plus IMU on the topics data_collection_node subscribes to, then validates the saved
run folder for the selected --mode:

  legacy        -> image/<ns>.png (whole frame, 2*H_HALF), imu/imu_data.txt (no header)
  kalibr_full   -> cam0/<ns>.png (whole frame), imu/imu0.csv (header + 7 cols)
  kalibr_stereo -> cam0/<ns>.png (top half) + cam1/<ns>.png (bottom half), imu/imu0.csv

Validation replicates kalibr_bagcreater's folder scan and timestamp parsing.

Start the node first, e.g. (add -p kalibr_format:=True / stereo_split:=False per mode):
  ros2 run livox_ros_driver2 data_collection_node --ros-args \
    -p output_dir:=/tmp/dc_smoke -p imu_topic:=/fake_imu -p lidar_topic:=/fake_lidar \
    -p image_topic:=/fake_img -p motion_detect:=false -p snap_shot:=false \
    -p enable_pause:=false -p check_camera_sync:=false -p check_lidar_exist:=false \
    -p check_ext_driver:=false -p image_format:=png

  python3 smoke_test_data_collection.py --output-dir /tmp/dc_smoke --mode kalibr_stereo
"""
import argparse
import glob
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu

W = 64
H_HALF = 48  # per-eye height; published combined frame is W x 2*H_HALF
N_IMG = 20
N_IMU = 300
IMG_HZ = 10.0
IMU_HZ = 100.0
GRAVITY = 9.81  # data_collection_node default gravity param
TOP_Y = 60   # top (left eye) luma level
BOT_Y = 200  # bottom (right eye) luma level


def make_combined_nv12(seq):
    """Combined NV12: Y plane = [top H_HALF rows | bottom H_HALF rows], then UV planes."""
    top = np.full(W * H_HALF, (TOP_Y + seq * 5) % 255, dtype=np.uint8)
    bot = np.full(W * H_HALF, (BOT_Y + seq * 5) % 255, dtype=np.uint8)
    uv = np.full(W * H_HALF, 128, dtype=np.uint8)  # neutral chroma for both halves
    return np.concatenate([top, bot, uv]).tobytes()


def nv12_bytes_to_gray(data, w, h):
    """Reference decode mirroring the node's cv::cvtColor(COLOR_YUV2BGR_NV12) -> gray."""
    import cv2
    nv12 = np.frombuffer(data, dtype=np.uint8).reshape(h * 3 // 2, w)
    bgr = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)


class Pub(Node):
    def __init__(self, imu_topic, img_topic):
        super().__init__("dc_smoke_pub")
        self.img_pub = self.create_publisher(Image, img_topic, 10)
        self.imu_pub = self.create_publisher(Imu, imu_topic, 10)
        self.img_ts = []
        self.imu_ts = []

    @staticmethod
    def stamp_ns(t):
        sec = int(t)
        nsec = int((t - sec) * 1e9)
        return sec, nsec, sec * 1_000_000_000 + nsec

    def run(self):
        t0 = time.time()
        next_img, next_imu = t0, t0
        i_img = i_imu = 0
        while i_img < N_IMG or i_imu < N_IMU:
            now = time.time()
            if i_img < N_IMG and now >= next_img:
                sec, nsec, ns = self.stamp_ns(next_img)
                msg = Image()
                msg.header.stamp.sec = sec
                msg.header.stamp.nanosec = nsec
                msg.height, msg.width, msg.encoding = 2 * H_HALF, W, "nv12"
                msg.step = W
                msg.data = bytes(make_combined_nv12(i_img))
                self.img_pub.publish(msg)
                self.img_ts.append(ns)
                i_img += 1
                next_img += 1.0 / IMG_HZ
            if i_imu < N_IMU and now >= next_imu:
                sec, nsec, ns = self.stamp_ns(next_imu)
                m = Imu()
                m.header.stamp.sec = sec
                m.header.stamp.nanosec = nsec
                m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = 0.1, 0.2, 0.3
                # driver stores accel in g units; node multiplies by gravity on save
                m.linear_acceleration.x, m.linear_acceleration.y = 0.01, 0.02
                m.linear_acceleration.z = 1.0
                self.imu_pub.publish(m)
                self.imu_ts.append(ns)
                i_imu += 1
                next_imu += 1.0 / IMU_HZ
            time.sleep(0.001)
        return self.img_ts, self.imu_ts


def ts_pairs_ok(saved_ts, pub_ts):
    # node computes sec*1e9 in double -> ns rounded to ~256ns granularity; tolerate it
    if len(saved_ts) != len(pub_ts):
        raise AssertionError(f"{len(saved_ts)} saved vs {len(pub_ts)} published")
    for s, p in zip(sorted(saved_ts), sorted(pub_ts)):
        assert abs(s - p) <= 512, f"ts drift {s - p} ns"


def check_images(run, folder, ref_gray_fn, img_ts, label):
    """ref_gray_fn(seq) -> expected gray Mat; seq is the publish index (ts-sorted order)."""
    import cv2
    pngs = sorted(glob.glob(os.path.join(run, folder, "*.png")))
    assert pngs, f"no png in {folder}/"
    saved_ts = []
    for seq, p in enumerate(pngs):  # ts-sorted == publish order
        base = os.path.splitext(os.path.basename(p))[0]
        assert base.isdigit() and len(base) >= 10, f"bad ts filename {p}"
        int(base[0:-9]); int(base[-9:])  # bagcreater's rospy.Time split
        im = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        ref = ref_gray_fn(seq)
        assert im is not None and im.shape == ref.shape, f"{p} shape mismatch"
        diff = int(np.abs(im.astype(int) - ref.astype(int)).max())
        assert diff <= 2, f"{p} content differs from reference (max diff {diff}, {label})"
        saved_ts.append(int(base))
    ts_pairs_ok(saved_ts, img_ts)
    print(f"{folder} OK: {len(pngs)} png {pngs[0] and ref.shape}, {label}, content pixel-match")
    return set(saved_ts)


def check_imu_csv(path, imu_ts, header):
    with open(path) as f:
        lines = [l.strip() for l in f if l.strip()]
    if header:
        assert lines[0].startswith("timestamp"), f"bad header: {lines[0]}"
        rows = [l.split(",") for l in lines[1:]]
    else:
        rows = [l.split(",") for l in lines]  # legacy: first line is data
    assert all(len(r) == 7 for r in rows), "row without 7 cols"
    ts_pairs_ok([int(r[0]) for r in rows], imu_ts)
    r0 = rows[0]
    assert abs(float(r0[1]) - 0.1) < 1e-6 and abs(float(r0[3]) - 0.3) < 1e-6, r0
    assert abs(float(r0[6]) - 1.0 * GRAVITY) < 1e-2, r0
    print(f"{os.path.basename(path)} OK: {len(rows)} rows, gyro rad/s, accel m/s2")


def bagcreater_scan(run):
    """Replicate kalibr_bagcreater's folder discovery."""
    cam_folders, imu_files = [], []
    for path, folders, files in os.walk(run):
        for folder in folders:
            if folder[0:3] == "cam":
                cam_folders.append(folder)
        for f in files:
            if f[0:3] == "imu" and os.path.splitext(f)[1] == ".csv":
                imu_files.append(f)
    return sorted(cam_folders), sorted(imu_files)


def validate(out_dir, mode, img_ts, imu_ts):
    runs = sorted(glob.glob(os.path.join(out_dir, "ros2_data", "*")))
    assert runs, f"no run dir under {out_dir}/ros2_data/"
    run = runs[-1]
    print("run dir:", run)

    if mode == "legacy":
        assert not os.path.isdir(os.path.join(run, "cam0")), "cam0/ must not exist in legacy mode"
        assert not glob.glob(os.path.join(run, "**", "imu*.csv"), recursive=True), \
            "no imu*.csv expected in legacy mode"
        check_images(run, "image",
                     lambda seq: nv12_bytes_to_gray(make_combined_nv12(seq), W, 2 * H_HALF),
                     img_ts, "whole frame")
        check_imu_csv(os.path.join(run, "imu", "imu_data.txt"), imu_ts, header=False)
        cams, imus = bagcreater_scan(run)
        assert not cams and not imus, f"bagcreater should find nothing: {cams} {imus}"
        print("legacy layout OK: image/ + imu_data.txt (no header), bagcreater-invisible")
    else:
        cams, imus = bagcreater_scan(run)
        assert imus == ["imu0.csv"], f"bagcreater imu scan: {imus}"
        imu_csv = os.path.join(run, "imu0.csv")
        assert os.path.isfile(imu_csv), "imu0.csv must sit at run-dir root (kalibr dataset structure)"
        check_imu_csv(imu_csv, imu_ts, header=True)
        if mode == "kalibr_full":
            assert cams == ["cam0"], f"cam folders: {cams}"
            check_images(run, "cam0/data",
                         lambda seq: nv12_bytes_to_gray(make_combined_nv12(seq), W, 2 * H_HALF),
                         img_ts, "whole frame")
            print("bagcreater scan OK -> /cam0/image_raw + /imu0 (whole frame)")
        else:  # kalibr_stereo
            assert cams == ["cam0", "cam1"], f"cam folders: {cams}"

            def half(seq, top):
                g = nv12_bytes_to_gray(make_combined_nv12(seq), W, 2 * H_HALF)
                return g[:H_HALF] if top else g[H_HALF:]

            ts0 = check_images(run, "cam0/data", lambda s: half(s, True), img_ts, "top/left half")
            ts1 = check_images(run, "cam1/data", lambda s: half(s, False), img_ts, "bottom/right half")
            assert ts0 == ts1, "cam0/cam1 timestamp sets differ"
            print("bagcreater scan OK -> /cam0/image_raw + /cam1/image_raw + /imu0 (split stereo)")

    print("\nALL SMOKE CHECKS PASSED (%s)" % mode)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--output-dir", default="/tmp/dc_smoke")
    ap.add_argument("--imu-topic", default="/fake_imu")
    ap.add_argument("--image-topic", default="/fake_img")
    ap.add_argument("--mode", choices=["legacy", "kalibr_full", "kalibr_stereo"],
                    default="kalibr_stereo")
    args = ap.parse_args()

    rclpy.init()
    node = Pub(args.imu_topic, args.image_topic)
    time.sleep(0.5)  # let subscriptions establish
    img_ts, imu_ts = node.run()
    print(f"published {len(img_ts)} img, {len(imu_ts)} imu; waiting for saver flush...")
    time.sleep(3)
    node.destroy_node()
    rclpy.shutdown()
    validate(args.output_dir, args.mode, img_ts, imu_ts)


if __name__ == "__main__":
    sys.exit(main())
