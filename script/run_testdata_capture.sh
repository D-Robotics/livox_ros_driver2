#!/bin/bash

mount -o rw,remount /
mount -o rw,remount /usr/hobot
resize2fs /dev/block/platform/by-name/system

export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/userdata/deps
source /opt/ros/humble/setup.bash
source /userdata/tros/install/setup.bash
source /userdata/install/setup.bash

IR_MODE=False
CALIB_FILE="/userdata/calib_3.yaml"
while getopts "i" opt; do
  case $opt in
    i)
      IR_MODE=True
      CALIB_FILE="/userdata/calib_3_ir.yaml"
      ;;
  esac
done

if [ "$IR_MODE" = "True" ]; then
  cp ox02c1s_tuning_ir_v1.1.json /usr/hobot/lib/sensor/ox02c1s_tuning.json
  echo "Data collection IR mode"
  cd /app/platform_samples/sunny_led
  chmod a+x ./illumination_test
  printf "2\n500000\n40000\n0\n" | ./illumination_test
  cd -
else
  cp ox02c1s_tuning_rgb_v0627.json /usr/hobot/lib/sensor/ox02c1s_tuning.json
  echo "Data collection RGB mode"
fi

bash run_husq.sh | tee camera_log.log > /dev/null &
bash run_lidar.sh | tee lidar_log.log > /dev/null &

ros2 launch livox_ros_driver2 data_collection.launch.py output_dir:="." \
check_ext_driver:=False save_pcd_bin:=True \
check_camera_sync:=False motion_detect:=True image_gap_mode:=2 log_level:=warn \
is_ir:=$IR_MODE snap_shot:=False \
motion_accel_th:=0.1 motion_gyro_th:=0.05 motion_imu_window_size:=120 \
pcd_static_collect_count:=115 img_static_collect_count:=1 \
calib_file:=$CALIB_FILE
