import roslib; roslib.load_manifest('sensor_msgs')
import rosbag
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3

def main():
    input_txt = "/mnt/VirtualBoxShare/stereo_imu_calib/1_imu_calib/livox_imu_2.txt"
    output_bag = "/mnt/VirtualBoxShare/stereo_imu_calib/1_imu_calib/livox_imu_2_ros1.bag"

    bag = rosbag.Bag(output_bag, 'w')

    with open(input_txt, 'r') as f:
        next(f)
        for line in f:
            parts = line.strip().split(',')

            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.from_sec(float(parts[0]))
            print(imu_msg.header.stamp)

            imu_msg.orientation = Quaternion(
                x=float(parts[1]),
                y=float(parts[2]),
                z=float(parts[3]),
                w=float(parts[4]))

            imu_msg.angular_velocity = Vector3(
                x=float(parts[5]),
                y=float(parts[6]),
                z=float(parts[7]))

            gravity = 9.7887
            imu_msg.linear_acceleration = Vector3(
                x=float(parts[8]) * gravity,
                y=float(parts[9]) * gravity,
                z=float(parts[10]) * gravity)

            bag.write("/imu/data", imu_msg, imu_msg.header.stamp)

    bag.close()

if __name__ == '__main__':
    rospy.init_node('txt_to_bag')
    main()