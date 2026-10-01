#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import os
from sensor_msgs.msg import LaserScan

class LidarCollector:
    def __init__(self):
        rospy.init_node('lidar_collector')
        self.label = rospy.get_param("~label", "").strip()
        if not self.label:
            self.label = input("👉 請輸入要收集的地形類別 (例如: corridor, corner, room): ").strip()
        default_dataset_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "dataset")
        dataset_dir = rospy.get_param("~dataset_dir", default_dataset_dir)
        self.save_dir = os.path.join(dataset_dir, self.label)
        os.makedirs(self.save_dir, exist_ok=True)
        self.count = 0
        self.img_size = 128
        self.max_dist = 4.0

        rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        rospy.loginfo(f"🚀 開始收集 {self.label} 的資料！請遙控車子移動...")

    def scan_cb(self, msg):
        if self.count >= 100:
            rospy.loginfo(f"✅ {self.label} 收集完成 100 張！請按 Ctrl+C 結束。")
            rospy.signal_shutdown("Done")
            return

        img = np.zeros((self.img_size, self.img_size), dtype=np.uint8)
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        valid = (ranges > msg.range_min) & (ranges < msg.range_max) & np.isfinite(ranges)
        angles = angles[valid]
        ranges = ranges[valid]

        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)

        pxs = (xs / self.max_dist * (self.img_size/2) + self.img_size/2).astype(int)
        pys = (ys / self.max_dist * (self.img_size/2) + self.img_size/2).astype(int)

        in_bounds = (pxs >= 0) & (pxs < self.img_size) & (pys >= 0) & (pys < self.img_size)
        pxs = pxs[in_bounds]
        pys = pys[in_bounds]

        for px, py in zip(pxs, pys):
            cv2.circle(img, (px, py), 2, 255, -1)

        filename = os.path.join(self.save_dir, f"{self.count:03d}.png")
        cv2.imwrite(filename, img)
        self.count += 1
        rospy.sleep(0.3)

if __name__ == '__main__':
    try:
        LidarCollector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
