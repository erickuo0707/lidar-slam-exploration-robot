#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import json
import os
from sensor_msgs.msg import LaserScan

class AIObserver:
    def __init__(self):
        rospy.init_node('ai_observer')
        
        script_dir = os.path.dirname(os.path.abspath(__file__))
        brain_path = rospy.get_param("~model_path", os.path.join(script_dir, "lidar_brain.onnx"))
        classes_path = rospy.get_param("~classes_path", os.path.join(script_dir, "classes.json"))
        
        with open(classes_path, "r") as f:
            self.labels = json.load(f)
            
        self.net = cv2.dnn.readNetFromONNX(brain_path)
        self.img_size = int(rospy.get_param("~image_size", 128))
        self.max_dist = float(rospy.get_param("~max_distance", 4.0))
        inference_hz = float(rospy.get_param("~inference_hz", 0.5))
        self.max_load = float(rospy.get_param("~max_system_load", 2.5))
        if self.img_size <= 0 or self.max_dist <= 0 or inference_hz <= 0:
            raise ValueError("image_size、max_distance 和 inference_hz 必須大於 0")
        self.latest_scan = None
        
        rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.rate = rospy.Rate(inference_hz)
        rospy.loginfo(f"🧠 AI 觀察器上線！推論頻率 {inference_hz:g} Hz")

    def scan_cb(self, msg):
        self.latest_scan = msg

    def is_system_busy(self):
        load = os.getloadavg()[0]
        if self.max_load > 0 and load > self.max_load:
            rospy.logwarn(f"⚠️  系統負載過高 ({load:.1f})，跳過這幀推理，讓位給 SLAM")
            return True
        return False

    def start_inference_loop(self):
        while not rospy.is_shutdown():
            if self.latest_scan is None or self.is_system_busy():
                self.rate.sleep()
                continue
                
            msg = self.latest_scan
            
            img = np.zeros((self.img_size, self.img_size), dtype=np.uint8)
            num_pts = len(msg.ranges)
            angles = msg.angle_min + np.arange(num_pts) * msg.angle_increment
            ranges = np.array(msg.ranges)

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

            blob = cv2.dnn.blobFromImage(
                img, scalefactor=1.0/255.0,
                size=(self.img_size, self.img_size), mean=0,
                swapRB=False, crop=False)

            try:
                self.net.setInput(blob)
                outputs = self.net.forward()
            except cv2.error as e:
                rospy.logwarn(f"⚠️  推理失敗，跳過這幀: {e}")
                self.rate.sleep()
                continue
            
            scores = outputs.reshape(-1)
            if len(scores) != len(self.labels):
                rospy.logwarn_throttle(30, "模型輸出數量與 classes.json 標籤數不一致，跳過推論")
                self.rate.sleep()
                continue
            exp_scores = np.exp(scores - np.max(scores))
            probs = exp_scores / np.sum(exp_scores)
            best_idx = np.argmax(probs)
            confidence = probs[best_idx] * 100
            
            label_name = self.labels[best_idx]
            rospy.loginfo(f"👀 AI 副駕：前面地形是【{label_name}】 (信心度: {confidence:.1f}%)")
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        observer = AIObserver()
        observer.start_inference_loop()
    except rospy.ROSInterruptException:
        pass
