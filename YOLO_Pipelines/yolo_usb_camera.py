#!/usr/bin/env python3

import cv2  # ✅ import OpenCV FIRST

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError  

from geometry_msgs.msg import PointStamped  

import numpy as np
from ultralytics import YOLO
import time

ENGINE_PATH = "YOLO_Models/yolo_usb.engine"

class YoloUsbNode:
    def __init__(self):
        rospy.init_node("yolo_usb_node")

        self.bridge = CvBridge()
        self.model = YOLO(ENGINE_PATH, task="detect")

        # Subscribe to your USB camera image
        self.sub = rospy.Subscriber(
            "/usb_cam/image_raw",
            Image,
            self.image_cb,
            queue_size=1,
            buff_size=2**24,
        )

        # Old topic: CSV detections (kept for compatibility)
        # Format: "class_id,x1,y1,x2,y2,conf"
        self.pub_2d = rospy.Publisher(
            "/yolo_usb/detections_2d",
            String,
            queue_size=10,
        )

        # NEW topic: geometry point with class in frame_id
        # point.x, point.y = bbox center (pixels), point.z = 0
        self.pub_center = rospy.Publisher(
            "/yolo_usb/center_point",
            PointStamped,
            queue_size=10,
        )

        self.show_window = True
        self.last_time = time.time()

        rospy.loginfo("[YOLO_USB] Node started, waiting for /usb_cam/image_raw")

    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"[YOLO_USB] CvBridge error: {e}")
            return

        h, w, _ = frame.shape

        t0 = time.perf_counter()
        results = self.model(frame, imgsz=640, device="cuda", verbose=False)
        t1 = time.perf_counter()

        r = results[0]
        annotated = frame.copy()

        if r.boxes is not None and len(r.boxes) > 0:
            boxes = r.boxes
            xyxy = boxes.xyxy.cpu().numpy()
            conf = boxes.conf.cpu().numpy()
            cls  = boxes.cls.cpu().numpy().astype(int)

            for i in range(len(xyxy)):
                x1, y1, x2, y2 = xyxy[i]
                class_id = int(cls[i])
                conf_i   = float(conf[i])

                # Draw bbox
                p1 = (int(x1), int(y1))
                p2 = (int(x2), int(y2))
                cv2.rectangle(annotated, p1, p2, (0, 255, 0), 2)

                # Label
                class_name = r.names[class_id] if r.names else str(class_id)
                label = f"{class_name} {conf_i:.2f}"
                text_org = (int(x1), max(0, int(y1) - 10))
                cv2.putText(
                    annotated,
                    label,
                    text_org,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA,
                )

                # === 1) Publish CSV: "class_id,x1,y1,x2,y2,conf" (unchanged) ===
                csv = f"{class_id},{x1:.1f},{y1:.1f},{x2:.1f},{y2:.1f},{conf_i:.2f}"
                self.pub_2d.publish(String(data=csv))

                # === 2) Publish geometry_msgs/PointStamped for bbox center ===
                cx = 0.5 * (x1 + x2)
                cy = 0.5 * (y1 + y2)

                pt_msg = PointStamped()
                pt_msg.header.stamp = rospy.Time.now()
                # Put class id as the "label" in frame_id (can also use class_name)
                pt_msg.header.frame_id = str(class_name)

                # Use pixel coordinates here; z = 0 because USB cam has no depth
                pt_msg.point.x = float(cx)
                pt_msg.point.y = float(cy)
                pt_msg.point.z = 0.0

                self.pub_center.publish(pt_msg)

        # Optional visualization
        if self.show_window:
            cv2.imshow("YOLO USB", annotated)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.show_window = False
                cv2.destroyWindow("YOLO USB")

        dt = (t1 - t0) * 1000
        fps = 1000.0 / max(dt, 1e-3)
        rospy.loginfo_throttle(1.0, f"[YOLO_USB] infer={dt:.1f} ms  FPS≈{fps:.1f}")


if __name__ == "__main__":
    node = YoloUsbNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyWindow("YOLO USB")
