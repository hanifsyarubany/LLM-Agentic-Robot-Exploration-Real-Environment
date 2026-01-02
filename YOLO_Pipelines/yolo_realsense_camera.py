#!/usr/bin/env python3
import pyrealsense2 as rs
import numpy as np
import cv2
import time
from ultralytics import YOLO

import rospy
from geometry_msgs.msg import PointStamped          # XYZ + class_id
from std_msgs.msg import Int32                      # 0/1 flag
from sensor_msgs.msg import Image, CameraInfo       # images + camera_info
from cv_bridge import CvBridge                     # cv2 <-> ROS Image

PRINT_TIMING = True

ENGINE_PATH = "YOLO_Models/yolo_realsense.engine"


def intrinsics_to_camerainfo(intr, frame_id, stamp):
    """
    Convert pyrealsense2 intrinsics -> sensor_msgs/CameraInfo
    intr: rs.intrinsics
    """
    msg = CameraInfo()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id

    msg.width = intr.width
    msg.height = intr.height

    # RealSense uses Brown-Conrady; ROS typically uses "plumb_bob" for pinhole
    msg.distortion_model = "plumb_bob"
    # intr.coeffs is a list of 5 distortion coefficients
    msg.D = list(intr.coeffs)

    # Camera matrix K
    msg.K = [0.0] * 9
    msg.K[0] = intr.fx
    msg.K[2] = intr.ppx
    msg.K[4] = intr.fy
    msg.K[5] = intr.ppy
    msg.K[8] = 1.0

    # Rectification matrix R (identity, since we’re not rectifying here)
    msg.R = [1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0]

    # Projection matrix P (no stereo baseline)
    msg.P = [0.0] * 12
    msg.P[0] = intr.fx
    msg.P[2] = intr.ppx
    msg.P[5] = intr.fy
    msg.P[6] = intr.ppy
    msg.P[10] = 1.0

    return msg


def main():
    rospy.init_node("yolo_rs_depth_node")

    bridge = CvBridge()

    # === Publishers ===
    depth_pub = rospy.Publisher(
        "/yolo_rs/depth_xyz",
        PointStamped,
        queue_size=10
    )

    flag_pub = rospy.Publisher(
        "/yolo_rs/object_detected",
        Int32,
        queue_size=10
    )

    # Camera image + camera_info publishers
    color_image_pub = rospy.Publisher(
        "/camera/color/image_raw",
        Image,
        queue_size=1
    )
    depth_image_pub = rospy.Publisher(
        "/camera/depth/image_rect_raw",
        Image,
        queue_size=1
    )
    color_info_pub = rospy.Publisher(
        "/camera/color/camera_info",
        CameraInfo,
        queue_size=10
    )
    depth_info_pub = rospy.Publisher(
        "/camera/depth/camera_info",
        CameraInfo,
        queue_size=10
    )

    # Annotated YOLO image
    annotated_pub = rospy.Publisher(
        "/yolo_rs/image_annotated",
        Image,
        queue_size=1
    )

    # ==== 1. RealSense setup ====
    pipeline = rs.pipeline()
    config = rs.config()

    # Enable BOTH depth and color, same resolution for easy alignment
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    rospy.loginfo("[YOLO_RS] Starting RealSense pipeline...")
    profile = pipeline.start(config)
    rospy.loginfo("[YOLO_RS] RealSense started.")

    # Align depth to color
    align_to_color = rs.align(rs.stream.color)

    # Get depth scale (to convert raw depth units to meters)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    rospy.loginfo(f"[YOLO_RS] Depth scale: {depth_scale}")

    # Get static intrinsics once (from the original streams)
    depth_stream = profile.get_stream(rs.stream.depth).as_video_stream_profile()
    color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
    depth_intrinsics_initial = depth_stream.get_intrinsics()
    color_intrinsics = color_stream.get_intrinsics()
    rospy.loginfo(f"[YOLO_RS] Depth intrinsics: {depth_intrinsics_initial}")
    rospy.loginfo(f"[YOLO_RS] Color intrinsics: {color_intrinsics}")

    # ==== 2. Load TensorRT YOLO engine ====
    rospy.loginfo("[YOLO_RS] Loading YOLO TensorRT engine...")
    model = YOLO(ENGINE_PATH, task="detect")
    rospy.loginfo("[YOLO_RS] YOLO engine loaded.")

    # ==== 3. Warmup (optional) ====
    rospy.loginfo("[YOLO_RS] Warming up model...")
    for i in range(3):
        frames = pipeline.wait_for_frames()
        aligned = align_to_color.process(frames)
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()
        if not depth_frame or not color_frame:
            continue
        frame = np.asanyarray(color_frame.get_data())
        # warmup with normal predict is fine
        _ = model(frame, imgsz=640, device="cuda", verbose=False)
        rospy.loginfo(f"[YOLO_RS] Warmup step {i+1}/3 done")
    rospy.loginfo("[YOLO_RS] Warmup done.")

    try:
        while not rospy.is_shutdown():
            t0 = time.perf_counter()

            # Get frames and align depth->color
            frames = pipeline.wait_for_frames()
            aligned = align_to_color.process(frames)
            depth_frame = aligned.get_depth_frame()
            color_frame = aligned.get_color_frame()
            if not depth_frame or not color_frame:
                continue
            t1 = time.perf_counter()

            # Convert to numpy
            color_image = np.asanyarray(color_frame.get_data())   # BGR
            depth_image = np.asanyarray(depth_frame.get_data())   # uint16 (z16)

            h, w, _ = color_image.shape

            # Get intrinsics for THIS aligned depth frame
            depth_profile = depth_frame.get_profile().as_video_stream_profile()
            depth_intr = depth_profile.get_intrinsics()

            # Shared timestamp for all topics this frame
            stamp = rospy.Time.now()

            # === Publish raw color image ===
            color_msg = bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
            color_msg.header.stamp = stamp
            color_msg.header.frame_id = "camera_link"
            color_image_pub.publish(color_msg)

            # === Publish raw aligned depth image ===
            depth_msg = bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")
            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = "camera_link"
            depth_image_pub.publish(depth_msg)

            # === Publish camera_info ===
            color_info = intrinsics_to_camerainfo(
                color_intrinsics,
                frame_id="camera_link",
                stamp=stamp,
            )
            depth_info = intrinsics_to_camerainfo(
                depth_intr,
                frame_id="camera_link",
                stamp=stamp,
            )
            color_info_pub.publish(color_info)
            depth_info_pub.publish(depth_info)

            # ==== YOLO inference (SWITCHED TO TRACKING) ====
            t2 = time.perf_counter()

            # === NEW: use tracking API ===
            results = model.track(
                source=color_image,
                imgsz=640,
                device="cuda",
                verbose=False,
                persist=True,      # keep tracks across frames
                show=False         # no OpenCV window from Ultralytics
                # tracker="bytetrack.yaml"  # optional, default is fine
            )

            t3 = time.perf_counter()

            r = results[0]
            annotated = color_image.copy()

            # Flag for this frame
            detection_present = False

            # ==== For each detection, get depth + 3D point ====
            if r.boxes is not None and len(r.boxes) > 0:
                detection_present = True   # at least one YOLO detection

                boxes = r.boxes
                xyxy = boxes.xyxy.cpu().numpy()
                conf = boxes.conf.cpu().numpy()
                cls  = boxes.cls.cpu().numpy().astype(int)

                # === NEW: track IDs (may be None) ===
                if boxes.id is not None:
                    track_ids = boxes.id.cpu().numpy().astype(int)
                else:
                    track_ids = [-1] * len(xyxy)

                for i in range(len(xyxy)):
                    x1, y1, x2, y2 = xyxy[i]
                    class_id = int(cls[i])
                    conf_i   = float(conf[i])
                    track_id = int(track_ids[i]) if track_ids[i] is not None else -1

                    # center of bbox
                    cx = int((x1 + x2) / 2.0)
                    cy = int((y1 + y2) / 2.0)

                    # Clamp to image bounds
                    cx = max(0, min(w - 1, cx))
                    cy = max(0, min(h - 1, cy))

                    # ---- Robust depth: median in small patch (e.g. 5x5) ----
                    patch_radius = 3
                    x_min = max(0, cx - patch_radius)
                    x_max = min(w, cx + patch_radius + 1)
                    y_min = max(0, cy - patch_radius)
                    y_max = min(h, cy + patch_radius + 1)

                    depth_patch = depth_image[y_min:y_max, x_min:x_max].astype(np.float32)
                    depth_patch_m = depth_patch * depth_scale  # convert to meters

                    valid = depth_patch_m[depth_patch_m > 0]
                    if valid.size > 0:
                        depth_m = float(np.median(valid))
                    else:
                        depth_m = 0.0

                    # ---- 3D deprojection (X, Y, Z) in camera frame ----
                    if depth_m > 0:
                        X, Y, Z = rs.rs2_deproject_pixel_to_point(
                            depth_intr,
                            [cx, cy],
                            depth_m,
                        )
                        depth_text = f"{depth_m:.2f}m"
                        xyz_text = f"X={X:.2f} Y={Y:.2f} Z={Z:.2f}"
                    else:
                        X = Y = Z = 0.0
                        depth_text = "?m"
                        xyz_text = "X=? Y=? Z=?"

                    class_name = r.names[class_id] if r.names else str(class_id)

                    # === NEW: include tracking id in label (bbox text) ===
                    if track_id >= 0:
                        label = f"{class_name}#{track_id} {conf_i:.2f} {depth_text}"
                        frame_label = f"{class_name}:{track_id}"
                    else:
                        label = f"{class_name} {conf_i:.2f} {depth_text}"
                        frame_label = class_name

                    # Draw bbox
                    p1 = (int(x1), int(y1))
                    p2 = (int(x2), int(y2))
                    cv2.rectangle(annotated, p1, p2, (0, 255, 0), 2)

                    # Draw label (with depth + track ID)
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

                    # Also draw XYZ below box if valid
                    if depth_m > 0:
                        text_org2 = (int(x1), min(h - 5, int(y2) + 15))
                        cv2.putText(
                            annotated,
                            xyz_text,
                            text_org2,
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 255),
                            1,
                            cv2.LINE_AA,
                        )

                        # === ROS PUBLISH: geometry_msgs/PointStamped ===
                        pt_msg = PointStamped()
                        pt_msg.header.stamp = stamp

                        # === NEW: frame_id = "<class_name>:<track_id>" ===
                        pt_msg.header.frame_id = frame_label

                        pt_msg.point.x = X
                        pt_msg.point.y = Y
                        pt_msg.point.z = Z  # forward distance

                        depth_pub.publish(pt_msg)

                        # Debug
                        rospy.loginfo_throttle(
                            1.0,
                            f"[YOLO_RS] {frame_label} XYZ=({X:.3f}, {Y:.3f}, {Z:.3f}) depth={depth_m:.3f}",
                        )
                    else:
                        rospy.loginfo_throttle(
                            1.0,
                            f"[YOLO_RS] {class_name}: no valid depth in patch near center",
                        )

            else:
                # No boxes at all from YOLO
                detection_present = False

            # === Publish the 0/1 flag once per frame ===
            flag_value = 1 if detection_present else 0
            flag_pub.publish(Int32(data=flag_value))

            # === Publish annotated image ===
            annotated_msg = bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            annotated_msg.header.stamp = stamp
            annotated_msg.header.frame_id = "camera_link"
            annotated_pub.publish(annotated_msg)

            # Timing
            grab_ms = (t1 - t0) * 1000
            infer_ms = (t3 - t2) * 1000
            total_ms = (t3 - t0) * 1000
            fps = 1000.0 / max(total_ms, 1e-3)

            if PRINT_TIMING:
                rospy.loginfo_throttle(
                    1.0,
                    f"[YOLO_RS] grab={grab_ms:.1f} ms  infer={infer_ms:.1f} ms  total={total_ms:.1f} ms  FPS≈{fps:.1f}",
                )

    finally:
        rospy.loginfo("[YOLO_RS] Stopping RealSense pipeline.")
        pipeline.stop()
        # No cv2.destroyWindow since we never opened one


if __name__ == "__main__":
    main()
