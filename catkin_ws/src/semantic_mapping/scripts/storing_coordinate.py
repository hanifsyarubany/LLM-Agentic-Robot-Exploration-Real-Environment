#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import time
import math
from collections import Counter

import rospy
import rospkg

from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Int32


class CoordinateStorer(object):
    def __init__(self):
        # --------------------------------------------------------------
        # Config
        # --------------------------------------------------------------
        self.poi_labels = {"cafe", "convenience_store",
                           "hamburger_store", "pharmacy"}

        # Topics (can be overridden via params if needed)
        self.pose_topic = rospy.get_param("~pose_topic",
                                          "/orb_slam3/pose_corrected")
        self.yolo_topic = rospy.get_param("~yolo_topic", "/yolo_rs/xy")
        self.trigger_topic = rospy.get_param("~trigger_topic",
                                             "/storing_coordinate")

        # Sampling settings
        self.sample_duration = float(rospy.get_param("~sample_duration", 1.0))
        self.max_radius = float(rospy.get_param("~max_radius", 0.6))

        # Last robot pose (x, y)
        self.last_pose_xy = None  # (x, y)

        # Static coordinate points loaded from store_coordinates.json
        self.anchor_points = []   # list of {"x": float, "y": float}

        # DB for stored coordinates
        self.db = []
        self.coord_keys = set()   # to avoid duplicates by (x, y)
        self.db_file = None

        # Sampling state
        self.sampling_active = False
        self.sampling_start_time = None
        self.sampled_labels = []
        self.current_anchor = None  # {"x": .., "y": ..}

        # --------------------------------------------------------------
        # Init paths, load anchor points, reset DB
        # --------------------------------------------------------------
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("semantic_mapping")

        # Static coordinate file
        self.anchor_file = os.path.join(pkg_path, "maps", "store_coordinates.json")
        # Output database file
        self.db_file = os.path.join(pkg_path, "maps", "true_store_coordinates.json")

        self._load_anchor_points()
        self._reset_db()

        # --------------------------------------------------------------
        # Subscribers
        # --------------------------------------------------------------
        self.pose_sub = rospy.Subscriber(
            self.pose_topic, PoseStamped, self.pose_callback, queue_size=10
        )
        self.yolo_sub = rospy.Subscriber(
            self.yolo_topic, PointStamped, self.yolo_callback, queue_size=50
        )
        self.trigger_sub = rospy.Subscriber(
            self.trigger_topic, Int32, self.trigger_callback, queue_size=1
        )

        rospy.loginfo("[storing_coordinate] Node initialised.")
        rospy.loginfo("[storing_coordinate] Pose topic: %s", self.pose_topic)
        rospy.loginfo("[storing_coordinate] YOLO topic: %s", self.yolo_topic)
        rospy.loginfo("[storing_coordinate] Trigger topic: %s", self.trigger_topic)
        rospy.loginfo("[storing_coordinate] Anchor file: %s", self.anchor_file)
        rospy.loginfo("[storing_coordinate] DB file: %s", self.db_file)

    # ----------------------------------------------------------
    # File helpers
    # ----------------------------------------------------------
    def _load_anchor_points(self):
        """Load static coordinate points from store_coordinates.json."""
        if not os.path.exists(self.anchor_file):
            rospy.logerr("[storing_coordinate] Anchor file not found: %s",
                         self.anchor_file)
            self.anchor_points = []
            return

        try:
            with open(self.anchor_file, "r") as f:
                data = json.load(f)
            pts = data.get("points", [])
            self.anchor_points = [{"x": float(p["x"]), "y": float(p["y"])}
                                  for p in pts]
            rospy.loginfo("[storing_coordinate] Loaded %d anchor points.",
                          len(self.anchor_points))
        except Exception as e:
            rospy.logerr("[storing_coordinate] Failed to load anchor file: %s", str(e))
            self.anchor_points = []

    def _reset_db(self):
        """Reset stored_coordinates.json to an empty list at node startup."""
        self.db = []
        self.coord_keys = set()
        try:
            # Ensure directory exists
            db_dir = os.path.dirname(self.db_file)
            if not os.path.exists(db_dir):
                os.makedirs(db_dir)

            with open(self.db_file, "w") as f:
                json.dump(self.db, f, indent=2)
            rospy.loginfo("[storing_coordinate] DB reset: %s", self.db_file)
        except Exception as e:
            rospy.logerr("[storing_coordinate] Failed to reset DB '%s': %s",
                         self.db_file, str(e))

    def _save_db(self):
        try:
            with open(self.db_file, "w") as f:
                json.dump(self.db, f, indent=2)
        except Exception as e:
            rospy.logerr("[storing_coordinate] Failed to write DB: %s", str(e))

    # ----------------------------------------------------------
    # Callbacks
    # ----------------------------------------------------------
    def pose_callback(self, msg):
        """Keep track of the robot's current x, y."""
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.last_pose_xy = (x, y)

    def trigger_callback(self, msg):
        """
        Trigger logic when /storing_coordinate == 1.

        Steps:
          1. Use current robot pose (x,y).
          2. Find closest anchor point in store_coordinates.json.
          3. If distance > max_radius (0.6 m) -> do nothing.
          4. Else:
             - Remember that anchor.
             - Start 1-second sampling of /yolo_rs/xy to determine a POI label.
        """
        val = int(msg.data)
        if val != 1:
            # Only react on "1" as requested
            return

        if self.sampling_active:
            rospy.logwarn("[storing_coordinate] Trigger received but sampling already active, ignoring.")
            return

        if self.last_pose_xy is None:
            rospy.logwarn("[storing_coordinate] Trigger received but no robot pose yet.")
            return

        if not self.anchor_points:
            rospy.logwarn("[storing_coordinate] Trigger received but no anchor points loaded.")
            return

        # 1. Find closest anchor to current robot pose
        rx, ry = self.last_pose_xy
        best_pt = None
        best_dist = None

        for p in self.anchor_points:
            dx = rx - p["x"]
            dy = ry - p["y"]
            dist = math.hypot(dx, dy)
            if best_pt is None or dist < best_dist:
                best_pt = p
                best_dist = dist

        if best_pt is None:
            rospy.logwarn("[storing_coordinate] No anchor point found (unexpected).")
            return

        rospy.loginfo("[storing_coordinate] Closest anchor: (%.3f, %.3f), dist=%.3f",
                      best_pt["x"], best_pt["y"], best_dist)

        # 2. Check radius tolerance
        if best_dist > self.max_radius:
            rospy.loginfo(
                "[storing_coordinate] Distance %.3f > %.3f m. Not storing.",
                best_dist, self.max_radius,
            )
            return

        # 3. Start sampling labels from /yolo_rs/xy for 1 second
        self.current_anchor = {"x": best_pt["x"], "y": best_pt["y"]}
        self.sampled_labels = []
        self.sampling_active = True
        self.sampling_start_time = rospy.Time.now().to_sec()

        rospy.loginfo(
            "[storing_coordinate] Start sampling labels for 1s at anchor (%.3f, %.3f)",
            best_pt["x"], best_pt["y"],
        )

    def yolo_callback(self, msg):
        """
        While sampling is active:
          - collect POI labels from /yolo_rs/xy for sample_duration seconds
          - after that, finalize and store if valid
        """
        if not self.sampling_active:
            return

        now_t = msg.header.stamp.to_sec() if msg.header.stamp != rospy.Time() \
            else rospy.Time.now().to_sec()

        # Still within sampling window?
        if now_t - self.sampling_start_time <= self.sample_duration:
            label = msg.header.frame_id
            if label in self.poi_labels:
                self.sampled_labels.append(label)

        # If we have passed the sampling window, finalize
        if now_t - self.sampling_start_time >= self.sample_duration:
            self._finalize_sampling()

    # ----------------------------------------------------------
    # Sampling finalization & DB storing
    # ----------------------------------------------------------
    def _finalize_sampling(self):
        """Decide label from sampled_labels and store the coordinate if valid."""
        if not self.sampling_active:
            return

        self.sampling_active = False  # stop further sampling for this trigger

        if self.current_anchor is None:
            rospy.logwarn("[storing_coordinate] Sampling ended but no current anchor set.")
            return

        if not self.sampled_labels:
            rospy.loginfo(
                "[storing_coordinate] No POI labels detected in the 1s window. Not storing."
            )
            self.current_anchor = None
            return

        # Majority label among sampled_labels
        counts = Counter(self.sampled_labels)
        chosen_label, count = max(counts.items(), key=lambda x: x[1])

        rospy.loginfo(
            "[storing_coordinate] Sampling done. Chosen label='%s' (%d votes).",
            chosen_label, count,
        )

        ax = self.current_anchor["x"]
        ay = self.current_anchor["y"]

        # Duplicate removal based on coordinate (x, y)
        key = f"{ax:.6f}_{ay:.6f}"
        if key in self.coord_keys:
            rospy.loginfo(
                "[storing_coordinate] Coordinate (%.3f, %.3f) already in DB, skipping.",
                ax, ay,
            )
            self.current_anchor = None
            return

        # Build DB entry
        entry = {
            "x": ax,
            "y": ay,
            "label": chosen_label,
            "created_at_unix": time.time(),
        }

        self.db.append(entry)
        self.coord_keys.add(key)
        self._save_db()

        rospy.loginfo(
            "[storing_coordinate] Stored new coordinate: (%.3f, %.3f) -> %s",
            ax, ay, chosen_label,
        )

        # Clear state for next trigger
        self.current_anchor = None
        self.sampled_labels = []


def main():
    rospy.init_node("storing_coordinate_node")
    node = CoordinateStorer()
    rospy.spin()


if __name__ == "__main__":
    main()
