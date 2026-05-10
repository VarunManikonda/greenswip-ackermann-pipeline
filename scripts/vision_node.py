#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import math


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        
        # HSV bounds for RED. Red wraps the Hue circle (0 and 180),
        # so we need TWO ranges and OR them. Tuned to be tight enough
        # to reject the orange capsule (Hue ~15-25).
        
        self.lower_red_1 = np.array([0,   140, 70])
        self.upper_red_1 = np.array([8,   255, 255])
        self.lower_red_2 = np.array([172, 140, 70])
        self.upper_red_2 = np.array([180, 255, 255])

        
        # Detection thresholds
        
        self.min_blob_area    = 400     # ignore noise smaller than this
        self.poly_epsilon_pct = 0.04    # 4% of contour perimeter
        self.box_vertex_count = 4       # boxes have 4 corners

        # ROS interfaces
        self.bridge = CvBridge()
        self.offset_pub  = self.create_publisher(Float32, '/target_offset',  10)
        self.visible_pub = self.create_publisher(Bool,    '/target_visible', 10)
        self.debug_pub   = self.create_publisher(Image,   '/vision/debug',   10)

        # Subscribe with best-effort QoS to match Gazebo's camera publisher
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        qos = QoSProfile(depth=10)
        # QoS: use default RELIABLE to match ros_gz_bridge

        self.create_subscription(Image, "/camera/image", self.image_cb, 10)

        self.get_logger().info('vision_node started — looking for red BOX.')

    
    def image_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge failed: {e}')
            return

        h, w, _ = frame.shape
        image_center_x = w // 2

        # --- Color filter: red mask ---
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self.lower_red_1, self.upper_red_1)
        mask2 = cv2.inRange(hsv, self.lower_red_2, self.upper_red_2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Cleanup: open removes specks, close fills small holes
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # --- Find contours ---
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # --- Filter by area + shape ---
        box_candidates = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_blob_area:
                continue

            # Approximate polygon, count vertices
            perim = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, self.poly_epsilon_pct * perim, True)
            n_vertices = len(approx)

            # Box has 4 corners
            if n_vertices == self.box_vertex_count:
                box_candidates.append((c, approx, area))

        # --- Choose largest box candidate as the target ---
        target_visible = False
        target_offset = float('nan')
        target_centroid = None
        target_area = 0
        target_approx = None

        if box_candidates:
            # Largest by area
            best = max(box_candidates, key=lambda x: x[2])
            best_contour, target_approx, target_area = best

            # Centroid via image moments
            M = cv2.moments(best_contour)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                target_centroid = (cx, cy)
                # Normalize: -1.0 (full left) to +1.0 (full right)
                target_offset = (cx - image_center_x) / float(image_center_x)
                target_visible = True

        # --- Publish ---
        offset_msg = Float32()
        offset_msg.data = target_offset
        self.offset_pub.publish(offset_msg)

        visible_msg = Bool()
        visible_msg.data = target_visible
        self.visible_pub.publish(visible_msg)

        self._publish_debug(frame, mask, target_centroid, target_offset,
                            target_area, target_approx, target_visible,
                            len(contours))

        if target_visible:
            self.get_logger().info(
                f'BOX detected — area={int(target_area)} '
                f'offset={target_offset:+.3f}',
                throttle_duration_sec=1.0)
        else:
            self.get_logger().info(
                f'no box — {len(contours)} red blobs, none with 4 corners',
                throttle_duration_sec=1.0)

    
    def _publish_debug(self, frame, mask, centroid, offset, area,
                       approx, visible, n_contours):
        """Annotate the frame for RViz / video demo."""
        debug = frame.copy()
        h, w, _ = debug.shape

        # Red mask overlay (semi-transparent)
        red_overlay = np.zeros_like(debug)
        red_overlay[:, :, 2] = mask
        debug = cv2.addWeighted(debug, 1.0, red_overlay, 0.4, 0)

        # Image-center vertical line
        cv2.line(debug, (w // 2, 0), (w // 2, h), (255, 255, 0), 1)

        # Draw the 4-vertex polygon outline if box detected
        if visible and approx is not None:
            cv2.drawContours(debug, [approx], 0, (0, 255, 0), 3)
            cx, cy = centroid
            cv2.circle(debug, (cx, cy), 8, (0, 255, 255), -1)
            cv2.line(debug, (cx, cy), (w // 2, cy), (0, 255, 255), 2)

        # Status text
        if visible:
            label = f'BOX  off={offset:+.2f}  area={int(area)}'
            color = (0, 255, 0)
        else:
            label = f'NO BOX  {n_contours} red blobs'
            color = (0, 0, 255)
        cv2.putText(debug, label, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        try:
            out = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            self.debug_pub.publish(out)
        except Exception as e:
            self.get_logger().warn(f'debug publish failed: {e}')


def main():
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
