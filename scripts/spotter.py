import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import math
import time
import os
import json
from ultralytics import YOLO

# ==========================================
# ⚙️ WAREHOUSE GEOMETRY TUNING PARAMETERS
# ==========================================
MIN_STOP_DIST = 0.30          # Hardware blind spot (meters)
MAX_FORWARD_VISION = 2.23     # Max distance (m) before FOV hits the shelves
MAX_LATERAL_VISION = 1.50     # Max width (m) to the left/right of the robot

CONFIDENCE_THRESHOLD = 0.60   # YOLO confidence required

STOP_DURATION_SEC = 5.0      # How long to freeze for a picture
CAMERA_WIDTH_PIXELS = 640     # Orbbec Dabai resolution width
H_FOV_DEGREES = 67.9          # Orbbec Dabai Horizontal FOV
# ==========================================

class PatrolSpotter(Node):
    def __init__(self):
        super().__init__('patrol_spotter_node')
        
        self.bridge = CvBridge()
        self.state = "NORMAL"
        self.known_anomalies = []
        self.last_vision_check = 0.0
        self.latest_boxes = None # <--- Added for Smooth UI Streaming
        
        # 📁 Create folder for web dashboard images
        self.image_save_path = '/home/agilex/webpage_ws/anomalies_log/'
        if not os.path.exists(self.image_save_path):
            os.makedirs(self.image_save_path)
            self.get_logger().info(f"📁 Created directory for web images: {self.image_save_path}")
        
        self.get_logger().info("Loading YOLOv8 Nano AI Model...")
        self.yolo_model = YOLO('yolov8n.pt')
        
        self.current_pose = None
        self.global_map = None
        self.latest_depth_image = None
        
        # --- SUBSCRIBERS ---
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        
        self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/color/image_raw', self.color_callback, qos_profile_sensor_data)
        
        # --- PUBLISHERS ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
        self.marker_pub = self.create_publisher(Marker, '/spotter/anomaly_markers', 10)
        self.annotated_img_pub = self.create_publisher(CompressedImage, '/spotter/annotated_image/compressed', 10)
        self.alert_pub = self.create_publisher(String, '/spotter/alerts', 10)
        
        self.get_logger().info("🟢 Spotter AI Online. Broadcasting to Dashboard...")

    def map_callback(self, msg):
        if self.global_map is None:
            self.get_logger().info("🗺️ Received Global Map.")
        self.global_map = msg

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def depth_callback(self, msg):
        self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def color_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        current_time = time.time()
        
        # 1. RUN HEAVY AI AT 2 FPS
        if (current_time - self.last_vision_check) >= 0.5:
            self.last_vision_check = current_time
            
            # Save the raw bounding box data, NOT the static image
            results = self.yolo_model(cv_image, verbose=False)[0]
            self.latest_boxes = results.boxes 
            
            # --- NAVIGATION STATE CHECK ---
            if self.global_map is not None and self.latest_depth_image is not None and self.state == "NORMAL" and self.current_pose is not None:
                for box in results.boxes:
                    conf = float(box.conf[0])
                    class_id = int(box.cls[0])
                    label = self.yolo_model.names[class_id]
                    
                    if conf > CONFIDENCE_THRESHOLD:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        center_x = int((x1 + x2) / 2)
                        center_y = int((y1 + y2) / 2)
                        
                        try:
                            distance_mm = self.latest_depth_image[center_y, center_x]
                            distance_m = distance_mm / 1000.0
                        except IndexError:
                            continue 
                        
                        if MIN_STOP_DIST < distance_m < MAX_FORWARD_VISION:
                            degrees_per_pixel = H_FOV_DEGREES / CAMERA_WIDTH_PIXELS
                            angle_offset_deg = (center_x - (CAMERA_WIDTH_PIXELS / 2)) * degrees_per_pixel
                            angle_offset_rad = math.radians(angle_offset_deg)
                            
                            lateral_dist = distance_m * math.tan(angle_offset_rad)
                            
                            if abs(lateral_dist) < MAX_LATERAL_VISION:
                                # Create a temporary annotated frame just to save to the hard drive log
                                save_frame = results.plot()
                                self.verify_with_map(distance_m, angle_offset_rad, save_frame, label)
                                break 
            elif self.current_pose is None:
                self.get_logger().info("⚠️ AI Vision is streaming, but Braking logic is paused waiting for AMCL Pose.", throttle_duration_sec=2.0)

        # 2. DRAW BOXES ON LIVE 30FPS VIDEO
        # This keeps the video perfectly smooth while the AI runs in the background
        if self.latest_boxes is not None:
            for box in self.latest_boxes:
                if float(box.conf[0]) > CONFIDENCE_THRESHOLD:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    label = self.yolo_model.names[int(box.cls[0])]
                    # Draw Red Box
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    # Draw Label
                    cv2.putText(cv_image, f"{label.upper()} {float(box.conf[0]):.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Compress and publish the smooth frame
        comp_msg = CompressedImage()
        comp_msg.header.stamp = self.get_clock().now().to_msg()
        comp_msg.format = "jpeg"
        comp_msg.data = cv2.imencode('.jpg', cv_image)[1].tobytes()
        self.annotated_img_pub.publish(comp_msg)

    def verify_with_map(self, distance_m, angle_offset_rad, annotated_frame, label):
        # --- GATE 4: THE MAP MEMORY CHECK ---
        self.get_logger().info("🗺️ [GATE 4] Cross-referencing coordinates with Global Base Map...")
        
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        
        q = self.current_pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        target_x = robot_x + (distance_m * math.cos(robot_yaw - angle_offset_rad))
        target_y = robot_y + (distance_m * math.sin(robot_yaw - angle_offset_rad))
        
        for (kx, ky) in self.known_anomalies:
            if math.sqrt((target_x - kx)**2 + (target_y - ky)**2) < 0.5:
                self.get_logger().info("❌ [GATE 4 FAIL] We already recorded this obstacle. Ignoring.")
                return
                
        res = self.global_map.info.resolution
        orig_x = self.global_map.info.origin.position.x
        orig_y = self.global_map.info.origin.position.y
        
        px = int((target_x - orig_x) / res)
        py = int((target_y - orig_y) / res)
        
        try:
            map_value = self.global_map.data[py * self.global_map.info.width + px]
            if map_value < 50: 
                self.get_logger().info("✅ [GATE 4 PASS] Map confirms this spot should be empty!")
                self.get_logger().info(f"🚨 BRAKES ENGAGED: NEW OBSTACLE AT {distance_m:.2f}m!")
                
                self.known_anomalies.append((target_x, target_y))
                self.drop_map_pin(target_x, target_y, len(self.known_anomalies))
                self.trigger_anomaly_sequence(annotated_frame, label, distance_m)
            else:
                self.get_logger().info("❌ [GATE 4 FAIL] Map says there is a known wall/pillar here. Ignoring.")
        except IndexError:
            self.get_logger().info("❌ [GATE 4 FAIL] Coordinates are off the edge of the known map.")

    def drop_map_pin(self, x, y, anomaly_id):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.ns = "anomaly_pins"
        marker.id = anomaly_id
        marker.type = Marker.SPHERE 
        marker.action = Marker.ADD
        
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = 0.2 
        
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        
        marker.color.r = 1.0
        marker.color.g = 0.1
        marker.color.b = 0.1
        marker.color.a = 1.0 
        
        self.get_logger().info(f"📍 Broadcasting Red Pin to RViz at X:{x:.2f}, Y:{y:.2f}")
        self.marker_pub.publish(marker)

    def trigger_anomaly_sequence(self, annotated_frame, label, distance_m):
        self.state = "STOPPED"
        self.stop_timer = self.create_timer(0.1, self.publish_stop)
        
        timestamp = int(time.time())
        img_filename = f"anomaly_{timestamp}.jpg"
        filepath = f"{self.image_save_path}{img_filename}"
        
        cv2.imwrite(filepath, annotated_frame)
        self.get_logger().info(f"📸 Incident saved to {filepath}. Freezing for 10 seconds...")
        
        # Grab the exact coordinates we just calculated to send to the web dashboard
        last_x, last_y = 0.0, 0.0
        if self.known_anomalies:
            last_x, last_y = self.known_anomalies[-1]
            
        alert_data = {
            "type": "ANOMALY",
            "label": label.upper(),
            "distance": round(distance_m, 2),
            "image": img_filename,
            "x": round(last_x, 2), # Now passing X
            "y": round(last_y, 2)  # Now passing Y
        }
        alert_msg = String()
        alert_msg.data = json.dumps(alert_data)
        self.alert_pub.publish(alert_msg)
        
        self.wake_timer = self.create_timer(STOP_DURATION_SEC, self.wake_up_sequence)

    def publish_stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)

    def wake_up_sequence(self):
        self.stop_timer.cancel()
        self.wake_timer.cancel()
        self.state = "BLIND_DRIVE"
        self.get_logger().info("👁️ Nav2 restored. Driving blind for 5 seconds to pass obstacle...")
        self.blind_timer = self.create_timer(5.0, self.return_to_normal)
        
    def return_to_normal(self):
        self.blind_timer.cancel()
        self.state = "NORMAL"
        self.get_logger().info("🟢 Spotter fully armed and monitoring.")

def main(args=None):
    rclpy.init(args=args)
    node = PatrolSpotter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
