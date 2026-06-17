import time
import math
import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

class ArucoCommandFollower(Node):
    def __init__(self):
        super().__init__("aruco_command_follower")

        # Declare parameters to match launch file exactly
        self.declare_parameter("cooldown_seconds", 5.0)
        self.declare_parameter("linear_speed", 0.45)
        self.declare_parameter("reverse_speed", 0.35)
        self.declare_parameter("angular_speed", 0.75)
        self.declare_parameter("obstacle_stop_distance", 0.55)
        self.declare_parameter("show_camera", True)
        self.declare_parameter("search_for_markers", True)
        self.declare_parameter("search_angular_speed", 0.25)
        self.declare_parameter("initial_target_id", 2) # Start by heading towards Marker 2 (north wall)

        # Retrieve parameter values
        self.cooldown_seconds = self.get_parameter("cooldown_seconds").get_parameter_value().double_value
        self.show_camera = self.get_parameter("show_camera").get_parameter_value().bool_value
        self.search_angular_speed = self.get_parameter("search_angular_speed").get_parameter_value().double_value
        self.target_marker_id = self.get_parameter("initial_target_id").get_parameter_value().integer_value

        # Waypoints in front of each ArUco marker (x, y, yaw)
        # Positioned 1.6 meters away from the wall centers, facing the markers
        self.waypoints = {
            0: (1.72, -2.55, -0.942),     # Waypoint for Marker 0 (wall 1)
            1: (2.99, 0.97, 0.314),       # Waypoint for Marker 1 (wall 2)
            2: (0.0, 3.14, 1.571),        # Waypoint for Marker 2 (wall 3)
            3: (-2.99, 0.97, 2.827),      # Waypoint for Marker 3 (wall 4)
            4: (-1.72, -2.55, -2.199),    # Waypoint for Marker 4 (wall 5)
        }

        # Commands mapped to each ID (representing leading the robot to the next marker/wall)
        self.commands = {
            0: "Navigate to Wall 2 (Marker 1)",
            1: "Navigate to Wall 3 (Marker 2)",
            2: "Navigate to Wall 4 (Marker 3)",
            3: "Navigate to Wall 5 (Marker 4)",
            4: "Navigate to Wall 1 (Marker 0)",
        }

        # Next target sequence mapping
        self.next_targets = {
            0: 1,
            1: 2,
            2: 3,
            3: 4,
            4: 0
        }

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.cooldown_until = 0.0
        self.state = "INIT"  # States: INIT, NAVIGATING, SEARCHING, IDLE
        self.goal_handle = None

        # ArUco Dict and parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        if hasattr(aruco, "ArucoDetector"):
            self.aruco_params = aruco.DetectorParameters()
        elif hasattr(aruco, "DetectorParameters_create"):
            self.aruco_params = aruco.DetectorParameters_create()
        else:
            self.aruco_params = None

        # Publishers, subscribers and action clients
        self.image_sub = self.create_subscription(
            Image,
            "/camera/image",
            self.image_callback,
            10,
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/camera/camera_info",
            self.camera_info_callback,
            10,
        )
        
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Timers
        self.init_timer = self.create_timer(1.0, self.init_timer_callback)
        self.control_timer = self.create_timer(0.1, self.control_timer_callback)

        self.get_logger().info("Aruco Command Follower Node initialized. Waiting for Nav2...")

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)

    def init_timer_callback(self):
        if self.state == "INIT":
            if self.nav_to_pose_client.server_is_ready():
                self.get_logger().info("Nav2 server is ready. Launching first goal (Marker 2)...")
                self.init_timer.cancel()
                self.start_navigation_to(self.target_marker_id)
            else:
                self.get_logger().warn("Waiting for Nav2 action server to become available...")

    def start_navigation_to(self, marker_id):
        if marker_id not in self.waypoints:
            self.get_logger().error(f"Marker ID {marker_id} is not in waypoints list!")
            return

        x, y, yaw = self.waypoints[marker_id]
        self.get_logger().info(f"Sending Nav2 goal for Marker {marker_id} (Wall): x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.state = "NAVIGATING"
        self.target_marker_id = marker_id
        
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(
            lambda fut, tid=marker_id: self.goal_response_callback(fut, tid)
        )

    def goal_response_callback(self, future, target_id):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 goal was rejected by server!")
            if self.target_marker_id == target_id:
                self.get_logger().warn("Nav2 goal was rejected. Retrying in 2.0 seconds...")
                self.state = "INIT"
                self.retry_timer = self.create_timer(2.0, self.retry_timer_callback)
            return

        self.get_logger().info(f"Nav2 goal for Marker {target_id} accepted by server.")
        if self.target_marker_id == target_id:
            self.goal_handle = goal_handle
        
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(
            lambda fut, gh=goal_handle, tid=target_id: self.goal_result_callback(fut, gh, tid)
        )

    def retry_timer_callback(self):
        if hasattr(self, 'retry_timer') and self.retry_timer is not None:
            self.retry_timer.cancel()
            self.destroy_timer(self.retry_timer)
            self.retry_timer = None
        if self.state == "INIT":
            self.get_logger().info("Retrying Nav2 goal now...")
            self.start_navigation_to(self.target_marker_id)

    def goal_result_callback(self, future, goal_handle, target_id):
        # Only process if this is still the active target marker goal we care about
        if target_id != self.target_marker_id:
            self.get_logger().info(f"Received result for an inactive/superseded target (Marker {target_id}). Ignoring.")
            return

        # Also ignore if the goal handle itself has changed
        if self.goal_handle is None or goal_handle.goal_id != self.goal_handle.goal_id:
            self.get_logger().info("Received result for an inactive/superseded goal handle. Ignoring.")
            return

        result = future.result()
        status = result.status
        self.get_logger().info(f"Nav2 goal finished with status code: {status}")
        
        # If we reached the goal and are still in NAVIGATING state, transition to SEARCHING
        if self.state == "NAVIGATING":
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("Successfully reached waypoint. Starting SEARCHING spin...")
                self.state = "SEARCHING"
            else:
                self.get_logger().warn("Goal failed or was canceled. Transitioning to SEARCHING...")
                self.state = "SEARCHING"

    def cancel_active_goal(self):
        if self.goal_handle is not None:
            self.get_logger().info("Canceling active Nav2 goal...")
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

    def control_timer_callback(self):
        # Publish velocity commands if searching
        if self.state == "SEARCHING":
            twist = Twist()
            twist.angular.z = self.search_angular_speed
            twist.linear.x = 0.0
            self.cmd_vel_pub.publish(twist)
        elif self.state == "IDLE":
            twist = Twist()
            self.cmd_vel_pub.publish(twist)

    def detect_markers(self, gray):
        if hasattr(aruco, "ArucoDetector"):
            detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            corners, ids, _ = detector.detectMarkers(gray)
        else:
            corners, ids, _ = aruco.detectMarkers(
                gray,
                self.aruco_dict,
                parameters=self.aruco_params,
            )
        return corners, ids

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids = self.detect_markers(gray)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            ids_flat = ids.flatten().tolist()
            
            for marker_id in ids_flat:
                # Trigger action if we see the current target marker
                if marker_id == self.target_marker_id:
                    now = time.time()
                    if now > self.cooldown_until:
                        self.get_logger().info(f"Target Marker {marker_id} detected!")
                        # Execute command
                        command_text = self.commands.get(marker_id, "Unknown Command")
                        self.get_logger().info(f"==========================================")
                        self.get_logger().info(f"EXECUTING COMMAND FOR MARKER {marker_id}:")
                        self.get_logger().info(f"  -> '{command_text}'")
                        self.get_logger().info(f"==========================================")

                        # Cooldown to avoid double triggering
                        self.cooldown_until = now + self.cooldown_seconds
                        
                        # Set up next target
                        next_id = self.next_targets[marker_id]
                        self.get_logger().info(f"Next goal set to Marker {next_id}.")
                        
                        # Start navigating to the next target
                        self.start_navigation_to(next_id)
                        break

        if self.show_camera:
            try:
                cv2.imshow("Aruco Command Follower Camera", frame)
                cv2.waitKey(1)
            except Exception as e:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = ArucoCommandFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel_active_goal()
        stop_twist = Twist()
        node.cmd_vel_pub.publish(stop_twist)
        if node.show_camera:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
