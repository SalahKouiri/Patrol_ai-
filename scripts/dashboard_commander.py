import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os
import signal

class DashboardCommander(Node):
    def __init__(self):
        super().__init__('dashboard_commander')
        self.subscription = self.create_subscription(String, '/dashboard_cmd', self.cmd_callback, 10)
        self.active_process = None
        self.map_dir = "/home/agilex/limo_ros2_ws/src/limo_ros2/limo_bringup/maps"
        self.get_logger().info("🤖 Dashboard Commander is READY. Listening on /dashboard_cmd...")

    def stop_current_process(self):
        if self.active_process:
            self.get_logger().info("🛑 Stopping current background process...")
            os.killpg(os.getpgid(self.active_process.pid), signal.SIGINT)
            self.active_process.wait()
            self.active_process = None

    def cmd_callback(self, msg):
        command = msg.data
        self.get_logger().info(f"📥 Received command: {command}")

        # --- MAPPING ---
        if command == "START_MAPPING":
            self.stop_current_process()
            self.get_logger().info("🚀 Launching SLAM...")
            cmd = ["ros2", "launch", "limo_bringup", "limo_mapping.launch.py"]
            self.active_process = subprocess.Popen(cmd, preexec_fn=os.setsid)

        # --- NAVIGATION ---
        elif command.startswith("START_NAV:"):
            self.stop_current_process()
            map_name = command.split(":")[1]
            map_yaml = os.path.join(self.map_dir, f"{map_name}.yaml")
            self.get_logger().info(f"🗺️ Launching Navigation with map: {map_yaml}...")
            cmd = ["ros2", "launch", "limo_bringup", "limo_nav2.launch.py", f"map:={map_yaml}"]
            self.active_process = subprocess.Popen(cmd, preexec_fn=os.setsid)

        # --- STOP ---
        elif command == "STOP":
            self.stop_current_process()
            self.get_logger().info("🛑 All background processes stopped.")


def main():
    rclpy.init()
    node = DashboardCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
