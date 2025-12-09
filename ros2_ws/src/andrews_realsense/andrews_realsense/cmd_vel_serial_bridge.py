import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial


class CmdVelSerialBridge(Node):
    def __init__(self):
        super().__init__("cmd_vel_serial_bridge")

        # Parameters
        port = (
            self.declare_parameter("port", "/dev/ttyUSB0")
            .get_parameter_value()
            .string_value
        )
        baud = (
            self.declare_parameter("baud", 115200)
            .get_parameter_value()
            .integer_value
        )

        self.get_logger().info(f"[BRIDGE] Opening serial port {port} @ {baud}...")
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.1)

        # Store last received cmd_vel
        self.last_cmd = Twist()

        # Subscribe to /cmd_vel
        self.cmd_sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10,
        )

        # Timer to send at fixed rate (20 Hz)
        self.counter = 0
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info(
            "[BRIDGE] CmdVelSerialBridge started. Subscribing to /cmd_vel and "
            "streaming to serial."
        )

    def cmd_vel_callback(self, msg: Twist):
        """Store the latest /cmd_vel."""
        self.last_cmd = msg
        self.get_logger().info(
            f"[BRIDGE] Received /cmd_vel: "
            f"lin=({msg.linear.x:.3f}, {msg.linear.y:.3f}), "
            f"ang_z={msg.angular.z:.3f}"
        )

    def timer_callback(self):
        """Periodically send the latest cmd_vel over serial."""
        self.counter += 1
        cmd = self.last_cmd

        # Format: x,y,omega\n
        line = f"{cmd.linear.x:.3f},{cmd.linear.y:.3f},{cmd.angular.z:.3f}\n"

        try:
            self.ser.write(line.encode("ascii"))
            self.get_logger().info(
                f"[BRIDGE] Sent #{self.counter}: {line.strip()}"
            )
        except Exception as e:
            self.get_logger().error(f"[BRIDGE] Serial write failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("[BRIDGE] Shutting down bridge node.")
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
