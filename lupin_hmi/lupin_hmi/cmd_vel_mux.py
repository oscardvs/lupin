import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist

class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')

        # Real Mirte firmware listens on /mirte_base_controller/cmd_vel; sim uses the
        # _unstamped variant. Override via the `cmd_vel_topic` parameter from the launch.
        self.declare_parameter('cmd_vel_topic', '/mirte_base_controller/cmd_vel')
        out_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.get_logger().info(f'cmd_vel_mux publishing on {out_topic}')

        # Both the mecanum controller and the sim's twist_mux subscribe BEST_EFFORT.
        # A RELIABLE publisher here silently drops messages through twist_mux, leaving
        # the gazebo_planar_move plugin idle even though wheels still spin.
        cmd_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher_ = self.create_publisher(Twist, out_topic, cmd_qos)
        
        # Inputs: Manual (iPhone/Keyboard) vs Autonomous (Group Logic)
        self.create_subscription(Twist, '/cmd_vel_manual', self.manual_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_auto', self.auto_callback, 10)
        
        self.last_manual_msg = Twist()
        self.manual_active_timeout = 0.5 # Seconds to keep control after manual input stops
        self.last_manual_time = self.get_clock().now()

    def manual_callback(self, msg):
        # If the joystick is being moved (not zero)
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self.last_manual_time = self.get_clock().now()
            self.publisher_.publish(msg)

    def auto_callback(self, msg):
        # Only pass autonomous commands if manual hasn't been used recently
        now = self.get_clock().now()
        diff = (now - self.last_manual_time).nanoseconds / 1e9
        
        if diff > self.manual_active_timeout:
            self.publisher_.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(CmdVelMux())