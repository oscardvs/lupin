import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')
        
        # Output to the robot base
        self.publisher_ = self.create_publisher(Twist, '/mirte_base_controller/cmd_vel_unstamped', 10)
        
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