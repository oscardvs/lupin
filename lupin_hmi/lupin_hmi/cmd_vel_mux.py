import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')
        
        # Subscribe to your PS4 controller
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_manual',
            self.listener_callback,
            10)
            
        # Publish exactly what the hardware wants: a normal Twist on the main cmd_vel topic
        self.publisher_ = self.create_publisher(
            Twist, 
            '/mirte_base_controller/cmd_vel', 
            10)

    def listener_callback(self, msg):
        # We just pass the message straight through!
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()