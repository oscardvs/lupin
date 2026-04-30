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