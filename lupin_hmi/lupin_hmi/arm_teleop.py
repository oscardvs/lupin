import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class ArmTeleop(Node):
    def __init__(self):
        super().__init__('arm_teleop')
        
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/mirte_master_arm_controller/joint_trajectory', 
            10)
            
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)
            
        # Hardware Fix: Slower 10Hz loop so the stepper motors don't choke
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.joint_names = [
            'shoulder_pan_joint', 
            'shoulder_lift_joint', 
            'elbow_joint', 
            'wrist_joint'
        ]
        
        # Start at neutral positions
        self.current_positions = [0.0, -1.56, -1.56, 1.56]
        self.joy_cmds = [0.0, 0.0, 0.0, 0.0]

    def joy_callback(self, msg):
        try:
            # 1. SHOULDER (Thumbstick Axes)
            self.joy_cmds[0] = msg.axes[2] * 1.0  # Pan
            self.joy_cmds[1] = msg.axes[3] * 1.0  # Lift

            # 2. ELBOW & WRIST (D-Pad Buttons 11, 12, 13, 14)
            self.joy_cmds[2] = 0.0
            self.joy_cmds[3] = 0.0

            if msg.buttons[11] == 1:
                self.joy_cmds[2] = 1.0
            elif msg.buttons[12] == 1:
                self.joy_cmds[2] = -1.0

            if msg.buttons[13] == 1:
                self.joy_cmds[3] = 1.0
            elif msg.buttons[14] == 1:
                self.joy_cmds[3] = -1.0
                
        except IndexError:
            pass

    def timer_callback(self):
        # Hardware Fix: Larger chunk size (0.15) for actual physical steps
        speed = 0.15 
        moved = False
        
        for i in range(4):
            if abs(self.joy_cmds[i]) > 0.05:
                self.current_positions[i] += self.joy_cmds[i] * speed
                moved = True
                
        if moved:
            self.send_trajectory()

    def send_trajectory(self):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = self.current_positions
        
        # HARDWARE FIX: Force a positive velocity to bypass Telemetrix 0.0 error
        point.velocities = [1.0, 1.0, 1.0, 1.0] 
        
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100000000 # 0.1 seconds
        
        traj.points.append(point)
        self.publisher_.publish(traj)

def main(args=None):
    rclpy.init(args=args)
    node = ArmTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()