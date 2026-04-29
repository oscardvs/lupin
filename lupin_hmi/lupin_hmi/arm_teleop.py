import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class ArmTeleop(Node):
    def __init__(self):
        super().__init__('arm_teleop')
        
        self.publisher_ = self.create_publisher(
            JointTrajectory, '/mirte_master_arm_controller/joint_trajectory', 10)
            
        # Listen to Foxglove sliders
        self.create_subscription(JointState, '/joint_states', self.slider_callback, 10)
        
        # Listen to the PS4 Controller
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_joint']
        self.current_positions = [0.0, 0.0, 0.0, 0.0]
        self.joy_cmds = [0.0, 0.0, 0.0, 0.0]
        
        # A 20Hz loop to smoothly move the arm when sticks are pushed
        self.timer = self.create_timer(0.05, self.timer_callback)

    def slider_callback(self, msg):
        if len(msg.velocity) > 0: return # Ignore Gazebo physical state
        
        changed = False
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                pos = float(msg.position[idx])
                if abs(self.current_positions[i] - pos) > 0.001:
                    changed = True
                self.current_positions[i] = pos
                
        if changed:
            self.send_trajectory()

    def joy_callback(self, msg):
        try:
            # 1. SHOULDER (Using Thumbstick Axes)
            self.joy_cmds[0] = msg.axes[2] * 1.0  # Shoulder Pan (Reversed to 1.0)
            self.joy_cmds[1] = msg.axes[3] * -1.0  # Shoulder Lift

            # 2. ELBOW & WRIST (Using D-Pad Buttons)
            # Reset them to 0 first, so they stop moving when you let go
            self.joy_cmds[2] = 0.0
            self.joy_cmds[3] = 0.0

            # Elbow (D-Pad Up/Down -> Buttons 11 and 12)
            if msg.buttons[11] == 1:
                self.joy_cmds[2] = 1.0
            elif msg.buttons[12] == 1:
                self.joy_cmds[2] = -1.0

            # Wrist (D-Pad Left/Right -> Buttons 13 and 14)
            if msg.buttons[13] == 1:
                self.joy_cmds[3] = 1.0
            elif msg.buttons[14] == 1:
                self.joy_cmds[3] = -1.0

        except IndexError:
            pass

    def timer_callback(self):
        speed = 0.06 # How fast the arm moves per tick when holding a joystick
        moved = False
        
        for i in range(4):
            # Apply a small deadzone to prevent drift
            if abs(self.joy_cmds[i]) > 0.1:
                self.current_positions[i] += self.joy_cmds[i] * speed
                # Clamp limits to avoid spinning out of control
                self.current_positions[i] = max(-3.14, min(3.14, self.current_positions[i]))
                moved = True
                
        if moved:
            self.send_trajectory()

    def send_trajectory(self):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = self.current_positions
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 50000000 # Give controller 0.05s to reach target
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