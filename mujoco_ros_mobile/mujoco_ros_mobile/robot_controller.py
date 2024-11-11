import time
import mujoco
import mujoco.viewer
import numpy as np
from jhmc import DifferentialWheel, MecanumWheel, CasterWheel
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Retrieve the robot_name parameter
        self.declare_parameter('robot_name', 'summit_xls')  # Default to 'summit_xls'
        robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        assert robot_name in ['summit_xls', 'husky', 'pcv'], f"{robot_name} is not included in summit_xls, husky, or pcv!"

        self.robot_name = robot_name
        self.target_base_vel = np.zeros(3)

        # Set up ROS 2 subscribers and publishers
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.target_velocity_callback, 1)
        self.publisher = self.create_publisher(Twist, '/current_velocity', 1)
        
        # Load the model
        xml_file_path = get_package_share_directory(__package__) + f'/robots/{robot_name}.xml'
        self.m = mujoco.MjModel.from_xml_path(xml_file_path)

        self.setup_controller()
        
        # Initial state
        self.d = mujoco.MjData(self.m)
        self.mobile_kv = 200
        self.current_wheel_pos = np.zeros(self.m.nu)
        self.current_wheel_vel = np.zeros(self.m.nu)
        self.current_base_vel = np.zeros(3)

        # Viewer parameters
        self.viewer_update_rate = 20  # Update viewer every 10 steps
        self.viewer_step = 0

    def setup_controller(self):
        floating_q_names = ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
        floating_v_names = ['vx', 'vy', 'vz', 'wx', 'wy', 'wz']
        joint_names = [self.m.names[self.m.name_jntadr[i]:].split(b'\x00', 1)[0].decode('utf-8') for i in range(self.m.njnt) if self.m.names[self.m.name_jntadr[i]:].split(b'\x00', 1)[0].decode('utf-8')]
        self.q_names = floating_q_names + joint_names
        self.v_names = floating_v_names + joint_names
        actuator_names = [self.m.names[self.m.name_actuatoradr[i]:].split(b'\x00', 1)[0].decode('utf-8') for i in range(self.m.nu)]
        
        if self.robot_name == 'summit_xls':
            self.controller = MecanumWheel(wheel_names=actuator_names, base_length=0.225*2, base_width=0.2045*2,
                                           wheel_radius=0.120, lin_vel_limit=1.5, ang_vel_limit=2.0,
                                           lin_acc_limit=1.0, ang_acc_limit=1.0, control_freq=100)
        elif self.robot_name == 'husky':
            self.controller = DifferentialWheel(wheel_names=actuator_names, base_width=1.875*0.2854*2,
                                                wheel_radius=0.1651, lin_vel_limit=1.0, ang_vel_limit=3.0,
                                                lin_acc_limit=2.0, ang_acc_limit=6.0, control_freq=100)
        elif self.robot_name == 'pcv':
            self.controller = CasterWheel(wheel_names=actuator_names, base_width=0.125*2, base_length=0.215*2,
                                          wheel_radius=0.055, wheel_offset=0.020, lin_vel_limit=1.0, ang_vel_limit=3.0,
                                          lin_acc_limit=2.0, ang_acc_limit=6.0, control_freq=100)

    def target_velocity_callback(self, msg):
        self.target_base_vel = np.array([msg.linear.x, msg.linear.y, msg.angular.z])

    def run(self):
        with mujoco.viewer.launch_passive(self.m, self.d) as viewer:
            while rclpy.ok() and viewer.is_running():
                step_start = time.time()
                rclpy.spin_once(self, timeout_sec=0.001)
                mujoco.mj_step(self.m, self.d)

                # Get current wheel position, velocity
                for i, wheel_name in enumerate(self.controller.wheel_names):
                    self.current_wheel_pos[i] = self.d.qpos[self.q_names.index(wheel_name)]
                    self.current_wheel_vel[i] = self.d.qvel[self.v_names.index(wheel_name)]
                    
                # Compute current base velocity
                if self.robot_name in ['summit_xls', 'husky']:
                    self.current_base_vel = self.controller.FK(self.current_wheel_vel)
                elif self.robot_name == 'pcv':
                    self.current_base_vel = self.controller.FK(self.current_wheel_vel, self.current_wheel_pos)

                # Compute desired wheel velocity
                if self.robot_name in ['summit_xls', 'husky']:
                    desired_wheel_vel = self.controller.VelocityCommand(current_base_vel=self.current_base_vel, 
                                                                        desired_base_vel=self.target_base_vel)
                elif self.robot_name == 'pcv':
                    desired_wheel_vel = self.controller.VelocityCommand(current_base_vel=self.current_base_vel, 
                                                                        desired_base_vel=self.target_base_vel,
                                                                        current_wheel_angle=self.current_wheel_pos)

                # Control input for wheels
                self.d.ctrl = (desired_wheel_vel - self.current_wheel_vel) * self.mobile_kv

                # Publish current base velocity as Twist
                current_vel_msg = Twist()
                current_vel_msg.linear.x = self.current_base_vel[0]
                current_vel_msg.linear.y = self.current_base_vel[1]
                current_vel_msg.angular.z = self.current_base_vel[2]
                self.publisher.publish(current_vel_msg)

                # Update viewer
                if self.viewer_step % self.viewer_update_rate == 0:
                    viewer.sync()
                    # print("\n===========================")
                    # print("targrt base velocity  : ", self.target_base_vel)
                    # print("current base velocity : ", self.current_base_vel)
                    # print("desired wheel velocity: ", desired_wheel_vel)
                    # print("current wheel velocity: ", self.current_wheel_vel)
                    # print("===========================\n")
                self.viewer_step += 1

                # Maintain physics timestep at 1000 Hz
                time_until_next_step = self.m.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

def main():
    rclpy.init()
    robot_controller = RobotController()
    robot_controller.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
