from .junheon_mobile_controller_wrapper_cpp import CasterWheel as CasterWheelcpp
import numpy as np

# Holmberg, Robert, and Oussama Khatib. "Development and control of a holonomic mobile robot for mobile manipulation tasks." The International Journal of Robotics Research 19.11 (2000): 1066-1074.
class CasterWheel:
    """
    CasterWheel class for controlling a powerd-caster-wheel-based mobile robot.

    Attributes:
        wheel_names (list of str): Names of the wheels.
        base_width (float): Width of the robot's base.
        base_length (float): Length of the robot's base.
        wheel_radius (float): Radius of the wheels.
        wheel_offset (float): Offset of the caster.
        lin_vel_limit (float): Linear velocity limit.
        ang_vel_limit (float): Angular velocity limit.
        lin_acc_limit (float): Linear acceleration limit.
        ang_acc_limit (float): Angular acceleration limit.
        hz (float): Control frequency.

    Methods:
        IK(desired_base_velocity: np.array, current_wheel_angle: np.array) -> np.array:
            Computes the inverse kinematics for the desired base velocity.
        
        FK(desired_wheel_vel: np.array, current_wheel_angle: np.array) -> np.array:
            Computes the forward kinematics for the given wheel velocities.
        
        VelocityCommand(desired_base_vel: np.array, current_base_vel: np.array, current_wheel_angle: np.array) -> np.array:
            Computes the velocity command for the wheels based on desired and current base velocities.
    """
    
    def __init__(self, wheel_names, base_width, base_length, wheel_radius, wheel_offset, lin_vel_limit, ang_vel_limit, lin_acc_limit, ang_acc_limit, control_freq):
        self.wheel_names = wheel_names
        self.base_width = base_width
        self.base_length = base_length
        self.wheel_radius = wheel_radius
        self.wheel_offset = wheel_offset
        self.lin_vel_limit = lin_vel_limit
        self.ang_vel_limit = ang_vel_limit
        self.lin_acc_limit = lin_acc_limit
        self.ang_acc_limit = ang_acc_limit
        self.hz = control_freq
        
        self.controller = CasterWheelcpp(self.wheel_names,
                                         self.base_width,
                                         self.base_length,
                                         self.wheel_radius,
                                         self.wheel_offset,
                                         self.lin_vel_limit,
                                         self.ang_vel_limit,
                                         self.lin_acc_limit,
                                         self.ang_acc_limit,
                                         self.hz)
        
        
    def IK(self, desired_base_velocity: np.array, current_wheel_angle: np.array) -> np.array:
        """
        Computes the inverse kinematics for the desired base velocity.
        
        Args:
            desired_base_velocity (np.array): Desired base velocity as a 3D vector.
            current_wheel_angle (np.array): Current wheel angle.
            
        Returns:
            np.array: Wheel velocities.
        """
        assert isinstance(desired_base_velocity, np.ndarray) and isinstance(current_wheel_angle, np.ndarray), "Input must be a numpy array."
        try:
            return self.controller.IK(desired_base_velocity, current_wheel_angle)
            
        except Exception as e:
            print(f"Error in IK calculation: {e}")
            return np.array([])

    def FK(self, desired_wheel_vel: np.array, current_wheel_angle: np.array) -> np.array:
        """
        Computes the forward kinematics for the given wheel velocities.
        
        Args:
            desired_wheel_vel (np.array): Desired wheel velocities.
            current_wheel_angle (np.array): Current wheel angle.

        Returns:
            np.array: Base velocities.
        """
        assert isinstance(desired_wheel_vel, np.ndarray) and isinstance(current_wheel_angle, np.ndarray), "Input must be a numpy array."
        try:
            return self.controller.FK(desired_wheel_vel, current_wheel_angle)
        except Exception as e:
            print(f"Error in FK calculation: {e}")
            return np.array([])

    def VelocityCommand(self, desired_base_vel: np.array, current_base_vel: np.array, current_wheel_angle: np.array) -> np.array:
        """
        Computes the velocity command for the wheels based on desired and current base velocities.
        
        Args:
            desired_base_vel (np.array): Desired base velocity.
            current_base_vel (np.array): Current base velocity.
            current_wheel_angle (np.array): Current wheel angle.
            
        Returns:
            np.array: Wheel velocity commands.
        """
        assert isinstance(desired_base_vel, np.ndarray) and isinstance(current_base_vel, np.ndarray) and isinstance(current_wheel_angle, np.ndarray), \
            "Both inputs must be numpy arrays."
        try:
            return self.controller.VelocityCommand(desired_base_vel, current_base_vel, current_wheel_angle)
        except Exception as e:
            print(f"Error in VelocityCommand calculation: {e}")
            return np.array([])

