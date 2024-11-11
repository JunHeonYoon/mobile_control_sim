from .junheon_mobile_controller_wrapper_cpp import DifferentialWheel as DifferentialWheelcpp
import numpy as np

class DifferentialWheel:
    """
    DifferentialWheel class for controlling a differential-drive robot.

    Attributes:
        wheel_names (list of str): Names of the wheels.
        base_width (float): Width of the robot's base.
        wheel_radius (float): Radius of the wheels.
        lin_vel_limit (float): Linear velocity limit.
        ang_vel_limit (float): Angular velocity limit.
        lin_acc_limit (float): Linear acceleration limit.
        ang_acc_limit (float): Angular acceleration limit.
        hz (float): Control frequency.

    Methods:
        IK(desired_base_velocity: np.array) -> np.array:
            Computes the inverse kinematics for the desired base velocity.
        
        FK(desired_wheel_vel: np.array) -> np.array:
            Computes the forward kinematics for the given wheel velocities.
        
        VelocityCommand(desired_base_vel: np.array, current_base_vel: np.array) -> np.array:
            Computes the velocity command for the wheels based on desired and current base velocities.
    """
    
    def __init__(self, wheel_names, base_width, wheel_radius, lin_vel_limit, ang_vel_limit, lin_acc_limit, ang_acc_limit, control_freq):
        self.wheel_names = wheel_names
        self.base_width = base_width
        self.wheel_radius = wheel_radius
        self.lin_vel_limit = lin_vel_limit
        self.ang_vel_limit = ang_vel_limit
        self.lin_acc_limit = lin_acc_limit
        self.ang_acc_limit = ang_acc_limit
        self.hz = control_freq
        
        self.controller = DifferentialWheelcpp(self.wheel_names,
                                               self.base_width,
                                               self.wheel_radius,
                                               self.lin_vel_limit,
                                               self.ang_vel_limit,
                                               self.lin_acc_limit,
                                               self.ang_acc_limit,
                                               self.hz)
        
    def IK(self, desired_base_velocity: np.array) -> np.array:
        """
        Computes the inverse kinematics for the desired base velocity.
        
        Args:
            desired_base_velocity (np.array): Desired base velocity as a 3D vector.
            
        Returns:
            np.array: Wheel velocities.
        """
        assert isinstance(desired_base_velocity, np.ndarray), "Input must be a numpy array."
        try:
            return self.controller.IK(desired_base_velocity)
        except Exception as e:
            print(f"Error in IK calculation: {e}")
            return np.array([])

    def FK(self, desired_wheel_vel: np.array) -> np.array:
        """
        Computes the forward kinematics for the given wheel velocities.
        
        Args:
            desired_wheel_vel (np.array): Desired wheel velocities.
            
        Returns:
            np.array: Base velocities.
        """
        assert isinstance(desired_wheel_vel, np.ndarray), "Input must be a numpy array."
        try:
            return self.controller.FK(desired_wheel_vel)
        except Exception as e:
            print(f"Error in FK calculation: {e}")
            return np.array([])

    def VelocityCommand(self, desired_base_vel: np.array, current_base_vel: np.array) -> np.array:
        """
        Computes the velocity command for the wheels based on desired and current base velocities.
        
        Args:
            desired_base_vel (np.array): Desired base velocity.
            current_base_vel (np.array): Current base velocity.
            
        Returns:
            np.array: Wheel velocity commands.
        """
        assert isinstance(desired_base_vel, np.ndarray) and isinstance(current_base_vel, np.ndarray), \
            "Both inputs must be numpy arrays."
        try:
            desired_base_vel[1] = 0
            return self.controller.VelocityCommand(desired_base_vel, current_base_vel)
        except Exception as e:
            print(f"Error in VelocityCommand calculation: {e}")
            return np.array([])

