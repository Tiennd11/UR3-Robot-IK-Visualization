"""
UR3 URDF-based Forward Kinematics
Matches exactly with ROS ur_description URDF
"""
import numpy as np


class UR3_URDF_ForwardKinematics:
    """
    URDF-based forward kinematics matching ROS ur_description package.
    This uses the EXACT joint transforms from the URDF file.
    """
    
    def __init__(self):
        """Initialize UR3 URDF parameters"""
        # From ur3.urdf.xacro
        self.d1 = 0.1519          # shoulder_height  
        self.a2 = -0.24365        # negative of upper_arm_length
        self.a3 = -0.21325        # negative of forearm_length
        self.d4 = 0.11235         # wrist_1 offset
        self.d5 = 0.08535         # wrist_2 offset
        self.d6 = 0.0819          # wrist_3 offset
        
        self.shoulder_offset = 0.1198  # Y offset for shoulder_lift_joint
        self.elbow_offset = -0.0925    # Y offset for elbow_joint
        
        # Computed lengths
        self.upper_arm_length = -self.a2  # 0.24365
        self.forearm_length = -self.a3    # 0.21325
        self.wrist_1_length = self.d4 - self.elbow_offset - self.shoulder_offset
        self.wrist_2_length = self.d5
        self.wrist_3_length = self.d6
        
        # UR3 Joint limits (radians) from official specifications
        # Note: UR3 has ±360° capability but practical limits may vary
        self.joint_limits = [
            (-2*np.pi, 2*np.pi),    # Joint 1 (base): ±360°
            (-2*np.pi, 2*np.pi),    # Joint 2 (shoulder): ±360°
            (-2*np.pi, 2*np.pi),    # Joint 3 (elbow): ±360°
            (-2*np.pi, 2*np.pi),    # Joint 4 (wrist 1): ±360°
            (-2*np.pi, 2*np.pi),    # Joint 5 (wrist 2): ±360°
            (-2*np.pi, 2*np.pi),    # Joint 6 (wrist 3): ±360°
        ]
    
    def joint_transform(self, xyz, rpy, q=0, axis='z'):
        """
        Create joint transform from URDF origin + joint rotation.
        
        Args:
            xyz: [x, y, z] translation
            rpy: [roll, pitch, yaw] rotation
            q: joint angle
            axis: rotation axis ('x', 'y', or 'z')
        """
        # Fixed transform from origin
        T_origin = self._create_transform(xyz, rpy)
        
        # Joint rotation
        if axis == 'z':
            T_joint = self._rotz(q)
        elif axis == 'y':
            T_joint = self._roty(q)
        else:  # 'x'
            T_joint = self._rotx(q)
        
        return T_origin @ T_joint
    
    def _create_transform(self, xyz, rpy):
        """Create 4x4 transform from xyz and rpy"""
        T = np.eye(4)
        T[:3, 3] = xyz
        
        r, p, y = rpy
        Rx = self._rotx(r)
        Ry = self._roty(p)
        Rz = self._rotz(y)
        T[:3, :3] = (Rz @ Ry @ Rx)[:3, :3]
        
        return T
    
    def _rotx(self, angle):
        """Rotation around X axis"""
        c, s = np.cos(angle), np.sin(angle)
        return np.array([
            [1, 0, 0, 0],
            [0, c, -s, 0],
            [0, s, c, 0],
            [0, 0, 0, 1]
        ])
    
    def _roty(self, angle):
        """Rotation around Y axis"""
        c, s = np.cos(angle), np.sin(angle)
        return np.array([
            [c, 0, s, 0],
            [0, 1, 0, 0],
            [-s, 0, c, 0],
            [0, 0, 0, 1]
        ])
    
    def _rotz(self, angle):
        """Rotation around Z axis"""
        c, s = np.cos(angle), np.sin(angle)
        return np.array([
            [c, -s, 0, 0],
            [s, c, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
    
    def forward_kinematics(self, joint_angles):
        """
        Compute forward kinematics using URDF joint transforms.
        
        Args:
            joint_angles: [q1, q2, q3, q4, q5, q6]
            
        Returns:
            Dictionary of link transforms
        """
        q = joint_angles
        
        # Base link is at origin
        T_base = np.eye(4)
        
        # shoulder_pan_joint: origin xyz="0 0 ${d1}" rpy="0 0 0", axis="0 0 1"
        T1 = self.joint_transform([0, 0, self.d1], [0, 0, 0], q[0], 'z')
        T_shoulder = T_base @ T1
        
        # shoulder_lift_joint: origin xyz="0 ${shoulder_offset} 0" rpy="0 ${pi/2} 0", axis="0 1 0"
        T2 = self.joint_transform([0, self.shoulder_offset, 0], [0, np.pi/2, 0], q[1], 'y')
        T_upper_arm = T_shoulder @ T2
        
        # elbow_joint: origin xyz="0 ${elbow_offset} ${upper_arm_length}" rpy="0 0 0", axis="0 1 0"
        T3 = self.joint_transform([0, self.elbow_offset, self.upper_arm_length], [0, 0, 0], q[2], 'y')
        T_forearm = T_upper_arm @ T3
        
        # wrist_1_joint: origin xyz="0 0 ${forearm_length}" rpy="0 ${pi/2} 0", axis="0 1 0"
        T4 = self.joint_transform([0, 0, self.forearm_length], [0, np.pi/2, 0], q[3], 'y')
        T_wrist_1 = T_forearm @ T4
        
        # wrist_2_joint: origin xyz="0 ${wrist_1_length} 0" rpy="0 0 0", axis="0 0 1"
        T5 = self.joint_transform([0, self.wrist_1_length, 0], [0, 0, 0], q[4], 'z')
        T_wrist_2 = T_wrist_1 @ T5
        
        # wrist_3_joint: origin xyz="0 0 ${wrist_2_length}" rpy="0 0 0", axis="0 1 0"
        T6 = self.joint_transform([0, 0, self.wrist_2_length], [0, 0, 0], q[5], 'y')
        T_wrist_3 = T_wrist_2 @ T6
        
        return {
            'base': T_base,
            'shoulder': T_shoulder,
            'upper_arm': T_upper_arm,
            'forearm': T_forearm,
            'wrist_1': T_wrist_1,
            'wrist_2': T_wrist_2,
            'wrist_3': T_wrist_3
        }
    
    def get_joint_positions(self, joint_angles):
        """
        Get the 3D positions of each joint.
        
        Args:
            joint_angles: [q1, q2, q3, q4, q5, q6]
            
        Returns:
            List of 3D positions for each joint
        """
        transforms = self.forward_kinematics(joint_angles)
        positions = [
            transforms['base'][:3, 3],
            transforms['shoulder'][:3, 3],
            transforms['upper_arm'][:3, 3],
            transforms['forearm'][:3, 3],
            transforms['wrist_1'][:3, 3],
            transforms['wrist_2'][:3, 3],
            transforms['wrist_3'][:3, 3]
        ]
        return positions
    
    def check_joint_limits(self, joint_angles):
        """Check if joint angles are within limits."""
        for i, angle in enumerate(joint_angles):
            lo, hi = self.joint_limits[i]
            if angle < lo or angle > hi:
                return False
        return True
    
    def normalize_joint_angles(self, joint_angles):
        """Normalize joint angles to [-pi, pi] range."""
        normalized = np.array(joint_angles, dtype=float)
        for i in range(len(normalized)):
            while normalized[i] > np.pi:
                normalized[i] -= 2 * np.pi
            while normalized[i] < -np.pi:
                normalized[i] += 2 * np.pi
        return normalized
    
    def get_end_effector_transform(self, joint_angles):
        """Get end-effector 4x4 transform matrix."""
        transforms = self.forward_kinematics(joint_angles)
        return transforms['wrist_3']


if __name__ == '__main__':
    # Test
    fk = UR3_URDF_ForwardKinematics()
    q_home = [0, 0, 0, 0, 0, 0]
    transforms = fk.forward_kinematics(q_home)
    
    print("URDF FK at home position:")
    for name, T in transforms.items():
        print(f"{name:12s}: {T[:3, 3]}")
