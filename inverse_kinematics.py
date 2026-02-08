"""
Inverse Kinematics for UR3 Robot

Numerical IK solver using multi-start optimization.
Finds multiple configurations that reach the same end-effector pose.
"""

import numpy as np
from urdf_forward_kinematics import UR3_URDF_ForwardKinematics


class UR3_InverseKinematics:
    """
    Numerical IK solver for UR3 robot using URDF FK.
    Uses multi-start optimization to find up to 8 distinct solutions.
    """
    
    def __init__(self):
        """Initialize IK solver."""
        self.fk = UR3_URDF_ForwardKinematics()
        self.epsilon = 1e-6
        self.max_iter = 200
        self.tol = 1e-6
    
    def solve_ik(self, T_desired):
        """
        Find all IK solutions for a desired end-effector pose.
        
        Args:
            T_desired: 4x4 homogeneous transformation matrix
            
        Returns:
            List of valid joint angle solutions
        """
        solutions = []
        
        # Generate diverse initial guesses to find multiple solutions
        seeds = self._generate_seeds()
        
        for seed in seeds:
            result = self._solve_single(T_desired, seed)
            if result is not None:
                # Normalize to [-pi, pi]
                result = self.fk.normalize_joint_angles(result)
                
                # Check if it's a new unique solution
                is_new = True
                for existing in solutions:
                    diff = np.abs(result - self.fk.normalize_joint_angles(existing))
                    if np.all(diff < 0.05):  # ~3 degrees
                        is_new = False
                        break
                
                if is_new:
                    solutions.append(list(result))
        
        return solutions
    
    def _generate_seeds(self):
        """Generate diverse starting configurations."""
        seeds = []
        
        # Standard configurations
        configs = [
            [0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0],   # Ready
            [0, 0, 0, 0, 0, 0],                                  # Home
            [0, -np.pi/2, -np.pi/2, 0, np.pi/2, 0],            # Elbow up
            [np.pi, -np.pi/2, np.pi/2, -np.pi/2, np.pi/2, 0],  # Rear
            [0, -np.pi/4, np.pi/4, -np.pi/2, -np.pi/2, 0],     # Mid
            [np.pi, -np.pi/2, -np.pi/2, np.pi/2, np.pi/2, 0],  # Rear elbow
            [0, -np.pi, np.pi/2, 0, -np.pi/2, np.pi],           # Flip
            [-np.pi, -np.pi/2, np.pi/2, np.pi/2, -np.pi/2, 0], # Left rear
        ]
        seeds.extend(configs)
        
        # Random seeds for broader coverage
        rng = np.random.RandomState(42)
        for _ in range(24):
            seed = rng.uniform(-np.pi, np.pi, 6)
            seeds.append(list(seed))
        
        return seeds
    
    def _solve_single(self, T_desired, q_init):
        """
        Solve IK from a single initial guess using damped least squares (Levenberg-Marquardt).
        
        Args:
            T_desired: 4x4 target pose
            q_init: initial joint angles
            
        Returns:
            Solution joint angles or None
        """
        q = np.array(q_init, dtype=float)
        
        p_desired = T_desired[:3, 3]
        R_desired = T_desired[:3, :3]
        
        damping = 0.01
        
        for iteration in range(self.max_iter):
            # Current FK
            T_current = self.fk.get_end_effector_transform(q)
            p_current = T_current[:3, 3]
            R_current = T_current[:3, :3]
            
            # Position error
            dp = p_desired - p_current
            
            # Orientation error (axis-angle from rotation error matrix)
            R_err = R_desired @ R_current.T
            # Extract axis-angle
            angle = np.arccos(np.clip((np.trace(R_err) - 1) / 2, -1, 1))
            if angle < self.epsilon:
                dw = np.zeros(3)
            else:
                dw = angle / (2 * np.sin(angle)) * np.array([
                    R_err[2, 1] - R_err[1, 2],
                    R_err[0, 2] - R_err[2, 0],
                    R_err[1, 0] - R_err[0, 1]
                ])
            
            # Error vector [position; orientation]
            error = np.concatenate([dp, dw])
            
            # Check convergence
            pos_err = np.linalg.norm(dp)
            rot_err = np.linalg.norm(dw)
            
            if pos_err < self.tol and rot_err < self.tol * 5:
                # Verify with FK
                if self._verify(q, T_desired):
                    return q
                else:
                    return None
            
            # Compute Jacobian numerically
            J = self._numerical_jacobian(q)
            
            # Damped least squares: dq = J^T (J J^T + lambda^2 I)^-1 * error
            JJT = J @ J.T + damping**2 * np.eye(6)
            dq = J.T @ np.linalg.solve(JJT, error)
            
            # Update
            q = q + dq
        
        return None  # Did not converge
    
    def _numerical_jacobian(self, q, delta=1e-6):
        """Compute 6x6 Jacobian numerically."""
        J = np.zeros((6, 6))
        
        T0 = self.fk.get_end_effector_transform(q)
        p0 = T0[:3, 3]
        R0 = T0[:3, :3]
        
        for i in range(6):
            q_plus = np.array(q, dtype=float)
            q_plus[i] += delta
            
            T_plus = self.fk.get_end_effector_transform(q_plus)
            p_plus = T_plus[:3, 3]
            R_plus = T_plus[:3, :3]
            
            # Linear velocity
            J[:3, i] = (p_plus - p0) / delta
            
            # Angular velocity
            dR = R_plus @ R0.T
            angle = np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1))
            if angle < 1e-10:
                J[3:, i] = 0
            else:
                J[3:, i] = angle / (2 * np.sin(angle) * delta) * np.array([
                    dR[2, 1] - dR[1, 2],
                    dR[0, 2] - dR[2, 0],
                    dR[1, 0] - dR[0, 1]
                ])
        
        return J
    
    def _verify(self, q, T_desired, pos_tol=1e-4, rot_tol=5e-3):
        """Verify solution accuracy."""
        try:
            T_actual = self.fk.get_end_effector_transform(q)
            pos_err = np.linalg.norm(T_actual[:3, 3] - T_desired[:3, 3])
            R_err = np.linalg.norm(T_actual[:3, :3] - T_desired[:3, :3], 'fro')
            return pos_err < pos_tol and R_err < rot_tol
        except:
            return False


# Test
if __name__ == "__main__":
    print("=" * 60)
    print("UR3 Inverse Kinematics Test")
    print("=" * 60)
    
    fk = UR3_URDF_ForwardKinematics()
    ik = UR3_InverseKinematics()
    
    # Test: Round-trip FK -> IK
    print("\nTest: Round-trip FK -> IK")
    test_angles = [0.5, -0.8, 1.2, -0.5, -0.9, 0.3]
    T_desired = fk.get_end_effector_transform(test_angles)
    pos = T_desired[:3, 3]
    print(f"Input angles (deg): {[f'{np.degrees(a):.1f}' for a in test_angles]}")
    print(f"FK Position: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]")
    
    solutions = ik.solve_ik(T_desired)
    print(f"\nFound {len(solutions)} IK solutions")
    
    for i, sol in enumerate(solutions):
        print(f"\nSolution {i+1}:")
        print(f"  Angles (deg): {[f'{np.degrees(a):.1f}' for a in sol]}")
        T_check = fk.get_end_effector_transform(sol)
        pos_error = np.linalg.norm(T_check[:3, 3] - T_desired[:3, 3])
        print(f"  Position error: {pos_error:.6f} m")
    
    print("\n" + "=" * 60)
