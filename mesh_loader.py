"""
UR3 Mesh Loader and Transformer

This module handles loading official UR3 STL mesh files and applying
forward kinematics transformations to each link.

Each link mesh is loaded from an STL file and transformed using the
cumulative transformation matrix T_0_i(q) computed from URDF parameters.
"""

import sys
import numpy as np
import trimesh
import os
from pathlib import Path
from urdf_forward_kinematics import UR3_URDF_ForwardKinematics


def resource_path(relative_path):
    """Get absolute path to resource, works for dev and PyInstaller."""
    if getattr(sys, 'frozen', False):
        base_path = sys._MEIPASS
    else:
        base_path = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(base_path, relative_path)


class UR3_MeshLoader:
    """
    Loads and manages UR3 robot mesh files.
    
    The UR3 robot consists of 7 visual components:
    - base: Fixed base platform
    - shoulder: Link 1 (rotates around base)
    - upper_arm: Link 2 (shoulder to elbow)
    - forearm: Link 3 (elbow to wrist)
    - wrist_1: Link 4 (first wrist joint)
    - wrist_2: Link 5 (second wrist joint)
    - wrist_3: Link 6 (third wrist joint/end-effector)
    """
    
    def __init__(self, mesh_directory=None):
        """
        Initialize mesh loader.
        
        Args:
            mesh_directory: Path to directory containing STL files
        """
        if mesh_directory is None:
            mesh_directory = resource_path('meshes')
        self.mesh_dir = Path(mesh_directory)
        
        # Define mesh file names (standard UR3 naming convention)
        self.mesh_files = {
            'base': 'base.stl',
            'shoulder': 'shoulder.stl',
            'upper_arm': 'upperarm.stl',
            'forearm': 'forearm.stl',
            'wrist_1': 'wrist1.stl',
            'wrist_2': 'wrist2.stl',
            'wrist_3': 'wrist3.stl'
        }
        
        # Store loaded meshes
        self.meshes = {}
        
        # TEST: Disable visual origins to see if meshes are already properly aligned
        # If meshes connect without offsets, then the issue is with how we apply offsets
        # If meshes still disconnect, then meshes themselves need adjustment
        
        self.mesh_offsets = {
            'base': np.eye(4),
            'shoulder': np.eye(4),
            'upper_arm': np.eye(4),
            'forearm': np.eye(4),
            'wrist_1': np.eye(4),
            'wrist_2': np.eye(4),
            'wrist_3': np.eye(4)
        }
    
    def _create_transform(self, translation, rotation_rpy):
        """
        Create a 4x4 transformation matrix from translation and RPY rotation.
        
        Args:
            translation: [x, y, z] translation
            rotation_rpy: [roll, pitch, yaw] in radians
            
        Returns:
            4x4 transformation matrix
        """
        T = np.eye(4)
        
        # Rotation matrix from RPY
        roll, pitch, yaw = rotation_rpy
        
        # Roll (X)
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])
        
        # Pitch (Y)
        Ry = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])
        
        # Yaw (Z)
        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        # Combined rotation
        R = Rz @ Ry @ Rx
        
        T[:3, :3] = R
        T[:3, 3] = translation
        
        return T
        
    def load_meshes(self):
        """
        Load all UR3 mesh files from the mesh directory.
        
        Returns:
            bool: True if all meshes loaded successfully
        """
        if not self.mesh_dir.exists():
            print(f"Mesh directory not found: {self.mesh_dir}")
            print("Please download UR3 STL files and place them in the 'meshes' folder")
            return False
        
        all_loaded = True
        
        for link_name, filename in self.mesh_files.items():
            filepath = self.mesh_dir / filename
            
            if filepath.exists():
                try:
                    mesh = trimesh.load(str(filepath))
                    
                    # Convert to trimesh if it's a Scene
                    if isinstance(mesh, trimesh.Scene):
                        mesh = mesh.dump(concatenate=True)
                    
                    # DON'T center meshes - preserve their coordinate systems
                    # Meshes are designed with specific origins for joint connections
                    
                    self.meshes[link_name] = mesh
                    print(f"[OK] Loaded {link_name}: {filename}")
                    
                except Exception as e:
                    print(f"[ERROR] Failed to load {link_name}: {e}")
                    all_loaded = False
            else:
                print(f"[ERROR] File not found: {filename}")
                all_loaded = False
        
        return all_loaded
    
    def get_mesh(self, link_name):
        """
        Get a mesh for a specific link.
        
        Args:
            link_name: Name of the link ('base', 'shoulder', etc.)
            
        Returns:
            trimesh.Trimesh: The mesh object, or None if not loaded
        """
        return self.meshes.get(link_name)
    
    def get_all_meshes(self):
        """
        Get all loaded meshes.
        
        Returns:
            dict: Dictionary of link_name -> trimesh.Trimesh
        """
        return self.meshes
    
    def apply_transform_to_mesh(self, mesh, transform_matrix, mesh_name=''):
        """
        Apply FK transformation to mesh.
        Currently testing without visual origins to isolate the problem.
        """
        mesh_copy = mesh.copy()
        
        # Simple direct application of FK transform
        if mesh_name in self.mesh_offsets:
            final_transform = transform_matrix @ self.mesh_offsets[mesh_name]
        else:
            final_transform = transform_matrix
        
        mesh_copy.apply_transform(final_transform)
        return mesh_copy
    
    def create_fallback_meshes(self):
        """
        Create simple geometric meshes as fallback if STL files are not available.
        
        This generates approximate UR3 geometry for visualization purposes.
        """
        print("\nCreating fallback geometric meshes...")
        
        # Base
        base = trimesh.creation.cylinder(
            radius=0.09,
            height=0.05,
            sections=32
        )
        base.apply_translation([0, 0, 0.025])
        self.meshes['base'] = base
        
        # Shoulder (large joint)
        shoulder = trimesh.creation.uv_sphere(
            radius=0.065,
            count=[32, 32]
        )
        shoulder.apply_translation([0, 0, 0.15])
        self.meshes['shoulder'] = shoulder
        
        # Upper arm (link 2)
        upper_arm = trimesh.creation.cylinder(
            radius=0.035,
            height=0.24,
            sections=32
        )
        # Rotate to align with link direction
        rot = trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0])
        upper_arm.apply_transform(rot)
        upper_arm.apply_translation([0.12, 0, 0.15])
        self.meshes['upper_arm'] = upper_arm
        
        # Forearm (link 3)
        forearm = trimesh.creation.cylinder(
            radius=0.030,
            height=0.21,
            sections=32
        )
        rot = trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0])
        forearm.apply_transform(rot)
        forearm.apply_translation([0.105, 0, 0.15])
        self.meshes['forearm'] = forearm
        
        # Wrist joints (smaller spheres)
        for i, name in enumerate(['wrist_1', 'wrist_2', 'wrist_3']):
            wrist = trimesh.creation.uv_sphere(
                radius=0.040 - i*0.005,
                count=[24, 24]
            )
            self.meshes[name] = wrist
        
        print("[OK] Fallback meshes created")
        return True


class UR3_MeshVisualizer:
    """
    Manages visualization of UR3 meshes with forward kinematics.
    
    This class:
    1. Loads UR3 mesh files via MeshLoader
    2. Computes forward kinematics transformations using URDF
    3. Applies transformations to each link mesh
    4. Prepares meshes for rendering in visualization engine
    """
    
    def __init__(self, forward_kinematics, mesh_loader):
        """
        Initialize mesh visualizer.
        
        Args:
            forward_kinematics: UR3_URDF_ForwardKinematics or UR3_ForwardKinematics instance
            mesh_loader: UR3_MeshLoader instance with loaded meshes
        """
        self.fk = forward_kinematics
        self.mesh_loader = mesh_loader
        
        # Link assignment (which mesh corresponds to which joint)
        self.link_mesh_mapping = {
            'base': 'base',           # Base is fixed
            0: 'shoulder',            # Joint 1
            1: 'upper_arm',           # Joint 2
            2: 'forearm',             # Joint 3
            3: 'wrist_1',             # Joint 4
            4: 'wrist_2',             # Joint 5
            5: 'wrist_3'              # Joint 6
        }
    
    def get_link_transforms(self, joint_angles):
        """
        Compute transformation matrices for each link using URDF FK.
        
        Args:
            joint_angles: Array of 6 joint angles [q1, q2, q3, q4, q5, q6]
            
        Returns:
            dict: Dictionary of link transforms
        """
        # Use URDF FK if available, otherwise fallback to DH
        if hasattr(self.fk, 'forward_kinematics'):
            # URDF FK
            urdf_transforms = self.fk.forward_kinematics(joint_angles)
            # Map to same format as before
            transforms = {
                'base': urdf_transforms['base'],
                0: urdf_transforms['shoulder'],
                1: urdf_transforms['upper_arm'],
                2: urdf_transforms['forearm'],
                3: urdf_transforms['wrist_1'],
                4: urdf_transforms['wrist_2'],
                5: urdf_transforms['wrist_3']
            }
        else:
            # Fallback to DH FK
            q = np.array(joint_angles)
            T1 = self.fk.dh_transform(0, self.fk.d1, np.pi/2, q[0])
            T2 = self.fk.dh_transform(self.fk.a2, 0, 0, q[1])
            T3 = self.fk.dh_transform(self.fk.a3, 0, 0, q[2])
            T4 = self.fk.dh_transform(0, self.fk.d4, np.pi/2, q[3])
            T5 = self.fk.dh_transform(0, self.fk.d5, -np.pi/2, q[4])
            T6 = self.fk.dh_transform(0, self.fk.d6, 0, q[5])
            
            transforms = {
                'base': np.eye(4),
                0: T1,
                1: T1 @ T2,
                2: T1 @ T2 @ T3,
                3: T1 @ T2 @ T3 @ T4,
                4: T1 @ T2 @ T3 @ T4 @ T5,
                5: T1 @ T2 @ T3 @ T4 @ T5 @ T6
            }
        
        return transforms
    
    def get_transformed_meshes(self, joint_angles):
        """
        Get all link meshes transformed to their positions for given joint angles.
        
        This is the core method that applies FK transformations to meshes.
        
        Args:
            joint_angles: Array of 6 joint angles
            
        Returns:
            dict: Dictionary of {link_name: transformed_mesh}
        """
        transforms = self.get_link_transforms(joint_angles)
        transformed_meshes = {}
        
        for link_id, mesh_name in self.link_mesh_mapping.items():
            mesh = self.mesh_loader.get_mesh(mesh_name)
            
            if mesh is not None:
                # Get the transformation for this link
                transform = transforms[link_id]
                
                # Apply transformation to mesh (with proper offset)
                transformed_mesh = self.mesh_loader.apply_transform_to_mesh(
                    mesh, transform, mesh_name
                )
                
                transformed_meshes[mesh_name] = transformed_mesh
        
        return transformed_meshes
    
    def get_combined_mesh(self, joint_angles):
        """
        Get a single combined mesh of the entire robot configuration.
        
        Args:
            joint_angles: Array of 6 joint angles
            
        Returns:
            trimesh.Trimesh: Combined mesh of all links
        """
        transformed_meshes = self.get_transformed_meshes(joint_angles)
        
        if not transformed_meshes:
            return None
        
        # Combine all meshes into one
        mesh_list = list(transformed_meshes.values())
        combined = trimesh.util.concatenate(mesh_list)
        
        return combined


def download_ur3_meshes_info():
    """
    Provide information on how to obtain official UR3 mesh files.
    """
    info = """
    ═══════════════════════════════════════════════════════════════
    HOW TO OBTAIN UR3 MESH FILES
    ═══════════════════════════════════════════════════════════════
    
    Option 1: Official Universal Robots ROS Package
    ------------------------------------------------
    1. Download from: https://github.com/ros-industrial/universal_robot
    2. Navigate to: universal_robot/ur_description/meshes/ur3/visual/
    3. Copy all STL files to the 'meshes' folder
    
    Files needed:
    - base.stl
    - shoulder.stl
    - upperarm.stl
    - forearm.stl
    - wrist1.stl
    - wrist2.stl
    - wrist3.stl
    
    Option 2: Alternative Sources
    ------------------------------
    - GrabCAD: Search for "UR3 robot"
    - Universal Robots Support: Contact for official CAD models
    - ROS-Industrial GitHub repositories
    
    Option 3: Use Fallback Geometry
    --------------------------------
    If STL files are unavailable, the system will automatically
    generate approximate geometric shapes (cylinders and spheres)
    for visualization purposes.
    
    ═══════════════════════════════════════════════════════════════
    """
    return info


if __name__ == '__main__':
    # Demo: Load and test mesh system
    print(download_ur3_meshes_info())
    
    loader = UR3_MeshLoader()
    
    if not loader.load_meshes():
        print("\nSTL files not found. Creating fallback meshes...")
        loader.create_fallback_meshes()
    
    print(f"\n✓ Loaded {len(loader.meshes)} meshes")
    
    # Test transformation
    from forward_kinematics import UR3_ForwardKinematics
    
    fk = UR3_ForwardKinematics()
    visualizer = UR3_MeshVisualizer(fk, loader)
    
    # Test with zero angles
    test_angles = [0, 0, 0, 0, 0, 0]
    transforms = visualizer.get_link_transforms(test_angles)
    
    print(f"\n✓ Computed {len(transforms)} link transformations")
    print("\nEnd-effector transform:")
    print(transforms[5])
