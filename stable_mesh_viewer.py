"""
UR3 Interactive Viewer - Stable version with PyQtGraph
Fixed: Connection issues and window stability
"""

import sys
import os
import numpy as np
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph.opengl as gl

from mesh_loader import UR3_MeshLoader, UR3_MeshVisualizer
from urdf_forward_kinematics import UR3_URDF_ForwardKinematics
from inverse_kinematics import UR3_InverseKinematics


def resource_path(relative_path):
    """Get absolute path to resource, works for dev and PyInstaller."""
    if getattr(sys, 'frozen', False):
        base_path = sys._MEIPASS
    else:
        base_path = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(base_path, relative_path)


class StableUR3Viewer(QtWidgets.QMainWindow):
    """Stable UR3 viewer with proper mesh connections."""
    
    def __init__(self):
        super().__init__()
        
        self.setWindowTitle("UR3 Robot Controller")
        self.setGeometry(100, 100, 1400, 800)
        
        # Set window icon
        icon_path = resource_path('app_icon.ico')
        if os.path.exists(icon_path):
            self.setWindowIcon(QtGui.QIcon(icon_path))
        
        # Initialize with URDF FK (matches ROS exactly)
        print("Initializing UR3 mesh system with URDF FK...")
        self.fk = UR3_URDF_ForwardKinematics()
        self.ik = UR3_InverseKinematics()
        self.mesh_loader = UR3_MeshLoader()
        
        # IK solutions
        self.ik_solutions = []
        self.current_solution_idx = 0
        
        if not self.mesh_loader.load_meshes():
            print("Failed to load meshes!")
            sys.exit(1)
        
        self.mesh_visualizer = UR3_MeshVisualizer(self.fk, self.mesh_loader)
        
        # Initial pose - Ready position
        self.joint_angles = [0.0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0.0]
        
        # Robot mesh items
        self.robot_items = []
        
        # Setup UI
        self.init_ui()
        
        # Initial draw
        self.update_robot()
        
    def init_ui(self):
        """Setup user interface."""
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        
        main_layout = QtWidgets.QHBoxLayout(central)
        
        # Control panel (left)
        control_panel = self.create_control_panel()
        main_layout.addWidget(control_panel, stretch=1)
        
        # 3D view container (right)
        view_container = QtWidgets.QWidget()
        view_layout = QtWidgets.QVBoxLayout(view_container)
        view_layout.setContentsMargins(0, 0, 0, 0)
        
        # 3D view
        self.view = gl.GLViewWidget()
        self.view.setBackgroundColor((240, 242, 245))
        self.view.setCameraPosition(distance=1.2, elevation=25, azimuth=135)
        view_layout.addWidget(self.view)
        
        # Position overlay (top-right corner)
        self.pos_overlay = QtWidgets.QLabel()
        self.pos_overlay.setStyleSheet("""
            background: rgba(255, 255, 255, 220);
            padding: 10px;
            border: 2px solid #66BB6A;
            border-radius: 6px;
            font-family: 'Consolas', monospace;
            font-size: 9pt;
            color: #2E7D32;
        """)
        self.pos_overlay.setParent(self.view)
        self.pos_overlay.move(10, 10)
        
        main_layout.addWidget(view_container, stretch=3)
        
        # Add environment
        self.setup_scene()
        
        # Initial position display
        self.update_position_display()
        
    def create_control_panel(self):
        """Create control panel with sliders."""
        panel = QtWidgets.QWidget()
        panel.setMaximumWidth(300)
        panel.setStyleSheet("""
            QWidget {
                background-color: #ffffff;
                font-family: 'Segoe UI', Arial;
            }
            QGroupBox {
                border: 1px solid #e0e0e0;
                border-radius: 6px;
                margin-top: 8px;
                padding-top: 8px;
                background-color: #fafafa;
            }
            QGroupBox::title {
                color: #424242;
                font-weight: 600;
                font-size: 9pt;
            }
        """)
        layout = QtWidgets.QVBoxLayout(panel)
        
        # Title
        title = QtWidgets.QLabel("Joint Control")
        title.setFont(QtGui.QFont("Segoe UI", 12, QtGui.QFont.Bold))
        title.setAlignment(QtCore.Qt.AlignCenter)
        title.setStyleSheet("""
            color: #1976D2;
            padding: 6px;
            background: #E3F2FD;
            border-radius: 4px;
            margin-bottom: 4px;
        """)
        layout.addWidget(title)
        
        # Sliders
        self.sliders = []
        self.labels = []
        
        joint_names = ["Base", "Shoulder", "Elbow", "Wrist 1", "Wrist 2", "Wrist 3"]
        
        for i, name in enumerate(joint_names):
            group = QtWidgets.QGroupBox(f"J{i+1}: {name}")
            group_layout = QtWidgets.QVBoxLayout()
            group_layout.setSpacing(2)
            group_layout.setContentsMargins(6, 4, 6, 4)
            
            # Slider with value in same row
            slider_layout = QtWidgets.QHBoxLayout()
            
            # Slider
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setMinimum(-3600)
            slider.setMaximum(3600)
            slider.setValue(int(np.degrees(self.joint_angles[i]) * 10))
            slider.setStyleSheet("""
                QSlider::groove:horizontal {
                    height: 4px;
                    background: #E0E0E0;
                    border-radius: 2px;
                }
                QSlider::handle:horizontal {
                    background: #1976D2;
                    width: 12px;
                    height: 12px;
                    margin: -4px 0;
                    border-radius: 6px;
                }
                QSlider::handle:horizontal:hover {
                    background: #1565C0;
                }
            """)
            slider.valueChanged.connect(lambda v, idx=i: self.on_slider_changed(idx, v))
            self.sliders.append(slider)
            slider_layout.addWidget(slider, stretch=3)
            
            # Value label
            label = QtWidgets.QLabel(f"{np.degrees(self.joint_angles[i]):.1f}°")
            label.setAlignment(QtCore.Qt.AlignCenter)
            label.setFixedWidth(55)
            label.setStyleSheet("""
                font-weight: bold;
                font-size: 9pt;
                color: #1976D2;
                background: #E3F2FD;
                border-radius: 3px;
                padding: 2px;
            """)
            self.labels.append(label)
            slider_layout.addWidget(label)
            
            group_layout.addLayout(slider_layout)
            group.setLayout(group_layout)
            layout.addWidget(group)
        
        # Buttons
        btn_layout = QtWidgets.QHBoxLayout()
        
        home_btn = QtWidgets.QPushButton("Home")
        home_btn.setFixedHeight(38)
        home_btn.setMinimumWidth(100)
        home_btn.setStyleSheet("""
            QPushButton {
                background-color: #1E88E5;
                color: white;
                padding: 8px 20px;
                border: 1px solid #1565C0;
                border-radius: 4px;
                font-weight: bold;
                font-size: 11pt;
                font-family: 'Segoe UI', Arial;
            }
            QPushButton:hover {
                background-color: #1976D2;
                border: 1px solid #0D47A1;
            }
            QPushButton:pressed {
                background-color: #1565C0;
            }
        """)
        home_btn.clicked.connect(lambda: self.load_preset([0, 0, 0, 0, 0, 0]))
        btn_layout.addWidget(home_btn)
        
        ready_btn = QtWidgets.QPushButton("Ready")
        ready_btn.setFixedHeight(38)
        ready_btn.setMinimumWidth(100)
        ready_btn.setStyleSheet("""
            QPushButton {
                background-color: #43A047;
                color: white;
                padding: 8px 20px;
                border: 1px solid #388E3C;
                border-radius: 4px;
                font-weight: bold;
                font-size: 11pt;
                font-family: 'Segoe UI', Arial;
            }
            QPushButton:hover {
                background-color: #388E3C;
                border: 1px solid #2E7D32;
            }
            QPushButton:pressed {
                background-color: #2E7D32;
            }
        """)
        ready_btn.clicked.connect(lambda: self.load_preset([0, -90, 90, -90, -90, 0]))
        btn_layout.addWidget(ready_btn)
        
        layout.addLayout(btn_layout)
        
        # IK Solutions panel
        ik_panel = QtWidgets.QGroupBox("IK Solutions")
        ik_panel.setStyleSheet("""
            QGroupBox {
                border: 2px solid #FF9800;
                border-radius: 8px;
                margin-top: 8px;
                padding-top: 12px;
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    stop:0 #FFF3E0, stop:1 #FFE0B2);
            }
            QGroupBox::title {
                color: #E65100;
                font-weight: 700;
                font-size: 10pt;
            }
        """)
        ik_layout = QtWidgets.QVBoxLayout()
        ik_layout.setSpacing(8)
        
        # Calculate IK button
        calc_ik_btn = QtWidgets.QPushButton("Calculate IK")
        calc_ik_btn.setFixedHeight(30)
        calc_ik_btn.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #FF9800, stop:1 #F57C00);
                color: white;
                border: none;
                border-radius: 6px;
                font-weight: 600;
                font-size: 10pt;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #FB8C00, stop:1 #EF6C00);
            }
            QPushButton:pressed {
                background: #E65100;
            }
        """)
        calc_ik_btn.clicked.connect(self.calculate_ik_solutions)
        ik_layout.addWidget(calc_ik_btn)
        
        # Solutions count label
        self.ik_count_label = QtWidgets.QLabel("Solutions: 0")
        self.ik_count_label.setStyleSheet("""
            font-size: 10pt;
            font-weight: bold;
            color: #E65100;
            padding: 4px 0px;
        """)
        ik_layout.addWidget(self.ik_count_label)
        
        # Scrollable solutions list
        scroll_area = QtWidgets.QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setMaximumHeight(200)
        scroll_area.setStyleSheet("""
            QScrollArea {
                border: 1px solid #FF9800;
                border-radius: 4px;
                background: white;
            }
        """)
        
        self.solutions_widget = QtWidgets.QWidget()
        self.solutions_layout = QtWidgets.QVBoxLayout(self.solutions_widget)
        self.solutions_layout.setSpacing(2)
        self.solutions_layout.setContentsMargins(4, 4, 4, 4)
        self.solutions_layout.addStretch()
        
        scroll_area.setWidget(self.solutions_widget)
        ik_layout.addWidget(scroll_area)
        
        # Radio button group for solution selection
        self.solution_buttons = QtWidgets.QButtonGroup()
        self.solution_buttons.buttonClicked.connect(self.on_solution_selected)
        
        ik_panel.setLayout(ik_layout)
        layout.addWidget(ik_panel)
        
        layout.addStretch()
        
        return panel
        
    def setup_scene(self):
        """Setup scene with grid and axes."""
        # Grid
        grid = gl.GLGridItem()
        grid.setSize(x=2, y=2)
        grid.setSpacing(x=0.1, y=0.1)
        grid.setColor((180, 180, 180, 255))
        self.view.addItem(grid)
        
        # World frame
        axis_size = 0.2
        # X - Red
        x_line = gl.GLLinePlotItem(
            pos=np.array([[0, 0, 0], [axis_size, 0, 0]]),
            color=(1, 0, 0, 1), width=3, antialias=True
        )
        self.view.addItem(x_line)
        
        # Y - Green
        y_line = gl.GLLinePlotItem(
            pos=np.array([[0, 0, 0], [0, axis_size, 0]]),
            color=(0, 1, 0, 1), width=3, antialias=True
        )
        self.view.addItem(y_line)
        
        # Z - Blue
        z_line = gl.GLLinePlotItem(
            pos=np.array([[0, 0, 0], [0, 0, axis_size]]),
            color=(0, 0, 1, 1), width=3, antialias=True
        )
        self.view.addItem(z_line)
        
    def on_slider_changed(self, joint_idx, value):
        """Handle slider change."""
        angle_deg = value / 10.0
        angle_rad = np.radians(angle_deg)
        
        self.joint_angles[joint_idx] = angle_rad
        self.labels[joint_idx].setText(f"{angle_deg:.1f}°")
        
        self.update_robot()
        self.update_position_display()
        
    def load_preset(self, angles_deg):
        """Load preset angles."""
        for i, angle_deg in enumerate(angles_deg):
            self.sliders[i].setValue(int(angle_deg * 10))
    
    def update_position_display(self):
        """Update end effector position display in 3D overlay."""
        transforms = self.fk.forward_kinematics(self.joint_angles)
        pos = transforms['wrist_3'][:3, 3]
        
        text = f"""<b style='color:#1B5E20;font-size:10pt;'>End Effector</b><br>
<span style='color:#2E7D32;'><b>X:</b> {pos[0]:6.3f} m</span><br>
<span style='color:#2E7D32;'><b>Y:</b> {pos[1]:6.3f} m</span><br>
<span style='color:#2E7D32;'><b>Z:</b> {pos[2]:6.3f} m</span>"""
        
        self.pos_overlay.setText(text)
        
    def update_robot(self):
        """Update robot visualization."""
        # Clear old items
        for item in self.robot_items:
            self.view.removeItem(item)
        self.robot_items.clear()
        
        # Get transformed meshes
        transformed_meshes = self.mesh_visualizer.get_transformed_meshes(self.joint_angles)
        
        # Colors - Professional UR3 blue/gray scheme
        colors = {
            'base': (0.25, 0.27, 0.30, 1.0),
            'shoulder': (0.15, 0.45, 0.75, 1.0),
            'upper_arm': (0.88, 0.90, 0.92, 1.0),
            'forearm': (0.88, 0.90, 0.92, 1.0),
            'wrist_1': (0.15, 0.45, 0.75, 1.0),
            'wrist_2': (0.15, 0.45, 0.75, 1.0),
            'wrist_3': (0.25, 0.27, 0.30, 1.0),
        }
        
        # Add meshes
        for mesh_name, trimesh_obj in transformed_meshes.items():
            vertices = np.asarray(trimesh_obj.vertices)
            faces = np.asarray(trimesh_obj.faces)
            
            # Create mesh data
            mesh_data = gl.MeshData(vertexes=vertices, faces=faces)
            
            # Create mesh item
            mesh_item = gl.GLMeshItem(
                meshdata=mesh_data,
                smooth=True,
                color=colors.get(mesh_name, (0.7, 0.7, 0.7, 1.0)),
                shader='normalColor',
                glOptions='opaque'
            )
            
            self.view.addItem(mesh_item)
            self.robot_items.append(mesh_item)
        
        # Add joint frames
        joint_positions = self.fk.get_joint_positions(self.joint_angles)
        frame_size = 0.06
        
        for i, pos in enumerate(joint_positions[1:]):  # Skip base
            # X axis - Red
            x_line = gl.GLLinePlotItem(
                pos=np.array([pos, pos + [frame_size, 0, 0]]),
                color=(1, 0, 0, 0.8), width=2, antialias=True
            )
            self.view.addItem(x_line)
            self.robot_items.append(x_line)
            
            # Y axis - Green
            y_line = gl.GLLinePlotItem(
                pos=np.array([pos, pos + [0, frame_size, 0]]),
                color=(0, 1, 0, 0.8), width=2, antialias=True
            )
            self.view.addItem(y_line)
            self.robot_items.append(y_line)
            
            # Z axis - Blue
            z_line = gl.GLLinePlotItem(
                pos=np.array([pos, pos + [0, 0, frame_size]]),
                color=(0, 0, 1, 0.8), width=2, antialias=True
            )
            self.view.addItem(z_line)
            self.robot_items.append(z_line)
    
    def calculate_ik_solutions(self):
        """Calculate IK solutions for current end-effector pose."""
        # Get current end-effector transform
        transforms = self.fk.forward_kinematics(self.joint_angles)
        T_current = transforms['wrist_3']
        
        # Solve IK
        try:
            self.ik_solutions = self.ik.solve_ik(T_current)
            
            # Update count
            self.ik_count_label.setText(f"Solutions: {len(self.ik_solutions)}")
            
            # Clear old solution buttons
            for button in self.solution_buttons.buttons():
                self.solution_buttons.removeButton(button)
                button.deleteLater()
            
            # Clear layout
            while self.solutions_layout.count() > 1:
                item = self.solutions_layout.takeAt(0)
                if item.widget():
                    item.widget().deleteLater()
            
            # Add new solution radio buttons
            if len(self.ik_solutions) > 0:
                for i, solution in enumerate(self.ik_solutions):
                    angles_deg = [np.degrees(a) for a in solution]
                    text = f"Sol {i+1}: [{angles_deg[0]:.0f}, {angles_deg[1]:.0f}, {angles_deg[2]:.0f}, {angles_deg[3]:.0f}, {angles_deg[4]:.0f}, {angles_deg[5]:.0f}]"
                    
                    radio = QtWidgets.QRadioButton(text)
                    radio.setStyleSheet("""
                        QRadioButton {
                            font-size: 8pt;
                            font-family: 'Consolas', monospace;
                            color: #E65100;
                            padding: 2px;
                        }
                        QRadioButton:hover {
                            background: #FFF3E0;
                        }
                        QRadioButton:checked {
                            font-weight: bold;
                            background: #FFE0B2;
                        }
                    """)
                    
                    self.solution_buttons.addButton(radio, i)
                    self.solutions_layout.insertWidget(i, radio)
                
                self.solution_buttons.button(0).setChecked(True)
                print(f"\nFound {len(self.ik_solutions)} IK solutions")
            else:
                no_label = QtWidgets.QLabel("No solutions found")
                no_label.setStyleSheet("color: #E65100; padding: 4px;")
                self.solutions_layout.insertWidget(0, no_label)
                print("\nNo IK solutions found")
                
        except Exception as e:
            print(f"\nIK error: {e}")
            import traceback
            traceback.print_exc()
            self.ik_count_label.setText("Solutions: Error")
    
    def on_solution_selected(self, button):
        """Handle solution selection from radio button."""
        idx = self.solution_buttons.id(button)
        if idx >= 0 and idx < len(self.ik_solutions):
            solution = self.ik_solutions[idx]
            for i, angle_rad in enumerate(solution):
                angle_deg = np.degrees(angle_rad)
                self.sliders[i].setValue(int(angle_deg * 10))
            print(f"\nSwitched to Solution {idx+1}")


def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle('Fusion')
    
    # Dark palette
    palette = QtGui.QPalette()
    palette.setColor(QtGui.QPalette.Window, QtGui.QColor(240, 244, 248))
    palette.setColor(QtGui.QPalette.WindowText, QtCore.Qt.black)
    app.setPalette(palette)
    
    print("\n" + "="*70)
    print("UR3 INTERACTIVE MESH VIEWER")
    print("="*70)
    
    window = StableUR3Viewer()
    window.show()
    
    print("\n✓ Application started successfully!")
    print("  Use sliders to control robot joints")
    print("  'Home' button: Reset to zero")
    print("  'Ready' button: Standard pose")
    print("="*70 + "\n")
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
