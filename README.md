# UR3 Robot Visualization & IK Solver

Interactive 3D visualization and inverse kinematics solver for the **Universal Robots UR3** robotic arm.  
Built with Python, PyQt5, and OpenGL — no ROS required.

![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)
![Platform](https://img.shields.io/badge/Platform-Windows-lightgrey.svg)
![License](https://img.shields.io/badge/License-MIT-green.svg)

---

## Features

- **Realistic 3D Mesh Rendering** — STL meshes from official UR3 URDF
- **URDF-based Forward Kinematics** — exact match with ROS `ur_description`
- **Numerical IK Solver** — multi-start optimization, finds up to 8 distinct solutions
- **Interactive Joint Control** — real-time sliders for all 6 joints
- **IK Solutions Panel** — browse and apply all valid IK solutions
- **End Effector Overlay** — live XYZ position display on 3D viewport
- **Preset Poses** — Home (zero position) and Ready position
- **Standalone .exe** — can be packaged into a single portable executable
- **Mouse Controls:**
  - Left drag → Rotate
  - Right drag → Pan
  - Scroll → Zoom

---

## Quick Start

### 1. Clone the repository

```bash
git clone https://github.com/mranv/UR3-Robot-IK-Visualization.git
cd UR3-Robot-IK-Visualization
```

### 2. Install dependencies

**Option A — One-click (Windows):**  
Double-click `install.bat`

**Option B — Manual:**

```bash
pip install -r requirements.txt
```

### 3. Run the app

**Option A — One-click (Windows):**  
Double-click `run.bat`

**Option B — Terminal:**

```bash
python stable_mesh_viewer.py
```

### 4. (Optional) Create desktop shortcut

Double-click `install_shortcut.bat` to add a **UR3 Robot Controller** shortcut to your Desktop.

- Works on any Windows PC — automatically detects the current user's Desktop folder
- The shortcut uses the included `app_icon.ico` (blue robot head)
- Launches the app via `run.bat` (no terminal window stays open)

### 5. (Optional) Build standalone .exe

Double-click `build_exe.bat` to package everything into a single `UR3_Robot_Controller.exe`.  
No Python installation required to run the .exe.

---

## Requirements

- Python 3.8+ (for running from source)
- Windows 10/11

| Package | Purpose |
|---------|---------|
| PyQt5 | GUI framework |
| pyqtgraph | 3D OpenGL visualization |
| PyOpenGL | OpenGL bindings |
| numpy | Numerical computing |
| trimesh | STL mesh loading |

All dependencies are listed in `requirements.txt` and auto-installed via `install.bat`.

---

## Project Structure

```
├── stable_mesh_viewer.py        # Main application (GUI + 3D viewer)
├── urdf_forward_kinematics.py   # URDF-based forward kinematics
├── inverse_kinematics.py        # Numerical IK solver (multi-start)
├── mesh_loader.py               # STL mesh loading & transformation
├── create_icon.py               # App icon generator
├── meshes/                      # UR3 STL mesh files
│   ├── base.stl
│   ├── shoulder.stl
│   ├── upperarm.stl
│   ├── forearm.stl
│   ├── wrist1.stl
│   ├── wrist2.stl
│   └── wrist3.stl
├── app_icon.ico                 # Application icon
├── requirements.txt             # Python dependencies
├── install.bat                  # One-click dependency installer
├── install_shortcut.bat         # Creates desktop shortcut
├── run.bat                      # One-click app launcher
├── build_exe.bat                # One-click .exe builder
├── UR3_Robot_Controller.spec    # PyInstaller build config
├── .gitignore
└── README.md
```

---

## How It Works

### Forward Kinematics
Uses URDF parameters from the official UR3 description:
- Joint axes: Z, Y, Y, Y, Z, Y

### Inverse Kinematics
Numerical multi-start solver using damped least squares (Levenberg-Marquardt):
- 32 initial seeds (8 predefined + 24 random configurations)
- 6×6 numerical Jacobian with axis-angle orientation error
- Finds up to **8 distinct solutions** with near-zero position error
- All solutions verified against UR3 joint limits

---

## License

MIT License — free for personal and educational use.
