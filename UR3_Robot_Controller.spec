# -*- mode: python ; coding: utf-8 -*-
"""PyInstaller spec file for UR3 Robot Controller"""

import os

block_cipher = None
base_dir = os.path.dirname(os.path.abspath(SPEC))

a = Analysis(
    ['stable_mesh_viewer.py'],
    pathex=[base_dir],
    binaries=[],
    datas=[
        ('meshes/*.stl', 'meshes'),
        ('app_icon.ico', '.'),
    ],
    hiddenimports=[
        'pyqtgraph.opengl',
        'pyqtgraph.opengl.items',
        'pyqtgraph.opengl.items.GLMeshItem',
        'pyqtgraph.opengl.items.GLGridItem',
        'pyqtgraph.opengl.items.GLAxisItem',
        'pyqtgraph.opengl.GLViewWidget',
        'OpenGL.platform.win32',
        'OpenGL.arrays.numpymodule',
        'OpenGL.arrays.ctypesarrays',
        'OpenGL.arrays.ctypesparameters',
        'OpenGL.arrays.ctypespointers',
        'OpenGL.arrays.strings',
        'OpenGL.arrays.numbers',
        'OpenGL.arrays.formathandler',
        'trimesh',
        'trimesh.exchange',
        'trimesh.exchange.stl',
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[
        'matplotlib', 'scipy', 'pandas', 'tkinter',
        'IPython', 'jupyter', 'notebook',
    ],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.zipfiles,
    a.datas,
    [],
    name='UR3_Robot_Controller',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,          # No console window (GUI app)
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon='app_icon.ico',    # Embedded icon
)
