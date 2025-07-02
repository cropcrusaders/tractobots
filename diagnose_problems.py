#!/usr/bin/env python3
"""
Problem Diagnosis Script for Tractobots
Identifies specific issues and provides targeted fixes
"""

import os
import sys
import subprocess
import shutil
from pathlib import Path

def print_header(title):
    print(f"\n{'='*60}")
    print(f" {title}")
    print(f"{'='*60}")

def print_status(description, status, details="", fix=""):
    status_icon = "✅" if status else "❌"
    print(f"{status_icon} {description}")
    if details:
        print(f"    📋 {details}")
    if fix and not status:
        print(f"    🔧 Fix: {fix}")

def check_wsl_environment():
    """Check if running in WSL"""
    try:
        with open('/proc/version', 'r') as f:
            version = f.read().lower()
            return 'microsoft' in version or 'wsl' in version
    except:
        return False

def check_ros2_installation():
    """Check ROS2 installation"""
    print_header("ROS2 Installation Diagnosis")
    
    problems = []
    fixes = []
    
    # Check ROS2 command
    ros2_available = shutil.which('ros2') is not None
    print_status("ROS2 command available", ros2_available,
                "Found at: " + (shutil.which('ros2') or "Not found"),
                "./fix_all_problems.sh (installs ROS2)")
    
    if not ros2_available:
        problems.append("ROS2 not installed")
        fixes.append("Run: ./fix_all_problems.sh")
    
    # Check ROS_DISTRO
    ros_distro = os.environ.get('ROS_DISTRO')
    distro_ok = ros_distro in ['humble', 'jazzy']
    print_status("ROS_DISTRO environment variable", distro_ok,
                f"Current: {ros_distro or 'Not set'}",
                "source /opt/ros/jazzy/setup.bash")
    
    if not distro_ok:
        problems.append("ROS_DISTRO not set to supported version")
        fixes.append("Run: source /opt/ros/jazzy/setup.bash")
    
    # Check ROS2 packages directory
    ros_dir = Path(f"/opt/ros/{ros_distro}" if ros_distro else "/opt/ros/jazzy")
    ros_dir_exists = ros_dir.exists()
    print_status("ROS2 installation directory", ros_dir_exists,
                f"Path: {ros_dir}",
                "./install_ros2_jazzy_noble.sh")
    
    if not ros_dir_exists:
        problems.append("ROS2 installation directory missing")
        fixes.append("Run: ./install_ros2_jazzy_noble.sh")
    
    return problems, fixes

def check_python_environment():
    """Check Python environment"""
    print_header("Python Environment Diagnosis")
    
    problems = []
    fixes = []
    
    # Check Python version
    python_version = sys.version_info
    python_ok = python_version.major == 3 and python_version.minor >= 8
    print_status("Python version", python_ok,
                f"Version: {python_version.major}.{python_version.minor}.{python_version.micro}",
                "Install Python 3.8+")
    
    if not python_ok:
        problems.append(f"Python version {python_version.major}.{python_version.minor} too old")
        fixes.append("Install Python 3.8 or newer")
    
    # Check pip
    pip_available = shutil.which('pip3') is not None
    print_status("pip3 available", pip_available,
                "Used for installing Python packages",
                "sudo apt install python3-pip")
    
    if not pip_available:
        problems.append("pip3 not available")
        fixes.append("Run: sudo apt install python3-pip")
    
    # Check ROS2 Python packages
    ros_packages = ['rclpy', 'geometry_msgs', 'std_msgs', 'sensor_msgs']
    for package in ros_packages:
        try:
            __import__(package)
            available = True
        except ImportError:
            available = False
        
        print_status(f"Python package: {package}", available,
                    "Required for ROS2 Python nodes",
                    "Install ROS2 or source environment")
        
        if not available:
            problems.append(f"Python package {package} missing")
            fixes.append("Install ROS2 and source environment")
    
    return problems, fixes

def check_workspace_structure():
    """Check workspace structure"""
    print_header("Workspace Structure Diagnosis")
    
    problems = []
    fixes = []
    
    # Check current directory
    current_dir = Path.cwd()
    is_workspace = (current_dir / "src").exists() and (current_dir / "README.md").exists()
    print_status("In tractobots workspace", is_workspace,
                f"Current directory: {current_dir}",
                "cd to tractobots repository root")
    
    if not is_workspace:
        problems.append("Not in tractobots workspace root")
        fixes.append("Navigate to tractobots repository directory")
    
    # Check VS Code configuration
    vscode_dir = current_dir / ".vscode"
    vscode_exists = vscode_dir.exists()
    print_status("VS Code configuration", vscode_exists,
                f"Directory: {vscode_dir}",
                "VS Code files missing")
    
    if not vscode_exists:
        problems.append("VS Code configuration missing")
        fixes.append("Re-run setup scripts")
    
    # Check key files
    key_files = [
        "quick_setup.sh",
        "fix_all_problems.sh",
        "system_test.py",
        "verify_setup.py"
    ]
    
    for file_name in key_files:
        file_path = current_dir / file_name
        exists = file_path.exists()
        executable = exists and os.access(file_path, os.X_OK)
        
        print_status(f"File: {file_name}", exists,
                    f"Executable: {executable}" if exists else "Missing",
                    f"chmod +x {file_name}" if exists and not executable else "File missing")
        
        if not exists:
            problems.append(f"Missing file: {file_name}")
            fixes.append(f"Create or restore {file_name}")
        elif not executable and file_name.endswith('.sh'):
            problems.append(f"File not executable: {file_name}")
            fixes.append(f"Run: chmod +x {file_name}")
    
    return problems, fixes

def check_build_system():
    """Check build system"""
    print_header("Build System Diagnosis")
    
    problems = []
    fixes = []
    
    # Check colcon
    colcon_available = shutil.which('colcon') is not None
    print_status("colcon build tool", colcon_available,
                "Required for building ROS2 workspaces",
                "sudo apt install python3-colcon-common-extensions")
    
    if not colcon_available:
        problems.append("colcon build tool missing")
        fixes.append("Run: sudo apt install python3-colcon-common-extensions")
    
    # Check rosdep
    rosdep_available = shutil.which('rosdep') is not None
    print_status("rosdep dependency manager", rosdep_available,
                "Required for installing ROS2 dependencies",
                "sudo apt install python3-rosdep")
    
    if not rosdep_available:
        problems.append("rosdep missing")
        fixes.append("Run: sudo apt install python3-rosdep")
    
    # Check workspace directory
    ws_dir = Path.home() / "ros2_tractobots"
    ws_exists = ws_dir.exists()
    print_status("ROS2 workspace directory", ws_exists,
                f"Path: {ws_dir}",
                "Run build script to create workspace")
    
    if not ws_exists:
        problems.append("ROS2 workspace directory missing")
        fixes.append("Run: ./quick_setup.sh or ./fix_all_problems.sh")
    
    return problems, fixes

def check_gui_dependencies():
    """Check GUI dependencies"""
    print_header("GUI Dependencies Diagnosis")
    
    problems = []
    fixes = []
    
    # Check GUI packages
    gui_packages = {
        'tkinter': 'Built-in GUI (should be available)',
        'flask': 'Web dashboard',
        'PyQt5': 'Professional Qt GUI',
        'numpy': 'Data processing'
    }
    
    for package, description in gui_packages.items():
        try:
            __import__(package)
            available = True
        except ImportError:
            available = False
        
        print_status(f"GUI package: {package}", available,
                    description,
                    "pip3 install --user flask PyQt5 numpy")
        
        if not available and package != 'tkinter':
            problems.append(f"GUI package {package} missing")
            fixes.append("Run: ./install_gui.sh")
    
    return problems, fixes

def check_shapefile_dependencies():
    """Check shapefile processing dependencies"""
    print_header("Shapefile Dependencies Diagnosis")
    
    problems = []
    fixes = []
    
    # Check shapefile packages
    shapefile_packages = {
        'geopandas': 'Shapefile reading and processing',
        'shapely': 'Geometry operations',
        'pyproj': 'Coordinate transformations',
        'fiona': 'File I/O for shapefiles',
        'yaml': 'YAML configuration export'
    }
    
    for package, description in shapefile_packages.items():
        try:
            __import__(package)
            available = True
        except ImportError:
            available = False
        
        print_status(f"Shapefile package: {package}", available,
                    description,
                    "pip3 install --user geopandas shapely pyproj fiona PyYAML")
        
        if not available:
            problems.append(f"Shapefile package {package} missing")
            fixes.append("Run: pip3 install --user geopandas shapely pyproj fiona PyYAML")
    
    # Check shapefile manager exists
    shapefile_manager_path = Path("src/tractobots_mission_ui/tractobots_mission_ui/shapefile_manager.py")
    manager_exists = shapefile_manager_path.exists()
    print_status("Shapefile manager module", manager_exists,
                f"Path: {shapefile_manager_path}",
                "Shapefile integration module missing")
    
    if not manager_exists:
        problems.append("Shapefile manager module missing")
        fixes.append("Create shapefile_manager.py module")
    
    # Check field data directory
    field_data_dir = Path("field_data")
    field_dir_exists = field_data_dir.exists()
    print_status("Field data directory", field_dir_exists,
                f"Path: {field_data_dir}",
                "mkdir field_data")
    
    if not field_dir_exists:
        problems.append("Field data directory missing")
        fixes.append("Run: mkdir field_data")
    
    return problems, fixes

def generate_fix_plan(all_problems, all_fixes):
    """Generate a comprehensive fix plan"""
    print_header("🔧 COMPREHENSIVE FIX PLAN")
    
    if not any(all_problems):
        print("🎉 No problems detected! Your system appears to be working correctly.")
        print("\n✅ You can proceed with:")
        print("   - Building: ./quick_setup.sh")
        print("   - Testing: python3 verify_setup.py")
        print("   - GUI: ros2 run tractobots_mission_ui enhanced_gui")
        return
    
    print("📋 Detected Problems:")
    problem_count = 0
    for problems in all_problems:
        for problem in problems:
            problem_count += 1
            print(f"   {problem_count}. {problem}")
    
    print(f"\n🎯 Recommended Fix Sequence:")
    print("   1. 🔧 Run the comprehensive fix script:")
    print("      ./fix_all_problems.sh")
    print("")
    print("   2. ✅ Verify the fixes:")
    print("      python3 verify_setup.py")
    print("")
    print("   3. 🚀 Test the system:")
    print("      ./quick_setup.sh")
    print("")
    print("   4. 🎨 Launch GUI:")
    print("      ros2 run tractobots_mission_ui enhanced_gui")
    
    print(f"\n💡 Alternative VS Code method:")
    print("   - Press Ctrl+Shift+P")
    print("   - Type 'Tasks: Run Task'")
    print("   - Select '🔧 Fix All Problems'")
    
    print(f"\n📚 Documentation:")
    print("   - GUI_GUIDE.md - GUI usage")
    print("   - VSCODE_BUILD_GUIDE.md - VS Code workflow")
    print("   - SETUP_COMPLETE.md - Complete setup guide")

def main():
    """Main diagnosis function"""
    print("🔍 TRACTOBOTS PROBLEM DIAGNOSIS")
    print("Analyzing your setup to identify specific issues...")
    
    # Check environment
    if not check_wsl_environment():
        print("❌ This script should be run in WSL (Windows Subsystem for Linux)")
        print("💡 Open WSL terminal and try again")
        return
    
    print("✅ Running in WSL environment")
    
    # Run all checks
    all_problems = []
    all_fixes = []
    
    # Individual diagnostics
    problems, fixes = check_ros2_installation()
    all_problems.append(problems)
    all_fixes.append(fixes)
    
    problems, fixes = check_python_environment()
    all_problems.append(problems)
    all_fixes.append(fixes)
    
    problems, fixes = check_workspace_structure()
    all_problems.append(problems)
    all_fixes.append(fixes)
    
    problems, fixes = check_build_system()
    all_problems.append(problems)
    all_fixes.append(fixes)
    
    problems, fixes = check_gui_dependencies()
    all_problems.append(problems)
    all_fixes.append(fixes)
    
    problems, fixes = check_shapefile_dependencies()
    all_problems.append(problems)
    all_fixes.append(fixes)
    
    # Generate fix plan
    generate_fix_plan(all_problems, all_fixes)

if __name__ == '__main__':
    main()
