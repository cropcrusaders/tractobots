#!/usr/bin/env python3
"""
Comprehensive setup verification script for Tractobots VS Code environment.
Run this script to verify that your development environment is properly configured.
"""

import os
import sys
import subprocess
import json
from pathlib import Path

def print_header(title):
    """Print a formatted header."""
    print(f"\n{'='*60}")
    print(f" {title}")
    print(f"{'='*60}")

def print_check(description, success, details=""):
    """Print a check result."""
    status = "✅ PASS" if success else "❌ FAIL"
    print(f"{status} {description}")
    if details:
        print(f"    {details}")

def check_ros2_installation():
    """Check if ROS2 is properly installed."""
    print_header("ROS2 Installation Check")
    
    try:
        # Check if ROS2 distro is set
        ros_distro = os.environ.get('ROS_DISTRO')
        print_check("ROS_DISTRO environment variable", 
                   ros_distro == 'humble', 
                   f"Found: {ros_distro}")
        
        # Check ROS2 installation path
        ros_root = os.environ.get('ROS_ROOT', '/opt/ros/humble')
        ros_exists = os.path.exists(ros_root)
        print_check("ROS2 installation directory", 
                   ros_exists, 
                   f"Path: {ros_root}")
        
        # Try to import ROS2 packages
        try:
            import rclpy
            print_check("rclpy import", True, f"Version: {rclpy.__version__}")
        except ImportError as e:
            print_check("rclpy import", False, f"Error: {e}")
        
        try:
            import geometry_msgs
            print_check("geometry_msgs import", True)
        except ImportError as e:
            print_check("geometry_msgs import", False, f"Error: {e}")
            
        try:
            from std_msgs import msg
            print_check("std_msgs import", True)
        except ImportError as e:
            print_check("std_msgs import", False, f"Error: {e}")
            
    except Exception as e:
        print_check("ROS2 environment check", False, f"Error: {e}")

def check_workspace_structure():
    """Check workspace structure."""
    print_header("Workspace Structure Check")
    
    workspace_root = Path.cwd()
    
    # Required directories
    required_dirs = [
        'src',
        'src/tractobots_bringup',
        'src/tractobots_navigation',
        'src/tractobots_gps',
        'src/Gcode_parser',
        '.vscode'
    ]
    
    for dir_path in required_dirs:
        full_path = workspace_root / dir_path
        exists = full_path.exists()
        print_check(f"Directory: {dir_path}", exists, f"Path: {full_path}")

def check_vscode_configuration():
    """Check VS Code configuration files."""
    print_header("VS Code Configuration Check")
    
    workspace_root = Path.cwd()
    vscode_dir = workspace_root / '.vscode'
    
    # Required VS Code files
    vscode_files = {
        'settings.json': 'VS Code settings',
        'tasks.json': 'VS Code tasks',
        'launch.json': 'Debug configurations'
    }
    
    for filename, description in vscode_files.items():
        file_path = vscode_dir / filename
        exists = file_path.exists()
        print_check(description, exists, f"File: {file_path}")
        
        if exists and filename == 'settings.json':
            try:
                with open(file_path, 'r') as f:
                    settings = json.load(f)
                    
                # Check key settings
                python_path = settings.get('python.defaultInterpreterPath')
                print_check("Python interpreter path", 
                           python_path == '/usr/bin/python3',
                           f"Set to: {python_path}")
                
                extra_paths = settings.get('python.analysis.extraPaths', [])
                ros_path_found = any('/opt/ros/humble' in path for path in extra_paths)
                print_check("ROS2 Python paths configured", 
                           ros_path_found,
                           f"Found {len(extra_paths)} extra paths")
                
                shellcheck_disabled = settings.get('shellcheck.enable', True) == False
                print_check("ShellCheck properly configured", 
                           shellcheck_disabled,
                           "ShellCheck disabled for Windows compatibility")
                           
            except Exception as e:
                print_check("Settings.json parsing", False, f"Error: {e}")

def check_build_system():
    """Check build system components."""
    print_header("Build System Check")
    
    workspace_root = Path.cwd()
    
    # Check for package.xml files
    package_files = list(workspace_root.glob('src/*/package.xml'))
    print_check("ROS2 packages found", 
               len(package_files) > 0, 
               f"Found {len(package_files)} packages")
    
    # Check for CMakeLists.txt files
    cmake_files = list(workspace_root.glob('src/*/CMakeLists.txt'))
    print_check("CMake packages found", 
               len(cmake_files) > 0, 
               f"Found {len(cmake_files)} CMake packages")
    
    # Check for setup.py files
    setup_files = list(workspace_root.glob('src/*/setup.py'))
    print_check("Python packages found", 
               len(setup_files) > 0, 
               f"Found {len(setup_files)} Python packages")

def check_scripts():
    """Check helper scripts."""
    print_header("Helper Scripts Check")
    
    workspace_root = Path.cwd()
    
    scripts = [
        'quick_setup.sh',
        'test_imports.py',
        'system_test.py'
    ]
    
    for script in scripts:
        script_path = workspace_root / script
        exists = script_path.exists()
        executable = exists and os.access(script_path, os.X_OK)
        print_check(f"Script: {script}", exists)
        if exists and script.endswith('.sh'):
            print_check(f"Script executable: {script}", executable)

def check_documentation():
    """Check documentation files."""
    print_header("Documentation Check")
    
    workspace_root = Path.cwd()
    
    docs = [
        'README.md',
        'GETTING_STARTED.md',
        'VSCODE_BUILD_GUIDE.md',
        'PYTHON_IMPORTS_FIX.md',
        'SHELLCHECK_FIX.md'
    ]
    
    for doc in docs:
        doc_path = workspace_root / doc
        exists = doc_path.exists()
        print_check(f"Documentation: {doc}", exists)

def run_quick_import_test():
    """Run a quick import test."""
    print_header("Quick Import Test")
    
    # Test critical imports
    test_imports = [
        ('os', 'Standard library'),
        ('sys', 'Standard library'),
        ('pathlib', 'Standard library'),
        ('rclpy', 'ROS2 core'),
        ('geometry_msgs.msg', 'ROS2 geometry messages'),
        ('std_msgs.msg', 'ROS2 standard messages'),
        ('sensor_msgs.msg', 'ROS2 sensor messages'),
        ('nav_msgs.msg', 'ROS2 navigation messages')
    ]
    
    for module, description in test_imports:
        try:
            __import__(module)
            print_check(f"Import {module}", True, description)
        except ImportError as e:
            print_check(f"Import {module}", False, f"Error: {e}")

def check_terminal_setup():
    """Check terminal and shell setup."""
    print_header("Terminal Setup Check")
    
    # Check if we're in WSL
    wsl_check = os.path.exists('/proc/version')
    if wsl_check:
        try:
            with open('/proc/version', 'r') as f:
                version_info = f.read()
                is_wsl = 'microsoft' in version_info.lower() or 'wsl' in version_info.lower()
                print_check("Running in WSL", is_wsl, "WSL environment detected")
        except:
            print_check("WSL detection", False, "Could not read /proc/version")
    else:
        print_check("WSL environment", False, "Not running in WSL/Linux")
    
    # Check shell
    shell = os.environ.get('SHELL', 'unknown')
    print_check("Shell environment", 
               'bash' in shell, 
               f"Current shell: {shell}")

def main():
    """Main verification function."""
    print_header("🚜 TRACTOBOTS VS CODE SETUP VERIFICATION")
    print("This script verifies your VS Code development environment for Tractobots.")
    print("Run this from the workspace root directory.")
    
    # Run all checks
    check_workspace_structure()
    check_vscode_configuration()
    check_terminal_setup()
    check_ros2_installation()
    check_build_system()
    check_scripts()
    check_documentation()
    run_quick_import_test()
    
    print_header("🎯 VERIFICATION COMPLETE")
    print("\n📋 NEXT STEPS:")
    print("1. Review any failed checks above")
    print("2. If ROS2 imports fail, run 'source /opt/ros/humble/setup.bash' first")
    print("3. Try building with: Ctrl+Shift+P → 'Tasks: Run Task' → '🚜 Build Tractobots'")
    print("4. For Python import warnings in VS Code, they are cosmetic - see PYTHON_IMPORTS_FIX.md")
    print("5. Use F5 to debug Python nodes with the configured debug profiles")
    
    print("\n🔧 TROUBLESHOOTING:")
    print("- For build issues: Check VSCODE_BUILD_GUIDE.md")
    print("- For import issues: Check PYTHON_IMPORTS_FIX.md") 
    print("- For shell issues: Check SHELLCHECK_FIX.md")
    print("- For setup help: Check GETTING_STARTED.md")

if __name__ == "__main__":
    main()
