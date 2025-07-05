#!/usr/bin/env python3
"""
Quick Dashboard Status Check
"""

print("🚜 TRACTOBOTS DASHBOARD STATUS CHECK")
print("====================================")
print()

# Check Python
import sys
print(f"✅ Python {sys.version.split()[0]} is working")

# Check imports
modules = [
    ('tkinter', 'GUI support'),
    ('matplotlib', 'Plotting'),
    ('numpy', 'Numerical computing'),
    ('flask', 'Web dashboard'),
    ('subprocess', 'System control')
]

for module, description in modules:
    try:
        __import__(module)
        print(f"✅ {module:12} - {description}")
    except ImportError:
        print(f"❌ {module:12} - {description} (install with: pip3 install {module})")

print()

# Check ROS2
try:
    import subprocess
    result = subprocess.run(['ros2', '--version'], capture_output=True, text=True, timeout=5)
    if result.returncode == 0:
        print("✅ ROS2 is available")
    else:
        print("❌ ROS2 not found")
except:
    print("❌ ROS2 not available")

# Check display
import os
if os.environ.get('DISPLAY'):
    print(f"✅ Display available: {os.environ.get('DISPLAY')}")
else:
    print("❌ No display environment (GUI may not work)")

print()
print("📊 Available Dashboards:")
print("1. Web Dashboard   - Works everywhere, access via browser")
print("2. GUI Dashboard   - Requires display environment")
print("3. Terminal Dashboard - Text-based, works in any terminal")
print()
print("🚀 To launch: ./launch_tractobots_dashboard.sh")
print("📱 Or directly: python3 tractobots_web_dashboard.py")
