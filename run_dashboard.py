#!/usr/bin/env python3
"""
Quick Dashboard Test and Launcher
"""

import os
import sys
import subprocess

def test_python():
    """Test Python environment"""
    print("🐍 Testing Python environment...")
    print(f"   Python version: {sys.version}")
    print("   ✅ Python is working!")
    return True

def test_imports():
    """Test required imports"""
    print("\n📦 Testing required packages...")
    
    # Test Flask
    try:
        import flask
        print("   ✅ Flask available")
        flask_ok = True
    except ImportError:
        print("   ❌ Flask not available - installing...")
        try:
            subprocess.run([sys.executable, '-m', 'pip', 'install', 'flask'], 
                         capture_output=True, check=True)
            print("   ✅ Flask installed successfully")
            flask_ok = True
        except:
            print("   ❌ Flask installation failed")
            flask_ok = False
    
    # Test other packages
    packages = ['json', 'time', 'threading', 'datetime', 'math', 'subprocess']
    for pkg in packages:
        try:
            __import__(pkg)
            print(f"   ✅ {pkg} available")
        except ImportError:
            print(f"   ❌ {pkg} not available")
    
    return flask_ok

def launch_web_dashboard():
    """Launch the web dashboard"""
    print("\n🚀 Starting Tractobots Web Dashboard...")
    
    # Check if dashboard file exists
    dashboard_file = "tractobots_web_dashboard.py"
    if not os.path.exists(dashboard_file):
        print(f"   ❌ {dashboard_file} not found")
        return False
    
    print(f"   ✅ Found {dashboard_file}")
    print("   🌐 Starting web server...")
    
    try:
        # Try to run the dashboard directly
        print("   📡 Starting dashboard process...")
        result = subprocess.run([sys.executable, dashboard_file], 
                              cwd=os.getcwd())
        
        if result.returncode == 0:
            print("   ✅ Dashboard completed successfully")
            return True
        else:
            print(f"   ❌ Dashboard exited with code {result.returncode}")
            return False
            
    except KeyboardInterrupt:
        print("\n   🛑 Dashboard stopped by user")
        return True
    except Exception as e:
        print(f"   ❌ Error starting dashboard: {e}")
        return False

def main():
    """Main function"""
    print("🚜 TRACTOBOTS DASHBOARD LAUNCHER")
    print("=" * 40)
    
    # Test system
    if not test_python():
        return False
    
    if not test_imports():
        print("\n❌ Required packages not available")
        return False
    
    # Launch dashboard
    print("\n🎯 System ready! Launching dashboard...")
    return launch_web_dashboard()

if __name__ == "__main__":
    try:
        success = main()
        if success:
            print("\n✅ Dashboard session completed successfully!")
        else:
            print("\n❌ Dashboard launch failed!")
    except Exception as e:
        print(f"\n💥 Unexpected error: {e}")
    
    print("\n🎉 Thank you for using Tractobots Dashboard!")
