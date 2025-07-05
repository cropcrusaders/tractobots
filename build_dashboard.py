import subprocess
import sys
import os

# --- Dependency Checker ---
def install_and_import(package, import_name=None):
    """Install a package if it is not already installed."""
    if import_name is None:
        import_name = package
    try:
        __import__(import_name)
        print(f"✅ {package} is already installed.")
    except ImportError:
        print(f"📦 {package} not found. Installing...")
        try:
            subprocess.check_call([sys.executable, "-m", "pip", "install", package])
            print(f"✅ {package} installed successfully.")
            __import__(import_name) # Try importing again after install
        except Exception as e:
            print(f"❌ Failed to install {package}. Please install it manually using: python -m pip install {package}")
            print(f"Error: {e}")
            sys.exit(1)

# Install PyInstaller
install_and_import("pyinstaller")

# Run PyInstaller
print("\n\nBuilding the executable...")
subprocess.check_call([sys.executable, '-m', 'PyInstaller', '--onefile', '--windowed', '--name', 'TractobotsDashboard', 'tractobots_win_dashboard.py'])

print("\n\n✅ Build complete!")
print(f"You can find the executable in the '{os.path.join(os.getcwd(), 'dist')}' folder.")
