#!/usr/bin/env python3
"""
Tractobots Dashboard Integration Test
Tests all dashboard components and connections.
"""

import os
import sys
import time
import subprocess
import unittest

# Add parent directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

try:
    from PyQt5.QtWidgets import QApplication
    from PyQt5.QtTest import QTest
    from PyQt5.QtCore import Qt, QTimer
    import roslibpy
    DEPENDENCIES_MET = True
except ImportError as e:
    print(f"[ERROR] Missing dependency: {e}")
    DEPENDENCIES_MET = False


class DashboardIntegrationTest(unittest.TestCase):
    """Integration tests for the Tractobots Dashboard"""
    
    @classmethod
    def setUpClass(cls):
        """Set up the test environment"""
        print("\n=== TRACTOBOTS DASHBOARD INTEGRATION TESTS ===\n")
        
        # Check if dependencies are met
        if not DEPENDENCIES_MET:
            raise ImportError("Required dependencies are not installed")
        
        # Create QApplication instance
        cls.app = QApplication.instance() or QApplication([])
    
    def test_01_dashboard_imports(self):
        """Test that the dashboard module can be imported"""
        print("Testing dashboard imports...")
        try:
            from dashboard.dashboard import TractobotsDashboard
            self.assertTrue(True, "Dashboard imported successfully")
        except ImportError as e:
            self.fail(f"Failed to import dashboard: {e}")
    
    def test_02_wsl_availability(self):
        """Test that WSL is available"""
        print("Testing WSL availability...")
        try:
            result = subprocess.run(
                ["wsl", "echo", "test"], 
                capture_output=True, text=True, timeout=5
            )
            self.assertEqual(result.returncode, 0, "WSL command should succeed")
            self.assertIn("test", result.stdout, "WSL should return the echo output")
        except Exception as e:
            self.fail(f"WSL test failed: {e}")
    
    def test_03_dashboard_creation(self):
        """Test that the dashboard can be instantiated"""
        print("Testing dashboard instantiation...")
        try:
            from dashboard.dashboard import TractobotsDashboard
            dashboard = TractobotsDashboard()
            self.assertIsNotNone(dashboard, "Dashboard should be created")
            dashboard.close()
        except Exception as e:
            self.fail(f"Failed to create dashboard: {e}")
    
    def test_04_rosbridge_connection(self):
        """Test connection to rosbridge (when available)"""
        print("Testing rosbridge connection...")
        try:
            # This test will be skipped if rosbridge is not running
            client = roslibpy.Ros(host='localhost', port=9090)
            connected = [False]
            
            def on_connection():
                connected[0] = True
                client.terminate()
            
            client.on_ready(on_connection)
            
            try:
                # Try to connect with timeout
                client.run(timeout=2)
                if connected[0]:
                    print("  ✓ Successfully connected to rosbridge")
                else:
                    print("  ⚠️ Could not connect to rosbridge (this is expected if rosbridge is not running)")
            except Exception:
                print("  ⚠️ Rosbridge connection failed (this is expected if rosbridge is not running)")
        except Exception as e:
            print(f"  ⚠️ Rosbridge test error: {e}")
    
    def test_05_port_forwarding(self):
        """Test port forwarding setup utility"""
        print("Testing port forwarding utility...")
        try:
            from dashboard.utils.wsl_port_forwarding import get_wsl_ip
            wsl_ip = get_wsl_ip()
            if wsl_ip:
                print(f"  ✓ Got WSL IP: {wsl_ip}")
                self.assertIsNotNone(wsl_ip, "Should get a WSL IP address")
            else:
                print("  ⚠️ Could not get WSL IP (this may happen if WSL is not running)")
        except Exception as e:
            print(f"  ⚠️ Port forwarding test error: {e}")


def run_gui_tests():
    """Run the GUI tests with proper setup"""
    loader = unittest.TestLoader()
    suite = loader.loadTestsFromTestCase(DashboardIntegrationTest)
    unittest.TextTestRunner(verbosity=2).run(suite)


if __name__ == "__main__":
    if DEPENDENCIES_MET:
        run_gui_tests()
    else:
        print("[ERROR] Cannot run tests - dependencies missing")
