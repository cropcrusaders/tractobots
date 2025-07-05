#!/usr/bin/env python3
"""
Comprehensive Gazebo Integration Test for Tractobots
Tests all aspects of Gazebo simulator integration with ROS2
"""

import os
import sys
import time
import subprocess
import threading
import signal
from pathlib import Path
import yaml
import json

# Colors for output
class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    WHITE = '\033[97m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    END = '\033[0m'

class GazeboTester:
    def __init__(self):
        self.test_results = {}
        self.processes = []
        self.test_count = 0
        self.passed_count = 0
        
    def print_header(self, title):
        """Print a formatted test header"""
        print(f"\n{Colors.BLUE}{'='*60}{Colors.END}")
        print(f"{Colors.BLUE}{Colors.BOLD}  {title}{Colors.END}")
        print(f"{Colors.BLUE}{'='*60}{Colors.END}")
        
    def print_test(self, test_name, status, details=""):
        """Print test result"""
        self.test_count += 1
        if status == "PASS":
            self.passed_count += 1
            print(f"{Colors.GREEN}✅ {test_name}{Colors.END}")
        elif status == "FAIL":
            print(f"{Colors.RED}❌ {test_name}{Colors.END}")
        elif status == "WARN":
            print(f"{Colors.YELLOW}⚠️  {test_name}{Colors.END}")
        else:
            print(f"{Colors.CYAN}ℹ️  {test_name}{Colors.END}")
            
        if details:
            print(f"   {details}")
            
        self.test_results[test_name] = {
            'status': status,
            'details': details
        }
        
    def run_command(self, cmd, timeout=30, capture_output=True):
        """Run a command with timeout"""
        try:
            if capture_output:
                result = subprocess.run(
                    cmd, 
                    shell=True, 
                    capture_output=True, 
                    text=True, 
                    timeout=timeout
                )
                return result.returncode == 0, result.stdout, result.stderr
            else:
                result = subprocess.run(
                    cmd,
                    shell=True,
                    timeout=timeout
                )
                return result.returncode == 0, "", ""
        except subprocess.TimeoutExpired:
            return False, "", "Command timed out"
        except Exception as e:
            return False, "", str(e)
    
    def test_system_requirements(self):
        """Test system requirements"""
        self.print_header("System Requirements")
        
        # Check Ubuntu version
        success, stdout, stderr = self.run_command("lsb_release -r")
        if success and "24.04" in stdout:
            self.print_test("Ubuntu 24.04 detected", "PASS")
        else:
            self.print_test("Ubuntu version check", "WARN", "Non-standard Ubuntu version")
            
        # Check ROS2 Jazzy
        success, stdout, stderr = self.run_command("which ros2")
        if success and "jazzy" in stdout:
            self.print_test("ROS2 Jazzy installation", "PASS")
        else:
            self.print_test("ROS2 Jazzy installation", "FAIL", "ROS2 Jazzy not found")
            return False
            
        # Check Python version
        success, stdout, stderr = self.run_command("python3 --version")
        if success and "3.12" in stdout:
            self.print_test("Python 3.12 detected", "PASS")
        else:
            self.print_test("Python version", "WARN", "Non-standard Python version")
            
        return True
    
    def test_gazebo_installation(self):
        """Test Gazebo installation"""
        self.print_header("Gazebo Installation")
        
        # Check Gazebo command
        success, stdout, stderr = self.run_command("which gz")
        if success:
            self.print_test("Gazebo command available", "PASS")
        else:
            self.print_test("Gazebo command available", "FAIL", "gz command not found")
            return False
            
        # Check Gazebo version
        success, stdout, stderr = self.run_command("gz sim --version")
        if success:
            self.print_test("Gazebo version check", "PASS", stdout.strip())
        else:
            self.print_test("Gazebo version check", "FAIL", stderr)
            
        # Check ROS-Gazebo bridge
        success, stdout, stderr = self.run_command("ros2 pkg list | grep ros_gz")
        if success and "ros_gz_bridge" in stdout:
            self.print_test("ROS-Gazebo bridge installed", "PASS")
        else:
            self.print_test("ROS-Gazebo bridge installed", "FAIL", "ros_gz packages not found")
            
        return True
    
    def test_gazebo_workspace(self):
        """Test Gazebo workspace setup"""
        self.print_header("Gazebo Workspace")
        
        # Check workspace directory
        gazebo_ws = Path.home() / "tractobots_gazebo_ws"
        if gazebo_ws.exists():
            self.print_test("Gazebo workspace exists", "PASS")
        else:
            self.print_test("Gazebo workspace exists", "FAIL", "Workspace not found")
            return False
            
        # Check package structure
        pkg_dir = gazebo_ws / "src" / "tractobots_gazebo"
        if pkg_dir.exists():
            self.print_test("Package structure", "PASS")
        else:
            self.print_test("Package structure", "FAIL", "Package directory not found")
            return False
            
        # Check required files
        required_files = [
            "package.xml",
            "CMakeLists.txt",
            "launch/tractobots_gazebo.launch.py",
            "worlds/farm_field.sdf"
        ]
        
        for file_path in required_files:
            if (pkg_dir / file_path).exists():
                self.print_test(f"Required file: {file_path}", "PASS")
            else:
                self.print_test(f"Required file: {file_path}", "FAIL", "File not found")
                
        # Check if workspace is built
        install_dir = gazebo_ws / "install"
        if install_dir.exists():
            self.print_test("Workspace built", "PASS")
        else:
            self.print_test("Workspace built", "FAIL", "Install directory not found")
            
        return True
    
    def test_ros2_integration(self):
        """Test ROS2 integration"""
        self.print_header("ROS2 Integration")
        
        # Source ROS2 environment
        ros2_setup = ". /opt/ros/jazzy/setup.bash"
        
        # Check ROS2 packages
        success, stdout, stderr = self.run_command(f"{ros2_setup} && ros2 pkg list")
        if success:
            packages = stdout.split('\n')
            
            # Check for required packages
            required_packages = [
                'ros_gz_bridge',
                'ros_gz_sim',
                'nav2_bringup',
                'robot_state_publisher',
                'rviz2'
            ]
            
            for pkg in required_packages:
                if pkg in packages:
                    self.print_test(f"ROS2 package: {pkg}", "PASS")
                else:
                    self.print_test(f"ROS2 package: {pkg}", "FAIL", "Package not found")
        else:
            self.print_test("ROS2 package check", "FAIL", stderr)
            
        # Check if workspace package is available
        gazebo_ws = Path.home() / "tractobots_gazebo_ws"
        if gazebo_ws.exists():
            ws_setup = f". {gazebo_ws}/install/setup.bash"
            success, stdout, stderr = self.run_command(f"{ros2_setup} && {ws_setup} && ros2 pkg list | grep tractobots_gazebo")
            if success:
                self.print_test("Tractobots Gazebo package", "PASS")
            else:
                self.print_test("Tractobots Gazebo package", "FAIL", "Package not found in workspace")
        
        return True
    
    def test_gazebo_launch(self):
        """Test Gazebo launch functionality"""
        self.print_header("Gazebo Launch Test")
        
        # Test basic Gazebo launch
        gazebo_ws = Path.home() / "tractobots_gazebo_ws"
        if not gazebo_ws.exists():
            self.print_test("Gazebo launch test", "FAIL", "Workspace not found")
            return False
            
        ros2_setup = ". /opt/ros/jazzy/setup.bash"
        ws_setup = f". {gazebo_ws}/install/setup.bash"
        
        # Test world file loading
        world_file = gazebo_ws / "src" / "tractobots_gazebo" / "worlds" / "farm_field.sdf"
        if world_file.exists():
            self.print_test("World file exists", "PASS")
            
            # Test world file syntax
            try:
                with open(world_file, 'r') as f:
                    content = f.read()
                    if '<world name="farm_field">' in content:
                        self.print_test("World file syntax", "PASS")
                    else:
                        self.print_test("World file syntax", "FAIL", "Invalid world structure")
            except Exception as e:
                self.print_test("World file syntax", "FAIL", str(e))
        else:
            self.print_test("World file exists", "FAIL", "farm_field.sdf not found")
            
        # Test launch file
        launch_file = gazebo_ws / "src" / "tractobots_gazebo" / "launch" / "tractobots_gazebo.launch.py"
        if launch_file.exists():
            self.print_test("Launch file exists", "PASS")
            
            # Test launch file syntax
            try:
                with open(launch_file, 'r') as f:
                    content = f.read()
                    if 'generate_launch_description' in content:
                        self.print_test("Launch file syntax", "PASS")
                    else:
                        self.print_test("Launch file syntax", "FAIL", "Invalid launch file structure")
            except Exception as e:
                self.print_test("Launch file syntax", "FAIL", str(e))
        else:
            self.print_test("Launch file exists", "FAIL", "Launch file not found")
            
        return True
    
    def test_shapefile_integration(self):
        """Test shapefile integration with Gazebo"""
        self.print_header("Shapefile Integration")
        
        # Check if shapefile manager exists
        shapefile_manager = Path("src/tractobots_mission_ui/tractobots_mission_ui/shapefile_manager.py")
        if shapefile_manager.exists():
            self.print_test("Shapefile manager exists", "PASS")
            
            # Test shapefile manager import
            try:
                sys.path.insert(0, "src/tractobots_mission_ui/tractobots_mission_ui")
                from shapefile_manager import ShapefileManager  # type: ignore
                manager = ShapefileManager()
                self.print_test("Shapefile manager import", "PASS")
                
                # Test if it has Gazebo export method
                if hasattr(manager, 'export_to_gazebo_world'):
                    self.print_test("Gazebo export method", "PASS")
                else:
                    self.print_test("Gazebo export method", "FAIL", "Method not implemented")
                    
            except Exception as e:
                self.print_test("Shapefile manager import", "FAIL", str(e))
        else:
            self.print_test("Shapefile manager exists", "FAIL", "File not found")
            
        return True
    
    def test_gps_integration(self):
        """Test John Deere GPS integration with Gazebo"""
        self.print_header("GPS Integration")
        
        # Check if GPS importer exists
        gps_importer = Path("john_deere_gps_importer.py")
        if gps_importer.exists():
            self.print_test("GPS importer exists", "PASS")
            
            # Test GPS importer
            try:
                sys.path.insert(0, ".")
                from john_deere_gps_importer import JohnDeereGPSImporter  # type: ignore
                importer = JohnDeereGPSImporter()
                self.print_test("GPS importer import", "PASS")
                
                # Test if it has Gazebo export method
                if hasattr(importer, 'export_to_gazebo_waypoints'):
                    self.print_test("Gazebo waypoint export", "PASS")
                else:
                    self.print_test("Gazebo waypoint export", "FAIL", "Method not implemented")
                    
            except Exception as e:
                self.print_test("GPS importer import", "FAIL", str(e))
        else:
            self.print_test("GPS importer exists", "FAIL", "File not found")
            
        return True
    
    def test_environment_variables(self):
        """Test environment variables"""
        self.print_header("Environment Variables")
        
        # Check Gazebo environment variables
        gazebo_vars = [
            'GZ_SIM_RESOURCE_PATH',
            'GAZEBO_MODEL_PATH',
            'GAZEBO_PLUGIN_PATH'
        ]
        
        for var in gazebo_vars:
            success, stdout, stderr = self.run_command(f"echo ${var}")
            if success and stdout.strip():
                self.print_test(f"Environment variable: {var}", "PASS", stdout.strip())
            else:
                self.print_test(f"Environment variable: {var}", "WARN", "Not set")
                
        return True
    
    def test_performance(self):
        """Test system performance for Gazebo"""
        self.print_header("Performance Check")
        
        # Check CPU cores
        success, stdout, stderr = self.run_command("nproc")
        if success:
            cores = int(stdout.strip())
            if cores >= 4:
                self.print_test("CPU cores", "PASS", f"{cores} cores detected")
            else:
                self.print_test("CPU cores", "WARN", f"Only {cores} cores - may affect performance")
        
        # Check memory
        success, stdout, stderr = self.run_command("free -h | grep Mem")
        if success:
            mem_info = stdout.strip()
            self.print_test("Memory check", "PASS", mem_info)
        
        # Check disk space
        success, stdout, stderr = self.run_command("df -h ~ | tail -1")
        if success:
            disk_info = stdout.strip()
            self.print_test("Disk space", "PASS", disk_info)
            
        return True
    
    def run_all_tests(self):
        """Run all tests"""
        print(f"{Colors.CYAN}{Colors.BOLD}")
        print("🚜 TRACTOBOTS GAZEBO INTEGRATION TEST SUITE 🚜")
        print(f"{Colors.END}")
        
        # Run test suites
        test_suites = [
            self.test_system_requirements,
            self.test_gazebo_installation,
            self.test_gazebo_workspace,
            self.test_ros2_integration,
            self.test_gazebo_launch,
            self.test_shapefile_integration,
            self.test_gps_integration,
            self.test_environment_variables,
            self.test_performance
        ]
        
        for test_suite in test_suites:
            try:
                test_suite()
            except Exception as e:
                self.print_test(f"Test suite error: {test_suite.__name__}", "FAIL", str(e))
        
        # Print summary
        self.print_summary()
        
    def print_summary(self):
        """Print test summary"""
        self.print_header("Test Summary")
        
        success_rate = (self.passed_count / self.test_count) * 100 if self.test_count > 0 else 0
        
        print(f"Total tests: {self.test_count}")
        print(f"Passed: {Colors.GREEN}{self.passed_count}{Colors.END}")
        print(f"Failed: {Colors.RED}{self.test_count - self.passed_count}{Colors.END}")
        print(f"Success rate: {Colors.GREEN if success_rate >= 80 else Colors.RED}{success_rate:.1f}%{Colors.END}")
        
        if success_rate >= 80:
            print(f"\n{Colors.GREEN}🎉 GAZEBO INTEGRATION READY! 🎉{Colors.END}")
            print(f"{Colors.GREEN}Your Tractobots system is ready for professional 3D simulation!{Colors.END}")
        else:
            print(f"\n{Colors.RED}❌ INTEGRATION ISSUES DETECTED{Colors.END}")
            print(f"{Colors.RED}Please review the failed tests above and run setup_gazebo_simulator.sh{Colors.END}")
            
        # Save results to file
        self.save_results()
        
    def save_results(self):
        """Save test results to file"""
        results = {
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'total_tests': self.test_count,
            'passed_tests': self.passed_count,
            'success_rate': (self.passed_count / self.test_count) * 100 if self.test_count > 0 else 0,
            'detailed_results': self.test_results
        }
        
        with open('gazebo_integration_test_results.json', 'w') as f:
            json.dump(results, f, indent=2)
            
        print(f"\n{Colors.BLUE}Test results saved to: gazebo_integration_test_results.json{Colors.END}")

def main():
    """Main test function"""
    tester = GazeboTester()
    
    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        print(f"\n{Colors.YELLOW}Test interrupted by user{Colors.END}")
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        tester.run_all_tests()
    except KeyboardInterrupt:
        print(f"\n{Colors.YELLOW}Test interrupted by user{Colors.END}")
    except Exception as e:
        print(f"\n{Colors.RED}Test suite error: {str(e)}{Colors.END}")
        sys.exit(1)

if __name__ == "__main__":
    main()
