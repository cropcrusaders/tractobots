#!/usr/bin/env python3
"""
Tractobots Dashboard Cleanup Utility

This script helps clean up redundant files after organizing the dashboard.
It will move or delete redundant files, ensuring a clean directory structure.

Run this script after the dashboard has been successfully reorganized.
"""

import os
import sys
import shutil
import glob

# Files to be deleted (relative paths from project root)
FILES_TO_DELETE = [
    # Older dashboard versions
    "tractobots_win_dashboard.py",
    "simple_dashboard.py",
    "simple_dashboard_test.py",
    "final_dashboard_test.py",
    "debug_dashboard.py",
    "debug_dashboard_launch.py",
    "dashboard_integration_test.py",
    
    # Older test files
    "test_dashboard_*.py",  # All dashboard test files
    "test_terminal.py",
    "test_fixed_dashboard.py",
    "test_ros_connection.py",
    "test_direct_wsl_connection.py", 
    "test_rosbridge_start.py",
    "test_improved_connection.py",
    "test_final_integration.py",
    "test_signal_direct.py",
    
    # Older utilities
    "fix_wsl_port_forwarding.py",
    "launch_dashboard.py",
    "setup_wsl_port_forwarding.bat",
    "Setup-WSL-PortForwarding.ps1",
    
    # Redundant validation and verification files
    "validate_fixes.py",
    "verify_fixes.py",
    "verify_installation.py", 
    "final_validation.py",
    "final_verification.py",
    "final_integration_test.py",
    "complete_integration_test.py",
    "complete_system_test.py",
    
    # Other redundant files
    "*dashboard*test.py",  # All dashboard test files
    "rosbridge_complete_solution.py",
    "simple_terminal_test.py",
    "simple_connection_test.py",
    "final_connection_test.py",
    "diagnose_problems.py",
    "fix_338_problems.py",
    "quick_status_check.py",
    "terminal_status_report.py",
]

# Get the project root directory
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(script_dir, "..", ".."))

def main():
    """Main cleanup function"""
    print("\n=== TRACTOBOTS DASHBOARD CLEANUP ===")
    print(f"Project root: {project_root}")
    print("\nThis script will delete redundant files after the dashboard reorganization.")
    print("Make sure you have backed up any important files before proceeding.\n")
    
    # Check if simulation mode is requested
    simulate = False
    if len(sys.argv) > 1 and sys.argv[1].lower() in ('--simulate', '-s', '/s'):
        simulate = True
        print("SIMULATION MODE: No files will actually be deleted.")
        print("This will only show which files would be deleted.\n")
    
    # Skip confirmation in simulation mode
    if not simulate:
        # Try to handle PowerShell input issues
        try:
            confirm = input("Proceed with cleanup? (y/n): ").lower()
            if confirm != 'y':
                print("Cleanup canceled.")
                return
        except Exception as e:
            print(f"Input error: {e}")
            print("Using default confirmation (n). Run with --simulate to see what would be deleted.")
            print("Or run with 'echo y | python cleanup_redundant_files.py' to confirm automatically.")
            return
    
    # Delete redundant files
    deleted_count = 0
    skipped_count = 0
    
    print("\nDeleting redundant files..." if not simulate else "\nFiles that would be deleted:")
    for file_pattern in FILES_TO_DELETE:
        # Handle glob patterns
        if '*' in file_pattern:
            matches = glob.glob(os.path.join(project_root, file_pattern))
            for file_path in matches:
                if os.path.exists(file_path):
                    try:
                        if not simulate:
                            os.remove(file_path)
                        print(f"  {'[WOULD DELETE]' if simulate else '✓ Deleted:'} {os.path.relpath(file_path, project_root)}")
                        deleted_count += 1
                    except Exception as e:
                        print(f"  ✗ Error with {file_path}: {e}")
                        skipped_count += 1
        else:
            # Single file
            file_path = os.path.join(project_root, file_pattern)
            if os.path.exists(file_path):
                try:
                    if not simulate:
                        os.remove(file_path)
                    print(f"  {'[WOULD DELETE]' if simulate else '✓ Deleted:'} {file_pattern}")
                    deleted_count += 1
                except Exception as e:
                    print(f"  ✗ Error with {file_path}: {e}")
                    skipped_count += 1
    
    # Print summary
    print("\n=== CLEANUP SUMMARY ===")
    if simulate:
        print(f"Files that would be deleted: {deleted_count}")
        print(f"Files that would be skipped: {skipped_count}")
        print("\nSimulation complete. No files were actually deleted.")
        print("To actually delete these files, run the script without the --simulate flag.")
    else:
        print(f"Files deleted: {deleted_count}")
        print(f"Files skipped: {skipped_count}")
        print("\nCleanup complete.")
    
    print("The dashboard code is now organized in the dashboard/ directory.")

if __name__ == "__main__":
    main()
