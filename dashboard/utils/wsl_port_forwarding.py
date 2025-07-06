#!/usr/bin/env python3
"""
WSL Port Forwarding Setup for Rosbridge
This script configures Windows to access WSL ports properly.
"""

import subprocess
import sys
import socket
import time

def get_wsl_ip():
    """Get the IP address of the WSL instance."""
    try:
        result = subprocess.run(
            ["wsl", "-e", "bash", "-c", "hostname -I | awk '{print $1}'"],
            capture_output=True, text=True, timeout=10
        )
        if result.returncode == 0:
            wsl_ip = result.stdout.strip()
            print(f"🔍 WSL IP Address: {wsl_ip}")
            return wsl_ip
        else:
            print(f"❌ Failed to get WSL IP: {result.stderr}")
            return None
    except Exception as e:
        print(f"❌ Error getting WSL IP: {e}")
        return None

def setup_port_forwarding(wsl_ip, port=9090):
    """Setup Windows port forwarding to WSL."""
    try:
        # Remove existing port proxy (if any)
        print(f"🔧 Removing existing port forwarding for port {port}...")
        subprocess.run([
            "netsh", "interface", "portproxy", "delete", "v4tov4", 
            f"listenport={port}"
        ], capture_output=True)
        
        # Add new port forwarding
        print(f"🔧 Setting up port forwarding: localhost:{port} -> {wsl_ip}:{port}")
        result = subprocess.run([
            "netsh", "interface", "portproxy", "add", "v4tov4",
            f"listenport={port}", "listenaddress=127.0.0.1",
            f"connectport={port}", f"connectaddress={wsl_ip}"
        ], capture_output=True, text=True)
        
        if result.returncode == 0:
            print("✅ Port forwarding configured successfully!")
            return True
        else:
            print(f"❌ Failed to set up port forwarding: {result.stderr}")
            return False
    except Exception as e:
        print(f"❌ Error setting up port forwarding: {e}")
        return False

def test_connection(port=9090):
    """Test connection to the forwarded port."""
    print(f"🔍 Testing connection to localhost:{port}...")
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2)
        result = sock.connect_ex(('127.0.0.1', port))
        sock.close()
        
        if result == 0:
            print(f"✅ Port {port} is open and accessible!")
            return True
        else:
            print(f"⚠️ Port {port} is not accessible (this is normal if rosbridge is not running)")
            return False
    except Exception as e:
        print(f"❌ Error testing connection: {e}")
        return False

def main():
    """Main entry point."""
    print("\n=== WSL PORT FORWARDING SETUP ===")
    print("This script configures port forwarding from Windows to WSL")
    print("for ROS Bridge WebSocket connections.")
    
    # Get WSL IP
    print("\n📡 Detecting WSL IP address...")
    wsl_ip = get_wsl_ip()
    
    if not wsl_ip:
        print("\n❌ Failed to get WSL IP address.")
        print("Please ensure WSL is installed and running.")
        return False
    
    # Set up port forwarding
    print("\n🔄 Setting up port forwarding...")
    success = setup_port_forwarding(wsl_ip)
    
    if not success:
        print("\n❌ Failed to set up port forwarding.")
        print("Please run this script as administrator.")
        return False
    
    # Test connection
    test_connection()
    
    print("\n✅ PORT FORWARDING SETUP COMPLETE")
    print("You can now connect to ROS Bridge WebSocket on:")
    print(f"  ws://localhost:9090")
    print("\nNOTE: If WSL IP changes after restart, run this script again.")
    
    return True

if __name__ == "__main__":
    try:
        if main():
            print("\n✅ Setup completed successfully!")
        else:
            print("\n⚠️ Setup encountered issues. See messages above.")
    except KeyboardInterrupt:
        print("\n🛑 Setup canceled by user.")
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
    
    # On Windows, keep the console open
    if sys.platform.startswith('win'):
        input("\nPress Enter to exit...")
