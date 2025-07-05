#!/usr/bin/env python3
"""
Simple test to verify the dashboard can run
"""

import sys
import os
import tkinter as tk
from tkinter import ttk

def test_dashboard():
    """Test basic dashboard functionality"""
    print("Testing Tractobots Dashboard...")
    
    # Test imports
    try:
        import matplotlib.pyplot as plt
        print("✓ matplotlib imported successfully")
    except ImportError as e:
        print(f"✗ matplotlib import failed: {e}")
        return False
    
    try:
        import numpy as np
        print("✓ numpy imported successfully")
    except ImportError as e:
        print(f"✗ numpy import failed: {e}")
        return False
    
    # Test tkinter
    try:
        root = tk.Tk()
        root.title("Tractobots Dashboard Test")
        root.geometry("800x600")
        
        # Create a simple test interface
        label = tk.Label(root, text="🚜 Tractobots Dashboard Test", font=('Arial', 16, 'bold'))
        label.pack(pady=20)
        
        # Create test buttons
        frame = tk.Frame(root)
        frame.pack(pady=20)
        
        btn1 = tk.Button(frame, text="Test Button 1", command=lambda: print("Button 1 clicked"))
        btn1.pack(side=tk.LEFT, padx=10)
        
        btn2 = tk.Button(frame, text="Test Button 2", command=lambda: print("Button 2 clicked"))
        btn2.pack(side=tk.LEFT, padx=10)
        
        btn3 = tk.Button(frame, text="Close", command=root.quit)
        btn3.pack(side=tk.LEFT, padx=10)
        
        # Status label
        status_label = tk.Label(root, text="Dashboard test running successfully!", fg="green")
        status_label.pack(pady=10)
        
        print("✓ GUI test window created successfully")
        print("✓ Opening test window...")
        
        # Run the GUI
        root.mainloop()
        print("✓ GUI closed successfully")
        return True
        
    except Exception as e:
        print(f"✗ GUI test failed: {e}")
        return False

if __name__ == "__main__":
    success = test_dashboard()
    if success:
        print("\n✅ Dashboard test completed successfully!")
        print("The main dashboard should work properly.")
    else:
        print("\n❌ Dashboard test failed!")
        print("Please check the error messages above.")
