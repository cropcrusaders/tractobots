#!/usr/bin/env python3
import sys
print("Python is working!")
print(f"Python version: {sys.version}")

try:
    import tkinter as tk
    print("✅ tkinter imported successfully")
    
    # Try to create a simple window
    root = tk.Tk()
    root.title("Test Window")
    root.geometry("400x300")
    
    label = tk.Label(root, text="🚜 Tractobots Test", font=('Arial', 16))
    label.pack(pady=50)
    
    button = tk.Button(root, text="Close", command=root.quit)
    button.pack(pady=20)
    
    print("✅ GUI window created successfully")
    print("🚀 Opening test window...")
    
    root.mainloop()
    
except Exception as e:
    print(f"❌ Error: {e}")
    sys.exit(1)
    
print("✅ Test completed successfully!")
