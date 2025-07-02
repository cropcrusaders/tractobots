#!/usr/bin/env python3
"""
John Deere GPS Import GUI
User-friendly interface for importing John Deere GPS guidance lines and field data
"""

import sys
import os
from pathlib import Path
import threading
import json

# Add project modules to path
sys.path.append('src/tractobots_mission_ui/tractobots_mission_ui')

try:
    import tkinter as tk
    from tkinter import ttk, filedialog, messagebox, scrolledtext
    TKINTER_AVAILABLE = True
except ImportError:
    TKINTER_AVAILABLE = False

from john_deere_gps_importer import JohnDeereGPSImporter

class JohnDeereImportGUI:
    """GUI for importing John Deere GPS data"""
    
    def __init__(self, root):
        self.root = root
        self.root.title("John Deere GPS Import Tool")
        self.root.geometry("800x600")
        
        # Initialize importer
        self.importer = JohnDeereGPSImporter()
        self.current_gps_data = None
        
        self.setup_gui()
        
    def setup_gui(self):
        """Set up the GUI layout"""
        
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        
        # Title
        title_label = ttk.Label(main_frame, text="🚜 John Deere GPS Import Tool", 
                               font=('TkDefaultFont', 16, 'bold'))
        title_label.grid(row=0, column=0, columnspan=3, pady=(0, 20))
        
        # File selection section
        file_frame = ttk.LabelFrame(main_frame, text="File Selection", padding="10")
        file_frame.grid(row=1, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10))
        file_frame.columnconfigure(1, weight=1)
        
        ttk.Label(file_frame, text="John Deere File:").grid(row=0, column=0, sticky=tk.W)
        
        self.file_path_var = tk.StringVar()
        self.file_entry = ttk.Entry(file_frame, textvariable=self.file_path_var, width=50)
        self.file_entry.grid(row=0, column=1, sticky=(tk.W, tk.E), padx=(10, 10))
        
        self.browse_button = ttk.Button(file_frame, text="Browse...", command=self.browse_file)
        self.browse_button.grid(row=0, column=2, sticky=tk.W)
        
        # Supported formats info
        formats_label = ttk.Label(file_frame, 
                                 text="Supported: .xml (Operations Center), .csv (Guidance Lines), .kml, .gpx, .json",
                                 font=('TkDefaultFont', 8))
        formats_label.grid(row=1, column=0, columnspan=3, sticky=tk.W, pady=(5, 0))
        
        # Import section
        import_frame = ttk.LabelFrame(main_frame, text="Import Options", padding="10")
        import_frame.grid(row=2, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10))
        
        self.import_button = ttk.Button(import_frame, text="Import GPS Data", 
                                       command=self.import_gps_data, state="disabled")
        self.import_button.grid(row=0, column=0, sticky=tk.W)
        
        # Progress bar
        self.progress_var = tk.StringVar(value="Ready to import...")
        self.progress_label = ttk.Label(import_frame, textvariable=self.progress_var)
        self.progress_label.grid(row=0, column=1, sticky=tk.W, padx=(20, 0))
        
        self.progress_bar = ttk.Progressbar(import_frame, mode='indeterminate')
        self.progress_bar.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(10, 0))
        import_frame.columnconfigure(1, weight=1)
        
        # Results section
        results_frame = ttk.LabelFrame(main_frame, text="Import Results", padding="10")
        results_frame.grid(row=3, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 10))
        results_frame.columnconfigure(0, weight=1)
        results_frame.rowconfigure(1, weight=1)
        main_frame.rowconfigure(3, weight=1)
        
        # Results summary
        self.results_summary = ttk.Label(results_frame, text="No data imported yet")
        self.results_summary.grid(row=0, column=0, sticky=tk.W, pady=(0, 10))
        
        # Results detail (scrollable text)
        self.results_text = scrolledtext.ScrolledText(results_frame, height=15, width=70)
        self.results_text.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Export section
        export_frame = ttk.LabelFrame(main_frame, text="Export Options", padding="10")
        export_frame.grid(row=4, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10))
        
        self.export_tractobots_btn = ttk.Button(export_frame, text="Export to Tractobots Format",
                                               command=self.export_tractobots, state="disabled")
        self.export_tractobots_btn.grid(row=0, column=0, sticky=tk.W, padx=(0, 10))
        
        self.export_ros2_btn = ttk.Button(export_frame, text="Export to ROS2 Format",
                                         command=self.export_ros2, state="disabled")
        self.export_ros2_btn.grid(row=0, column=1, sticky=tk.W, padx=(0, 10))
        
        self.view_shapefile_btn = ttk.Button(export_frame, text="Open in Shapefile GUI",
                                            command=self.open_in_shapefile_gui, state="disabled")
        self.view_shapefile_btn.grid(row=0, column=2, sticky=tk.W)
        
        # Bind file entry change
        self.file_path_var.trace('w', self.on_file_path_changed)
        
    def browse_file(self):
        """Browse for John Deere GPS file"""
        filetypes = [
            ("All Supported", "*.xml;*.csv;*.kml;*.gpx;*.json;*.agx"),
            ("XML files", "*.xml"),
            ("CSV files", "*.csv"),
            ("KML files", "*.kml"),
            ("GPX files", "*.gpx"),
            ("JSON files", "*.json"),
            ("AGX files", "*.agx"),
            ("All files", "*.*")
        ]
        
        filename = filedialog.askopenfilename(
            title="Select John Deere GPS File",
            filetypes=filetypes,
            initialdir=str(Path.home())
        )
        
        if filename:
            self.file_path_var.set(filename)
    
    def on_file_path_changed(self, *args):
        """Enable/disable import button based on file selection"""
        file_path = self.file_path_var.get()
        if file_path and Path(file_path).exists():
            self.import_button.config(state="normal")
        else:
            self.import_button.config(state="disabled")
    
    def import_gps_data(self):
        """Import GPS data in background thread"""
        file_path = self.file_path_var.get()
        
        if not file_path or not Path(file_path).exists():
            messagebox.showerror("Error", "Please select a valid file")
            return
        
        # Start import in background
        self.progress_var.set("Importing GPS data...")
        self.progress_bar.start()
        self.import_button.config(state="disabled")
        
        # Clear previous results
        self.results_text.delete(1.0, tk.END)
        
        # Start worker thread
        thread = threading.Thread(target=self._import_worker, args=(file_path,))
        thread.daemon = True
        thread.start()
    
    def _import_worker(self, file_path):
        """Worker thread for importing GPS data"""
        try:
            # Import the data
            self.current_gps_data = self.importer.import_file(file_path)
            
            # Update GUI in main thread
            self.root.after(0, self._import_completed)
            
        except Exception as e:
            # Update GUI with error in main thread
            self.root.after(0, self._import_error, str(e))
    
    def _import_completed(self):
        """Handle successful import completion"""
        self.progress_bar.stop()
        self.progress_var.set("Import completed successfully!")
        self.import_button.config(state="normal")
        
        # Enable export buttons
        self.export_tractobots_btn.config(state="normal")
        self.export_ros2_btn.config(state="normal")
        self.view_shapefile_btn.config(state="normal")
        
        # Display results
        if self.current_gps_data:
            guidance_lines = len(self.current_gps_data.get('guidance_lines', []))
            boundaries = len(self.current_gps_data.get('field_boundaries', []))
            waypoints = len(self.current_gps_data.get('waypoints', []))
            
            summary = f"Imported: {guidance_lines} guidance lines, {boundaries} boundaries, {waypoints} waypoints"
            self.results_summary.config(text=summary)
            
            # Display detailed results
            self.results_text.insert(tk.END, "Import Summary:\n")
            self.results_text.insert(tk.END, "=" * 50 + "\n\n")
            
            self.results_text.insert(tk.END, f"Source: {self.current_gps_data.get('source', 'Unknown')}\n")
            self.results_text.insert(tk.END, f"File: {self.current_gps_data.get('filename', 'Unknown')}\n")
            self.results_text.insert(tk.END, f"Imported at: {self.current_gps_data.get('imported_at', 'Unknown')}\n\n")
            
            # Guidance lines details
            if guidance_lines > 0:
                self.results_text.insert(tk.END, f"Guidance Lines ({guidance_lines}):\n")
                for i, line in enumerate(self.current_gps_data.get('guidance_lines', [])[:10]):  # Show first 10
                    line_id = line.get('line_id', f'line_{i}')
                    points = len(line.get('points', []))
                    line_type = line.get('type', 'unknown')
                    self.results_text.insert(tk.END, f"  - {line_id}: {points} points ({line_type})\n")
                
                if guidance_lines > 10:
                    self.results_text.insert(tk.END, f"  ... and {guidance_lines - 10} more lines\n")
                self.results_text.insert(tk.END, "\n")
            
            # Field boundaries details
            if boundaries > 0:
                self.results_text.insert(tk.END, f"Field Boundaries ({boundaries}):\n")
                for i, boundary in enumerate(self.current_gps_data.get('field_boundaries', [])[:5]):  # Show first 5
                    boundary_id = boundary.get('boundary_id', f'boundary_{i}')
                    points = len(boundary.get('points', []))
                    self.results_text.insert(tk.END, f"  - {boundary_id}: {points} points\n")
                
                if boundaries > 5:
                    self.results_text.insert(tk.END, f"  ... and {boundaries - 5} more boundaries\n")
                self.results_text.insert(tk.END, "\n")
            
            # Waypoints details
            if waypoints > 0:
                self.results_text.insert(tk.END, f"Waypoints ({waypoints}):\n")
                for i, waypoint in enumerate(self.current_gps_data.get('waypoints', [])[:10]):  # Show first 10
                    point_id = waypoint.get('point_id', f'point_{i}')
                    lat = waypoint.get('lat', 0)
                    lon = waypoint.get('lon', 0)
                    self.results_text.insert(tk.END, f"  - {point_id}: {lat:.6f}, {lon:.6f}\n")
                
                if waypoints > 10:
                    self.results_text.insert(tk.END, f"  ... and {waypoints - 10} more waypoints\n")
            
            # Metadata
            metadata = self.current_gps_data.get('metadata', {})
            if metadata:
                self.results_text.insert(tk.END, "\nMetadata:\n")
                for key, value in metadata.items():
                    self.results_text.insert(tk.END, f"  {key}: {value}\n")
        
        messagebox.showinfo("Import Complete", "GPS data imported successfully!")
    
    def _import_error(self, error_message):
        """Handle import error"""
        self.progress_bar.stop()
        self.progress_var.set("Import failed")
        self.import_button.config(state="normal")
        
        self.results_text.insert(tk.END, f"Import Error:\n{error_message}\n")
        messagebox.showerror("Import Error", f"Failed to import GPS data:\n\n{error_message}")
    
    def export_tractobots(self):
        """Export to Tractobots format"""
        if not self.current_gps_data:
            messagebox.showerror("Error", "No GPS data to export")
            return
        
        try:
            output_path = self.importer.export_to_tractobots_format(self.current_gps_data)
            messagebox.showinfo("Export Complete", f"Exported to Tractobots format:\n{output_path}")
        except Exception as e:
            messagebox.showerror("Export Error", f"Failed to export:\n{str(e)}")
    
    def export_ros2(self):
        """Export to ROS2 format"""
        if not self.current_gps_data:
            messagebox.showerror("Error", "No GPS data to export")
            return
        
        try:
            output_path = self.importer.create_guidance_lines_for_ros2(self.current_gps_data)
            messagebox.showinfo("Export Complete", f"Exported to ROS2 format:\n{output_path}")
        except Exception as e:
            messagebox.showerror("Export Error", f"Failed to export:\n{str(e)}")
    
    def open_in_shapefile_gui(self):
        """Open the exported data in the shapefile GUI"""
        if not self.current_gps_data:
            messagebox.showerror("Error", "No GPS data to open")
            return
        
        try:
            # Export to tractobots format first
            tractobots_file = self.importer.export_to_tractobots_format(self.current_gps_data)
            
            # Try to launch the shapefile GUI
            try:
                import subprocess
                subprocess.Popen([
                    'python3', 
                    'src/tractobots_mission_ui/tractobots_mission_ui/shapefile_gui.py'
                ])
                messagebox.showinfo("GUI Launched", 
                                   f"Shapefile GUI launched.\n\nYou can now load the exported file:\n{tractobots_file}")
            except Exception as e:
                messagebox.showwarning("GUI Launch", 
                                      f"Could not launch shapefile GUI automatically.\n\nExported file location:\n{tractobots_file}\n\nYou can manually open the shapefile GUI and load this file.")
        
        except Exception as e:
            messagebox.showerror("Error", f"Failed to prepare data for shapefile GUI:\n{str(e)}")

def main():
    """Main entry point"""
    if not TKINTER_AVAILABLE:
        print("❌ Tkinter not available. Please install python3-tk")
        sys.exit(1)
    
    # Create and run GUI
    root = tk.Tk()
    app = JohnDeereImportGUI(root)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        print("\n👋 John Deere GPS Import Tool closed")

if __name__ == '__main__':
    main()
