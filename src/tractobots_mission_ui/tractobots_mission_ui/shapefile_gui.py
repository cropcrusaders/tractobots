#!/usr/bin/env python3
"""
Shapefile Integration GUI for Tractobots
Provides a user interface for loading and managing field boundaries from shapefiles
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox, scrolledtext
import json
import os
from pathlib import Path
from typing import Dict, Optional
import threading

try:
    from .shapefile_manager import ShapefileFieldManager
except ImportError:
    import sys
    sys.path.append(os.path.dirname(__file__))
    from shapefile_manager import ShapefileFieldManager

class ShapefileGUI:
    """
    GUI for shapefile field boundary management
    """
    
    def __init__(self, parent=None):
        if parent:
            self.root = tk.Toplevel(parent)
        else:
            self.root = tk.Tk()
        
        self.root.title("Tractobots - Field Boundary Manager")
        self.root.geometry("1000x700")
        
        # Initialize shapefile manager
        self.shapefile_manager = ShapefileFieldManager()
        self.current_fields_data = None
        
        self.setup_ui()
        
    def setup_ui(self):
        """Setup the user interface"""
        # Main notebook for tabs
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Tab 1: Load Shapefiles
        self.load_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.load_frame, text="Load Shapefiles")
        self.setup_load_tab()
        
        # Tab 2: Field Management
        self.fields_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.fields_frame, text="Field Management")
        self.setup_fields_tab()
        
        # Tab 3: Path Planning
        self.path_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.path_frame, text="Path Planning")
        self.setup_path_tab()
        
        # Tab 4: Export & Integration
        self.export_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.export_frame, text="Export & Integration")
        self.setup_export_tab()
        
        # Status bar
        self.status_var = tk.StringVar()
        self.status_var.set("Ready - Load a shapefile to get started")
        status_bar = ttk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN)
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)
    
    def setup_load_tab(self):
        """Setup the shapefile loading tab"""
        # Title
        title_label = ttk.Label(self.load_frame, text="Load Field Boundaries from Shapefiles", 
                               font=('Arial', 14, 'bold'))
        title_label.pack(pady=10)
        
        # File selection frame
        file_frame = ttk.LabelFrame(self.load_frame, text="Shapefile Selection", padding=10)
        file_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.file_path_var = tk.StringVar()
        file_entry = ttk.Entry(file_frame, textvariable=self.file_path_var, width=60)
        file_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        
        browse_btn = ttk.Button(file_frame, text="Browse", command=self.browse_shapefile)
        browse_btn.pack(side=tk.RIGHT)
        
        # Load button
        load_btn = ttk.Button(file_frame, text="Load Shapefile", command=self.load_shapefile)
        load_btn.pack(side=tk.RIGHT, padx=(5, 0))
        
        # Dependencies frame
        deps_frame = ttk.LabelFrame(self.load_frame, text="Dependencies", padding=10)
        deps_frame.pack(fill=tk.X, padx=10, pady=5)
        
        deps_info = ttk.Label(deps_frame, 
                             text="Note: Shapefile processing requires GeoPandas, Shapely, and other dependencies.\n" +
                                  "Click 'Install Dependencies' if you haven't installed them yet.")
        deps_info.pack(anchor=tk.W)
        
        deps_btn = ttk.Button(deps_frame, text="Install Dependencies", 
                             command=self.install_dependencies)
        deps_btn.pack(anchor=tk.W, pady=(5, 0))
        
        # Results frame
        results_frame = ttk.LabelFrame(self.load_frame, text="Loading Results", padding=10)
        results_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        self.results_text = scrolledtext.ScrolledText(results_frame, height=15)
        self.results_text.pack(fill=tk.BOTH, expand=True)
    
    def setup_fields_tab(self):
        """Setup the field management tab"""
        # Title
        title_label = ttk.Label(self.fields_frame, text="Field Management", 
                               font=('Arial', 14, 'bold'))
        title_label.pack(pady=10)
        
        # Fields list frame
        list_frame = ttk.LabelFrame(self.fields_frame, text="Available Fields", padding=10)
        list_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Treeview for fields
        columns = ('ID', 'Name', 'Area (ha)', 'Center Lat', 'Center Lon', 'Boundary Points')
        self.fields_tree = ttk.Treeview(list_frame, columns=columns, show='headings', height=10)
        
        for col in columns:
            self.fields_tree.heading(col, text=col)
            self.fields_tree.column(col, width=120)
        
        scrollbar = ttk.Scrollbar(list_frame, orient=tk.VERTICAL, command=self.fields_tree.yview)
        self.fields_tree.configure(yscrollcommand=scrollbar.set)
        
        self.fields_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Field actions frame
        actions_frame = ttk.LabelFrame(self.fields_frame, text="Field Actions", padding=10)
        actions_frame.pack(fill=tk.X, padx=10, pady=5)
        
        select_btn = ttk.Button(actions_frame, text="Select Field", 
                               command=self.select_field)
        select_btn.pack(side=tk.LEFT, padx=(0, 5))
        
        boundary_btn = ttk.Button(actions_frame, text="Show Boundary", 
                                 command=self.show_boundary)
        boundary_btn.pack(side=tk.LEFT, padx=(0, 5))
        
        info_btn = ttk.Button(actions_frame, text="Field Info", 
                             command=self.show_field_info)
        info_btn.pack(side=tk.LEFT, padx=(0, 5))
    
    def setup_path_tab(self):
        """Setup the path planning tab"""
        # Title
        title_label = ttk.Label(self.path_frame, text="Coverage Path Planning", 
                               font=('Arial', 14, 'bold'))
        title_label.pack(pady=10)
        
        # Field selection frame
        field_frame = ttk.LabelFrame(self.path_frame, text="Field Selection", padding=10)
        field_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(field_frame, text="Selected Field:").pack(side=tk.LEFT)
        self.selected_field_var = tk.StringVar()
        self.selected_field_var.set("No field selected")
        field_label = ttk.Label(field_frame, textvariable=self.selected_field_var, 
                               font=('Arial', 10, 'bold'))
        field_label.pack(side=tk.LEFT, padx=(5, 0))
        
        # Parameters frame
        params_frame = ttk.LabelFrame(self.path_frame, text="Path Parameters", padding=10)
        params_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(params_frame, text="Implement Width (m):").grid(row=0, column=0, sticky=tk.W)
        self.swath_width_var = tk.DoubleVar(value=5.0)
        swath_entry = ttk.Entry(params_frame, textvariable=self.swath_width_var, width=10)
        swath_entry.grid(row=0, column=1, padx=(5, 20), sticky=tk.W)
        
        ttk.Label(params_frame, text="Overlap (%):").grid(row=0, column=2, sticky=tk.W)
        self.overlap_var = tk.DoubleVar(value=10.0)
        overlap_entry = ttk.Entry(params_frame, textvariable=self.overlap_var, width=10)
        overlap_entry.grid(row=0, column=3, padx=(5, 0), sticky=tk.W)
        
        # Generate button
        generate_btn = ttk.Button(params_frame, text="Generate Coverage Path", 
                                 command=self.generate_coverage_path)
        generate_btn.grid(row=1, column=0, columnspan=4, pady=(10, 0))
        
        # Results frame
        path_results_frame = ttk.LabelFrame(self.path_frame, text="Path Results", padding=10)
        path_results_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        self.path_text = scrolledtext.ScrolledText(path_results_frame, height=15)
        self.path_text.pack(fill=tk.BOTH, expand=True)
    
    def setup_export_tab(self):
        """Setup the export and integration tab"""
        # Title
        title_label = ttk.Label(self.export_frame, text="Export & ROS2 Integration", 
                               font=('Arial', 14, 'bold'))
        title_label.pack(pady=10)
        
        # Export options frame
        export_frame = ttk.LabelFrame(self.export_frame, text="Export Options", padding=10)
        export_frame.pack(fill=tk.X, padx=10, pady=5)
        
        json_btn = ttk.Button(export_frame, text="Export to JSON", 
                             command=self.export_json)
        json_btn.pack(side=tk.LEFT, padx=(0, 5))
        
        yaml_btn = ttk.Button(export_frame, text="Export to ROS2 YAML", 
                             command=self.export_yaml)
        yaml_btn.pack(side=tk.LEFT, padx=(0, 5))
        
        waypoints_btn = ttk.Button(export_frame, text="Export Waypoints", 
                                  command=self.export_waypoints)
        waypoints_btn.pack(side=tk.LEFT, padx=(0, 5))
        
        # ROS2 integration frame
        ros_frame = ttk.LabelFrame(self.export_frame, text="ROS2 Integration", padding=10)
        ros_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ros_info = ttk.Label(ros_frame, 
                            text="To use with ROS2:\n" +
                                 "1. Export to ROS2 YAML format\n" +
                                 "2. Copy to your ROS2 package params folder\n" +
                                 "3. Use field_boundary_service node\n" +
                                 "4. Launch with: ros2 run tractobots_mission_ui field_boundary_service")
        ros_info.pack(anchor=tk.W)
        
        # Status and logs frame
        logs_frame = ttk.LabelFrame(self.export_frame, text="Export Logs", padding=10)
        logs_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        self.export_text = scrolledtext.ScrolledText(logs_frame, height=10)
        self.export_text.pack(fill=tk.BOTH, expand=True)
    
    def browse_shapefile(self):
        """Browse for shapefile"""
        file_path = filedialog.askopenfilename(
            title="Select Shapefile",
            filetypes=[("Shapefiles", "*.shp"), ("All files", "*.*")]
        )
        if file_path:
            self.file_path_var.set(file_path)
    
    def load_shapefile(self):
        """Load the selected shapefile"""
        file_path = self.file_path_var.get()
        if not file_path:
            messagebox.showerror("Error", "Please select a shapefile first")
            return
        
        if not os.path.exists(file_path):
            messagebox.showerror("Error", "File does not exist")
            return
        
        def load_thread():
            try:
                self.status_var.set("Loading shapefile...")
                self.results_text.delete(1.0, tk.END)
                self.results_text.insert(tk.END, f"Loading shapefile: {file_path}\n\n")
                
                # Load shapefile
                fields_data = self.shapefile_manager.load_field_boundaries(file_path)
                
                if fields_data:
                    self.current_fields_data = fields_data
                    
                    # Update results
                    self.results_text.insert(tk.END, f"✅ Successfully loaded {len(fields_data['fields'])} fields\n\n")
                    
                    # Show field summary
                    for i, field in enumerate(fields_data['fields']):
                        self.results_text.insert(tk.END, f"Field {i+1}: {field['field_id']}\n")
                        self.results_text.insert(tk.END, f"  Name: {field['attributes'].get('NAME', 'Unnamed')}\n")
                        self.results_text.insert(tk.END, f"  Area: {field.get('area_hectares', 0):.2f} hectares\n")
                        self.results_text.insert(tk.END, f"  Center: {field['center']['lat']:.6f}, {field['center']['lon']:.6f}\n")
                        self.results_text.insert(tk.END, f"  Boundary points: {len(field['boundary_points'])}\n\n")
                    
                    # Update fields tree
                    self.update_fields_tree()
                    
                    self.status_var.set(f"Loaded {len(fields_data['fields'])} fields successfully")
                    
                else:
                    self.results_text.insert(tk.END, "❌ Failed to load shapefile\n")
                    self.status_var.set("Failed to load shapefile")
            
            except Exception as e:
                self.results_text.insert(tk.END, f"❌ Error: {str(e)}\n")
                self.status_var.set(f"Error: {str(e)}")
        
        # Run in thread to prevent GUI blocking
        thread = threading.Thread(target=load_thread)
        thread.daemon = True
        thread.start()
    
    def install_dependencies(self):
        """Install required dependencies"""
        def install_thread():
            try:
                self.status_var.set("Installing dependencies...")
                self.results_text.delete(1.0, tk.END)
                self.results_text.insert(tk.END, "Installing shapefile processing dependencies...\n\n")
                
                from shapefile_manager import install_dependencies
                success = install_dependencies()
                
                if success:
                    self.results_text.insert(tk.END, "✅ Dependencies installed successfully!\n")
                    self.status_var.set("Dependencies installed successfully")
                else:
                    self.results_text.insert(tk.END, "❌ Failed to install dependencies\n")
                    self.status_var.set("Failed to install dependencies")
            
            except Exception as e:
                self.results_text.insert(tk.END, f"❌ Error: {str(e)}\n")
                self.status_var.set(f"Error: {str(e)}")
        
        thread = threading.Thread(target=install_thread)
        thread.daemon = True
        thread.start()
    
    def update_fields_tree(self):
        """Update the fields tree with current data"""
        # Clear existing items
        for item in self.fields_tree.get_children():
            self.fields_tree.delete(item)
        
        if not self.current_fields_data:
            return
        
        # Add fields to tree
        for field in self.current_fields_data['fields']:
            values = (
                field['field_id'],
                field['attributes'].get('NAME', 'Unnamed'),
                f"{field.get('area_hectares', 0):.2f}",
                f"{field['center']['lat']:.6f}",
                f"{field['center']['lon']:.6f}",
                len(field['boundary_points'])
            )
            self.fields_tree.insert('', tk.END, values=values)
    
    def select_field(self):
        """Select a field for path planning"""
        selection = self.fields_tree.selection()
        if not selection:
            messagebox.showwarning("Warning", "Please select a field from the list")
            return
        
        item = self.fields_tree.item(selection[0])
        field_id = item['values'][0]
        field_name = item['values'][1]
        
        self.selected_field_var.set(f"{field_name} ({field_id})")
        self.current_selected_field = field_id
        
        messagebox.showinfo("Info", f"Selected field: {field_name}")
    
    def show_boundary(self):
        """Show field boundary information"""
        selection = self.fields_tree.selection()
        if not selection:
            messagebox.showwarning("Warning", "Please select a field from the list")
            return
        
        item = self.fields_tree.item(selection[0])
        field_id = item['values'][0]
        
        # Find field data
        field_data = None
        for field in self.current_fields_data['fields']:
            if field['field_id'] == field_id:
                field_data = field
                break
        
        if field_data:
            boundary_info = f"Field: {field_data['attributes'].get('NAME', field_id)}\n"
            boundary_info += f"Boundary Points: {len(field_data['boundary_points'])}\n\n"
            boundary_info += "Coordinates (Lat, Lon):\n"
            
            boundary_points = field_data['boundary_points']
            if isinstance(boundary_points[0], list):
                boundary_points = boundary_points[0]
            
            for i, point in enumerate(boundary_points[:10]):  # Show first 10 points
                boundary_info += f"{i+1}: {point['lat']:.6f}, {point['lon']:.6f}\n"
            
            if len(boundary_points) > 10:
                boundary_info += f"... and {len(boundary_points) - 10} more points\n"
            
            messagebox.showinfo("Field Boundary", boundary_info)
    
    def show_field_info(self):
        """Show detailed field information"""
        selection = self.fields_tree.selection()
        if not selection:
            messagebox.showwarning("Warning", "Please select a field from the list")
            return
        
        item = self.fields_tree.item(selection[0])
        field_id = item['values'][0]
        
        # Find field data
        field_data = None
        for field in self.current_fields_data['fields']:
            if field['field_id'] == field_id:
                field_data = field
                break
        
        if field_data:
            info_window = tk.Toplevel(self.root)
            info_window.title(f"Field Information - {field_id}")
            info_window.geometry("500x400")
            
            info_text = scrolledtext.ScrolledText(info_window)
            info_text.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
            
            info_content = f"Field ID: {field_data['field_id']}\n"
            info_content += f"Name: {field_data['attributes'].get('NAME', 'Unnamed')}\n"
            info_content += f"Area: {field_data.get('area_hectares', 0):.2f} hectares\n"
            info_content += f"Center: {field_data['center']['lat']:.6f}, {field_data['center']['lon']:.6f}\n"
            info_content += f"Geometry Type: {field_data['geometry_type']}\n"
            info_content += f"Boundary Points: {len(field_data['boundary_points'])}\n\n"
            
            info_content += "Attributes:\n"
            for key, value in field_data['attributes'].items():
                info_content += f"  {key}: {value}\n"
            
            info_text.insert(tk.END, info_content)
            info_text.config(state=tk.DISABLED)
    
    def generate_coverage_path(self):
        """Generate coverage path for selected field"""
        if not hasattr(self, 'current_selected_field'):
            messagebox.showwarning("Warning", "Please select a field first")
            return
        
        def generate_thread():
            try:
                self.status_var.set("Generating coverage path...")
                self.path_text.delete(1.0, tk.END)
                
                swath_width = self.swath_width_var.get()
                overlap = self.overlap_var.get() / 100  # Convert percentage to decimal
                
                self.path_text.insert(tk.END, f"Generating coverage path for {self.current_selected_field}\n")
                self.path_text.insert(tk.END, f"Implement width: {swath_width} m\n")
                self.path_text.insert(tk.END, f"Overlap: {overlap*100}%\n\n")
                
                waypoints = self.shapefile_manager.generate_coverage_path(
                    self.current_fields_data, self.current_selected_field, 
                    swath_width, overlap
                )
                
                if waypoints:
                    self.current_waypoints = waypoints
                    self.path_text.insert(tk.END, f"✅ Generated {len(waypoints)} waypoints\n\n")
                    
                    # Show first few waypoints
                    for i, (lat, lon) in enumerate(waypoints[:10]):
                        self.path_text.insert(tk.END, f"Waypoint {i+1}: {lat:.6f}, {lon:.6f}\n")
                    
                    if len(waypoints) > 10:
                        self.path_text.insert(tk.END, f"... and {len(waypoints) - 10} more waypoints\n")
                    
                    self.status_var.set(f"Generated {len(waypoints)} waypoints")
                else:
                    self.path_text.insert(tk.END, "❌ Failed to generate coverage path\n")
                    self.status_var.set("Failed to generate coverage path")
            
            except Exception as e:
                self.path_text.insert(tk.END, f"❌ Error: {str(e)}\n")
                self.status_var.set(f"Error: {str(e)}")
        
        thread = threading.Thread(target=generate_thread)
        thread.daemon = True
        thread.start()
    
    def export_json(self):
        """Export field data to JSON"""
        if not self.current_fields_data:
            messagebox.showwarning("Warning", "No field data to export")
            return
        
        file_path = filedialog.asksaveasfilename(
            title="Save Field Data as JSON",
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        
        if file_path:
            try:
                with open(file_path, 'w') as f:
                    json.dump(self.current_fields_data, f, indent=2)
                
                self.export_text.insert(tk.END, f"✅ Exported field data to {file_path}\n")
                messagebox.showinfo("Success", f"Field data exported to {file_path}")
            
            except Exception as e:
                self.export_text.insert(tk.END, f"❌ Error exporting: {str(e)}\n")
                messagebox.showerror("Error", f"Failed to export: {str(e)}")
    
    def export_yaml(self):
        """Export field data to ROS2 YAML format"""
        if not self.current_fields_data:
            messagebox.showwarning("Warning", "No field data to export")
            return
        
        file_path = filedialog.asksaveasfilename(
            title="Save as ROS2 YAML",
            defaultextension=".yaml",
            filetypes=[("YAML files", "*.yaml"), ("All files", "*.*")]
        )
        
        if file_path:
            try:
                success = self.shapefile_manager.generate_ros2_field_config(
                    self.current_fields_data, file_path
                )
                
                if success:
                    self.export_text.insert(tk.END, f"✅ Exported ROS2 config to {file_path}\n")
                    messagebox.showinfo("Success", f"ROS2 config exported to {file_path}")
                else:
                    self.export_text.insert(tk.END, f"❌ Failed to export ROS2 config\n")
                    messagebox.showerror("Error", "Failed to export ROS2 config")
            
            except Exception as e:
                self.export_text.insert(tk.END, f"❌ Error exporting: {str(e)}\n")
                messagebox.showerror("Error", f"Failed to export: {str(e)}")
    
    def export_waypoints(self):
        """Export current waypoints"""
        if not hasattr(self, 'current_waypoints') or not self.current_waypoints:
            messagebox.showwarning("Warning", "No waypoints to export. Generate a coverage path first.")
            return
        
        file_path = filedialog.asksaveasfilename(
            title="Save Waypoints",
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("CSV files", "*.csv"), ("All files", "*.*")]
        )
        
        if file_path:
            try:
                if file_path.endswith('.csv'):
                    # Export as CSV
                    import csv
                    with open(file_path, 'w', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow(['Waypoint', 'Latitude', 'Longitude'])
                        for i, (lat, lon) in enumerate(self.current_waypoints):
                            writer.writerow([i+1, lat, lon])
                else:
                    # Export as JSON
                    waypoints_data = {
                        'field_id': self.current_selected_field,
                        'waypoints': [{'lat': lat, 'lon': lon} for lat, lon in self.current_waypoints],
                        'metadata': {
                            'num_waypoints': len(self.current_waypoints),
                            'swath_width': self.swath_width_var.get(),
                            'overlap': self.overlap_var.get()
                        }
                    }
                    with open(file_path, 'w') as f:
                        json.dump(waypoints_data, f, indent=2)
                
                self.export_text.insert(tk.END, f"✅ Exported {len(self.current_waypoints)} waypoints to {file_path}\n")
                messagebox.showinfo("Success", f"Waypoints exported to {file_path}")
            
            except Exception as e:
                self.export_text.insert(tk.END, f"❌ Error exporting: {str(e)}\n")
                messagebox.showerror("Error", f"Failed to export: {str(e)}")
    
    def run(self):
        """Run the GUI"""
        self.root.mainloop()


def main():
    """Main entry point"""
    app = ShapefileGUI()
    app.run()


if __name__ == '__main__':
    main()
