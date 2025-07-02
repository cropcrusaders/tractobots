#!/usr/bin/env python3
"""
Tractobots Desktop Application
Modern desktop app for autonomous tractor field management with shapefile integration
"""

import sys
import os
import json
import subprocess
from pathlib import Path
from typing import Dict, List, Optional
import threading
import time

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QGridLayout, QTabWidget, QLabel, QPushButton, QTextEdit, QListWidget,
    QFileDialog, QMessageBox, QProgressBar, QGroupBox, QFormLayout,
    QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox, QCheckBox,
    QSplitter, QFrame, QScrollArea, QTableWidget, QTableWidgetItem,
    QMenuBar, QMenu, QAction, QStatusBar, QToolBar
)
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal, QSettings
from PyQt5.QtGui import QFont, QPixmap, QIcon, QPalette, QColor

# Try to import ROS2 and shapefile dependencies
try:
    import rclpy
    from rclpy.node import Node
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

try:
    import geopandas as gpd
    import shapely
    SHAPEFILE_AVAILABLE = True
except ImportError:
    SHAPEFILE_AVAILABLE = False

class FieldDataWorker(QThread):
    """Worker thread for processing shapefile data"""
    progress = pyqtSignal(int)
    finished = pyqtSignal(dict)
    error = pyqtSignal(str)
    
    def __init__(self, shapefile_path: str):
        super().__init__()
        self.shapefile_path = shapefile_path
    
    def run(self):
        try:
            if not SHAPEFILE_AVAILABLE:
                self.error.emit("GeoPandas not available. Install with: sudo apt install python3-geopandas")
                return
            
            # Add shapefile manager to path
            sys.path.append('src/tractobots_mission_ui/tractobots_mission_ui')
            
            self.progress.emit(20)
            
            from shapefile_manager import ShapefileFieldManager
            manager = ShapefileFieldManager()
            
            self.progress.emit(50)
            
            # Load field boundaries
            fields_data = manager.load_field_boundaries(self.shapefile_path)
            
            self.progress.emit(80)
            
            if fields_data:
                self.finished.emit(fields_data)
            else:
                self.error.emit("Failed to load field boundaries from shapefile")
                
            self.progress.emit(100)
            
        except Exception as e:
            self.error.emit(f"Error processing shapefile: {str(e)}")

class PathPlanningWorker(QThread):
    """Worker thread for generating coverage paths"""
    progress = pyqtSignal(int)
    finished = pyqtSignal(dict)
    error = pyqtSignal(str)
    
    def __init__(self, fields_data: dict, field_id: str, swath_width: float):
        super().__init__()
        self.fields_data = fields_data
        self.field_id = field_id
        self.swath_width = swath_width
    
    def run(self):
        try:
            sys.path.append('src/tractobots_mission_ui/tractobots_mission_ui')
            from shapefile_manager import ShapefileFieldManager
            
            self.progress.emit(30)
            
            manager = ShapefileFieldManager()
            path_data = manager.generate_coverage_path(
                self.fields_data, 
                self.field_id, 
                swath_width=self.swath_width
            )
            
            self.progress.emit(80)
            
            if path_data:
                self.finished.emit(path_data)
            else:
                self.error.emit("Failed to generate coverage path")
                
            self.progress.emit(100)
            
        except Exception as e:
            self.error.emit(f"Error generating path: {str(e)}")

class StatusWidget(QWidget):
    """System status display widget"""
    
    def __init__(self):
        super().__init__()
        self.init_ui()
        self.update_status()
        
        # Timer for periodic updates
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_status)
        self.timer.start(5000)  # Update every 5 seconds
    
    def init_ui(self):
        layout = QGridLayout()
        
        # ROS2 Status
        self.ros2_label = QLabel("ROS2: Checking...")
        self.ros2_label.setStyleSheet("color: orange;")
        layout.addWidget(QLabel("🤖"), 0, 0)
        layout.addWidget(self.ros2_label, 0, 1)
        
        # Shapefile Support
        self.shapefile_label = QLabel("Shapefiles: Checking...")
        self.shapefile_label.setStyleSheet("color: orange;")
        layout.addWidget(QLabel("🗺️"), 1, 0)
        layout.addWidget(self.shapefile_label, 1, 1)
        
        # GPS Status (placeholder)
        self.gps_label = QLabel("GPS: No Data")
        self.gps_label.setStyleSheet("color: gray;")
        layout.addWidget(QLabel("📍"), 2, 0)
        layout.addWidget(self.gps_label, 2, 1)
        
        # Mission Status
        self.mission_label = QLabel("Mission: Idle")
        self.mission_label.setStyleSheet("color: blue;")
        layout.addWidget(QLabel("🎯"), 3, 0)
        layout.addWidget(self.mission_label, 3, 1)
        
        self.setLayout(layout)
    
    def update_status(self):
        # Update ROS2 status
        if ROS2_AVAILABLE:
            self.ros2_label.setText("ROS2: Available")
            self.ros2_label.setStyleSheet("color: green;")
        else:
            self.ros2_label.setText("ROS2: Not Available")
            self.ros2_label.setStyleSheet("color: red;")
        
        # Update Shapefile status
        if SHAPEFILE_AVAILABLE:
            self.shapefile_label.setText("Shapefiles: Ready")
            self.shapefile_label.setStyleSheet("color: green;")
        else:
            self.shapefile_label.setText("Shapefiles: Install GeoPandas")
            self.shapefile_label.setStyleSheet("color: red;")

class FieldManagementTab(QWidget):
    """Field management and shapefile loading tab"""
    
    def __init__(self):
        super().__init__()
        self.fields_data = None
        self.init_ui()
    
    def init_ui(self):
        layout = QHBoxLayout()
        
        # Left side - Controls
        left_panel = QWidget()
        left_layout = QVBoxLayout()
        
        # File loading section
        file_group = QGroupBox("Shapefile Loading")
        file_layout = QVBoxLayout()
        
        self.file_path_edit = QLineEdit()
        self.file_path_edit.setPlaceholderText("Select shapefile...")
        file_layout.addWidget(self.file_path_edit)
        
        file_buttons = QHBoxLayout()
        self.browse_btn = QPushButton("Browse...")
        self.browse_btn.clicked.connect(self.browse_shapefile)
        file_buttons.addWidget(self.browse_btn)
        
        self.load_btn = QPushButton("Load Shapefile")
        self.load_btn.clicked.connect(self.load_shapefile)
        self.load_btn.setEnabled(False)
        file_buttons.addWidget(self.load_btn)
        
        file_layout.addLayout(file_buttons)
        
        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        file_layout.addWidget(self.progress_bar)
        
        file_group.setLayout(file_layout)
        left_layout.addWidget(file_group)
        
        # Field list section
        fields_group = QGroupBox("Field Boundaries")
        fields_layout = QVBoxLayout()
        
        self.fields_table = QTableWidget()
        self.fields_table.setColumnCount(4)
        self.fields_table.setHorizontalHeaderLabels(["Field ID", "Name", "Crop", "Area"])
        fields_layout.addWidget(self.fields_table)
        
        fields_group.setLayout(fields_layout)
        left_layout.addWidget(fields_group)
        
        left_panel.setLayout(left_layout)
        layout.addWidget(left_panel, 1)
        
        # Right side - Field details
        right_panel = QWidget()
        right_layout = QVBoxLayout()
        
        details_group = QGroupBox("Field Details")
        details_layout = QFormLayout()
        
        self.field_id_label = QLabel("No field selected")
        details_layout.addRow("Field ID:", self.field_id_label)
        
        self.field_name_label = QLabel("-")
        details_layout.addRow("Name:", self.field_name_label)
        
        self.field_crop_label = QLabel("-")
        details_layout.addRow("Crop:", self.field_crop_label)
        
        self.field_area_label = QLabel("-")
        details_layout.addRow("Area:", self.field_area_label)
        
        self.boundary_points_label = QLabel("-")
        details_layout.addRow("Boundary Points:", self.boundary_points_label)
        
        details_group.setLayout(details_layout)
        right_layout.addWidget(details_group)
        
        # Field actions
        actions_group = QGroupBox("Field Actions")
        actions_layout = QVBoxLayout()
        
        self.export_field_btn = QPushButton("Export Selected Field")
        self.export_field_btn.clicked.connect(self.export_selected_field)
        self.export_field_btn.setEnabled(False)
        actions_layout.addWidget(self.export_field_btn)
        
        self.export_all_btn = QPushButton("Export All Fields")
        self.export_all_btn.clicked.connect(self.export_all_fields)
        self.export_all_btn.setEnabled(False)
        actions_layout.addWidget(self.export_all_btn)
        
        actions_group.setLayout(actions_layout)
        right_layout.addWidget(actions_group)
        
        right_layout.addStretch()
        right_panel.setLayout(right_layout)
        layout.addWidget(right_panel, 1)
        
        self.setLayout(layout)
        
        # Connect table selection
        self.fields_table.itemSelectionChanged.connect(self.on_field_selected)
    
    def browse_shapefile(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self, 
            "Select Shapefile", 
            str(Path.home()), 
            "Shapefiles (*.shp);;All Files (*)"
        )
        
        if file_path:
            self.file_path_edit.setText(file_path)
            self.load_btn.setEnabled(True)
    
    def load_shapefile(self):
        shapefile_path = self.file_path_edit.text()
        if not shapefile_path or not Path(shapefile_path).exists():
            QMessageBox.warning(self, "Error", "Please select a valid shapefile")
            return
        
        self.progress_bar.setVisible(True)
        self.progress_bar.setValue(0)
        self.load_btn.setEnabled(False)
        
        # Start worker thread
        self.worker = FieldDataWorker(shapefile_path)
        self.worker.progress.connect(self.progress_bar.setValue)
        self.worker.finished.connect(self.on_fields_loaded)
        self.worker.error.connect(self.on_load_error)
        self.worker.start()
    
    def on_fields_loaded(self, fields_data):
        self.fields_data = fields_data
        self.populate_fields_table()
        self.progress_bar.setVisible(False)
        self.load_btn.setEnabled(True)
        self.export_all_btn.setEnabled(True)
        
        QMessageBox.information(
            self, 
            "Success", 
            f"Loaded {len(fields_data.get('fields', []))} field boundaries"
        )
    
    def on_load_error(self, error_msg):
        self.progress_bar.setVisible(False)
        self.load_btn.setEnabled(True)
        QMessageBox.critical(self, "Error", error_msg)
    
    def populate_fields_table(self):
        if not self.fields_data or 'fields' not in self.fields_data:
            return
        
        fields = self.fields_data['fields']
        self.fields_table.setRowCount(len(fields))
        
        for row, field in enumerate(fields):
            field_id = field.get('field_id', 'Unknown')
            attrs = field.get('attributes', {})
            
            self.fields_table.setItem(row, 0, QTableWidgetItem(field_id))
            self.fields_table.setItem(row, 1, QTableWidgetItem(attrs.get('field_name', 'Unknown')))
            self.fields_table.setItem(row, 2, QTableWidgetItem(attrs.get('crop_type', 'Unknown')))
            self.fields_table.setItem(row, 3, QTableWidgetItem(str(attrs.get('area_acres', 'Unknown'))))
        
        self.fields_table.resizeColumnsToContents()
    
    def on_field_selected(self):
        current_row = self.fields_table.currentRow()
        if current_row >= 0 and self.fields_data:
            field = self.fields_data['fields'][current_row]
            attrs = field.get('attributes', {})
            
            self.field_id_label.setText(field.get('field_id', 'Unknown'))
            self.field_name_label.setText(attrs.get('field_name', 'Unknown'))
            self.field_crop_label.setText(attrs.get('crop_type', 'Unknown'))
            self.field_area_label.setText(f"{attrs.get('area_acres', 'Unknown')} acres")
            
            boundary_points = field.get('boundary_points', [])
            self.boundary_points_label.setText(f"{len(boundary_points)} points")
            
            self.export_field_btn.setEnabled(True)
    
    def export_selected_field(self):
        current_row = self.fields_table.currentRow()
        if current_row >= 0 and self.fields_data:
            field = self.fields_data['fields'][current_row]
            field_id = field.get('field_id', 'Unknown')
            
            # Export single field
            self.export_field_data([field], f"field_{field_id}")
    
    def export_all_fields(self):
        if self.fields_data and 'fields' in self.fields_data:
            self.export_field_data(self.fields_data['fields'], "all_fields")
    
    def export_field_data(self, fields, filename):
        try:
            export_path, _ = QFileDialog.getSaveFileName(
                self,
                "Export Field Data",
                f"{filename}.yaml",
                "YAML Files (*.yaml);;JSON Files (*.json);;All Files (*)"
            )
            
            if export_path:
                export_data = {
                    'fields': fields,
                    'metadata': {
                        'exported_at': time.strftime('%Y-%m-%d %H:%M:%S'),
                        'source': 'Tractobots Desktop App'
                    }
                }
                
                if export_path.endswith('.json'):
                    with open(export_path, 'w') as f:
                        json.dump(export_data, f, indent=2)
                else:
                    # Default to YAML
                    try:
                        import yaml
                        with open(export_path, 'w') as f:
                            yaml.dump(export_data, f, default_flow_style=False)
                    except ImportError:
                        # Fallback to JSON if yaml not available
                        with open(export_path, 'w') as f:
                            json.dump(export_data, f, indent=2)
                
                QMessageBox.information(self, "Export Complete", f"Field data exported to {export_path}")
                
        except Exception as e:
            QMessageBox.critical(self, "Export Error", f"Failed to export: {str(e)}")

class PathPlanningTab(QWidget):
    """Path planning and coverage path generation tab"""
    
    def __init__(self):
        super().__init__()
        self.fields_data = None
        self.current_path = None
        self.init_ui()
    
    def init_ui(self):
        layout = QHBoxLayout()
        
        # Left panel - Controls
        left_panel = QWidget()
        left_layout = QVBoxLayout()
        
        # Path planning settings
        settings_group = QGroupBox("Path Planning Settings")
        settings_layout = QFormLayout()
        
        self.field_combo = QComboBox()
        self.field_combo.currentTextChanged.connect(self.on_field_changed)
        settings_layout.addRow("Field:", self.field_combo)
        
        self.swath_width_spin = QDoubleSpinBox()
        self.swath_width_spin.setRange(1.0, 20.0)
        self.swath_width_spin.setValue(6.0)
        self.swath_width_spin.setSuffix(" m")
        settings_layout.addRow("Swath Width:", self.swath_width_spin)
        
        self.overlap_spin = QDoubleSpinBox()
        self.overlap_spin.setRange(0.0, 2.0)
        self.overlap_spin.setValue(0.5)
        self.overlap_spin.setSuffix(" m")
        settings_layout.addRow("Overlap:", self.overlap_spin)
        
        self.pattern_combo = QComboBox()
        self.pattern_combo.addItems(["Parallel", "Spiral", "Zigzag"])
        settings_layout.addRow("Pattern:", self.pattern_combo)
        
        settings_group.setLayout(settings_layout)
        left_layout.addWidget(settings_group)
        
        # Generate button
        self.generate_btn = QPushButton("Generate Coverage Path")
        self.generate_btn.clicked.connect(self.generate_path)
        self.generate_btn.setEnabled(False)
        left_layout.addWidget(self.generate_btn)
        
        self.path_progress = QProgressBar()
        self.path_progress.setVisible(False)
        left_layout.addWidget(self.path_progress)
        
        # Path statistics
        stats_group = QGroupBox("Path Statistics")
        stats_layout = QFormLayout()
        
        self.waypoints_label = QLabel("0")
        stats_layout.addRow("Waypoints:", self.waypoints_label)
        
        self.distance_label = QLabel("0.0 km")
        stats_layout.addRow("Total Distance:", self.distance_label)
        
        self.time_label = QLabel("0.0 min")
        stats_layout.addRow("Est. Time:", self.time_label)
        
        stats_group.setLayout(stats_layout)
        left_layout.addWidget(stats_group)
        
        left_layout.addStretch()
        left_panel.setLayout(left_layout)
        layout.addWidget(left_panel, 1)
        
        # Right panel - Path details
        right_panel = QWidget()
        right_layout = QVBoxLayout()
        
        # Path export
        export_group = QGroupBox("Path Export")
        export_layout = QVBoxLayout()
        
        self.export_path_btn = QPushButton("Export to ROS2 Format")
        self.export_path_btn.clicked.connect(self.export_path)
        self.export_path_btn.setEnabled(False)
        export_layout.addWidget(self.export_path_btn)
        
        self.export_waypoints_btn = QPushButton("Export Waypoints CSV")
        self.export_waypoints_btn.clicked.connect(self.export_waypoints)
        self.export_waypoints_btn.setEnabled(False)
        export_layout.addWidget(self.export_waypoints_btn)
        
        export_group.setLayout(export_layout)
        right_layout.addWidget(export_group)
        
        # Waypoints list
        waypoints_group = QGroupBox("Waypoints Preview")
        waypoints_layout = QVBoxLayout()
        
        self.waypoints_list = QListWidget()
        waypoints_layout.addWidget(self.waypoints_list)
        
        waypoints_group.setLayout(waypoints_layout)
        right_layout.addWidget(waypoints_group)
        
        right_panel.setLayout(right_layout)
        layout.addWidget(right_panel, 1)
        
        self.setLayout(layout)
    
    def update_fields(self, fields_data):
        """Update available fields from shapefile data"""
        self.fields_data = fields_data
        self.field_combo.clear()
        
        if fields_data and 'fields' in fields_data:
            for field in fields_data['fields']:
                field_id = field.get('field_id', 'Unknown')
                field_name = field.get('attributes', {}).get('field_name', 'Unknown')
                self.field_combo.addItem(f"{field_name} ({field_id})", field_id)
            
            self.generate_btn.setEnabled(True)
    
    def on_field_changed(self):
        # Reset path when field changes
        self.current_path = None
        self.update_path_stats()
    
    def generate_path(self):
        if not self.fields_data:
            QMessageBox.warning(self, "No Fields", "Please load field boundaries first")
            return
        
        field_id = self.field_combo.currentData()
        if not field_id:
            return
        
        swath_width = self.swath_width_spin.value()
        
        self.path_progress.setVisible(True)
        self.path_progress.setValue(0)
        self.generate_btn.setEnabled(False)
        
        # Start path generation worker
        self.path_worker = PathPlanningWorker(self.fields_data, field_id, swath_width)
        self.path_worker.progress.connect(self.path_progress.setValue)
        self.path_worker.finished.connect(self.on_path_generated)
        self.path_worker.error.connect(self.on_path_error)
        self.path_worker.start()
    
    def on_path_generated(self, path_data):
        self.current_path = path_data
        self.update_path_stats()
        self.update_waypoints_list()
        
        self.path_progress.setVisible(False)
        self.generate_btn.setEnabled(True)
        self.export_path_btn.setEnabled(True)
        self.export_waypoints_btn.setEnabled(True)
        
        QMessageBox.information(
            self, 
            "Path Generated", 
            f"Coverage path generated with {len(path_data.get('waypoints', []))} waypoints"
        )
    
    def on_path_error(self, error_msg):
        self.path_progress.setVisible(False)
        self.generate_btn.setEnabled(True)
        QMessageBox.critical(self, "Path Generation Error", error_msg)
    
    def update_path_stats(self):
        if not self.current_path:
            self.waypoints_label.setText("0")
            self.distance_label.setText("0.0 km")
            self.time_label.setText("0.0 min")
            return
        
        waypoints = self.current_path.get('waypoints', [])
        self.waypoints_label.setText(str(len(waypoints)))
        
        # Calculate approximate distance and time
        if len(waypoints) > 1:
            total_distance = 0
            for i in range(1, len(waypoints)):
                # Simple distance calculation (not geodesic)
                prev = waypoints[i-1]
                curr = waypoints[i]
                dx = curr['lon'] - prev['lon']
                dy = curr['lat'] - prev['lat']
                total_distance += (dx**2 + dy**2)**0.5 * 111000  # Rough conversion to meters
            
            distance_km = total_distance / 1000
            self.distance_label.setText(f"{distance_km:.2f} km")
            
            # Estimate time at 5 km/h
            time_hours = distance_km / 5.0
            time_minutes = time_hours * 60
            self.time_label.setText(f"{time_minutes:.1f} min")
    
    def update_waypoints_list(self):
        self.waypoints_list.clear()
        
        if not self.current_path:
            return
        
        waypoints = self.current_path.get('waypoints', [])
        for i, wp in enumerate(waypoints[:20]):  # Show first 20 waypoints
            lat = wp.get('lat', 0)
            lon = wp.get('lon', 0)
            self.waypoints_list.addItem(f"WP {i+1}: {lat:.6f}, {lon:.6f}")
        
        if len(waypoints) > 20:
            self.waypoints_list.addItem(f"... and {len(waypoints) - 20} more waypoints")
    
    def export_path(self):
        if not self.current_path:
            return
        
        try:
            export_path, _ = QFileDialog.getSaveFileName(
                self,
                "Export Coverage Path",
                "coverage_path.yaml",
                "YAML Files (*.yaml);;JSON Files (*.json);;All Files (*)"
            )
            
            if export_path:
                export_data = {
                    'coverage_path': self.current_path,
                    'metadata': {
                        'generated_at': time.strftime('%Y-%m-%d %H:%M:%S'),
                        'swath_width': self.swath_width_spin.value(),
                        'overlap': self.overlap_spin.value(),
                        'pattern': self.pattern_combo.currentText()
                    }
                }
                
                if export_path.endswith('.json'):
                    with open(export_path, 'w') as f:
                        json.dump(export_data, f, indent=2)
                else:
                    try:
                        import yaml
                        with open(export_path, 'w') as f:
                            yaml.dump(export_data, f, default_flow_style=False)
                    except ImportError:
                        with open(export_path, 'w') as f:
                            json.dump(export_data, f, indent=2)
                
                QMessageBox.information(self, "Export Complete", f"Path exported to {export_path}")
                
        except Exception as e:
            QMessageBox.critical(self, "Export Error", f"Failed to export path: {str(e)}")
    
    def export_waypoints(self):
        if not self.current_path:
            return
        
        try:
            export_path, _ = QFileDialog.getSaveFileName(
                self,
                "Export Waypoints",
                "waypoints.csv",
                "CSV Files (*.csv);;All Files (*)"
            )
            
            if export_path:
                waypoints = self.current_path.get('waypoints', [])
                
                with open(export_path, 'w') as f:
                    f.write("index,latitude,longitude\n")
                    for i, wp in enumerate(waypoints):
                        f.write(f"{i},{wp.get('lat', 0)},{wp.get('lon', 0)}\n")
                
                QMessageBox.information(self, "Export Complete", f"Waypoints exported to {export_path}")
                
        except Exception as e:
            QMessageBox.critical(self, "Export Error", f"Failed to export waypoints: {str(e)}")

class TractorbotsDesktopApp(QMainWindow):
    """Main desktop application window"""
    
    def __init__(self):
        super().__init__()
        self.settings = QSettings('Tractobots', 'DesktopApp')
        self.init_ui()
        self.init_menu()
        self.init_status_bar()
        self.restore_settings()
    
    def init_ui(self):
        self.setWindowTitle("Tractobots Field Management System")
        self.setGeometry(100, 100, 1200, 800)
        
        # Set modern style
        self.setStyleSheet("""
            QMainWindow {
                background-color: #f5f5f5;
            }
            QTabWidget::pane {
                border: 1px solid #c0c0c0;
                background-color: white;
            }
            QTabBar::tab {
                background-color: #e1e1e1;
                padding: 8px 16px;
                margin-right: 2px;
            }
            QTabBar::tab:selected {
                background-color: white;
                border-bottom: 2px solid #0078d4;
            }
            QGroupBox {
                font-weight: bold;
                border: 2px solid #cccccc;
                border-radius: 5px;
                margin-top: 1ex;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
            QPushButton {
                background-color: #0078d4;
                color: white;
                border: none;
                padding: 8px 16px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #106ebe;
            }
            QPushButton:pressed {
                background-color: #005a9e;
            }
            QPushButton:disabled {
                background-color: #cccccc;
                color: #666666;
            }
        """)
        
        # Central widget with tabs
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        layout = QVBoxLayout()
        
        # Header with status
        header_widget = QWidget()
        header_layout = QHBoxLayout()
        
        # App title
        title_label = QLabel("🚜 Tractobots Field Management System")
        title_label.setFont(QFont("Arial", 16, QFont.Bold))
        header_layout.addWidget(title_label)
        
        header_layout.addStretch()
        
        # Status widget
        self.status_widget = StatusWidget()
        header_layout.addWidget(self.status_widget)
        
        header_widget.setLayout(header_layout)
        layout.addWidget(header_widget)
        
        # Main tabs
        self.tabs = QTabWidget()
        
        # Field Management tab
        self.field_tab = FieldManagementTab()
        self.tabs.addTab(self.field_tab, "🗺️ Field Management")
        
        # Path Planning tab
        self.path_tab = PathPlanningTab()
        self.tabs.addTab(self.path_tab, "🛣️ Path Planning")
        
        # Connect field updates to path planning
        self.field_tab.fields_table.itemSelectionChanged.connect(self.update_path_planning)
        
        layout.addWidget(self.tabs)
        central_widget.setLayout(layout)
    
    def init_menu(self):
        menubar = self.menuBar()
        
        # File menu
        file_menu = menubar.addMenu('File')
        
        open_action = QAction('Open Shapefile...', self)
        open_action.setShortcut('Ctrl+O')
        open_action.triggered.connect(self.field_tab.browse_shapefile)
        file_menu.addAction(open_action)
        
        file_menu.addSeparator()
        
        exit_action = QAction('Exit', self)
        exit_action.setShortcut('Ctrl+Q')
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)
        
        # Tools menu
        tools_menu = menubar.addMenu('Tools')
        
        launch_ros_action = QAction('Launch ROS2 Components', self)
        launch_ros_action.triggered.connect(self.launch_ros2_components)
        tools_menu.addAction(launch_ros_action)
        
        system_check_action = QAction('System Check', self)
        system_check_action.triggered.connect(self.run_system_check)
        tools_menu.addAction(system_check_action)
        
        # Help menu
        help_menu = menubar.addMenu('Help')
        
        about_action = QAction('About', self)
        about_action.triggered.connect(self.show_about)
        help_menu.addAction(about_action)
        
        docs_action = QAction('Documentation', self)
        docs_action.triggered.connect(self.show_documentation)
        help_menu.addAction(docs_action)
    
    def init_status_bar(self):
        self.status_bar = self.statusBar()
        self.status_bar.showMessage("Ready - Load a shapefile to begin")
    
    def update_path_planning(self):
        """Update path planning tab when fields are loaded"""
        if hasattr(self.field_tab, 'fields_data') and self.field_tab.fields_data:
            self.path_tab.update_fields(self.field_tab.fields_data)
            self.status_bar.showMessage(f"Loaded {len(self.field_tab.fields_data.get('fields', []))} fields")
    
    def launch_ros2_components(self):
        """Launch ROS2 components in separate processes"""
        if not ROS2_AVAILABLE:
            QMessageBox.warning(
                self,
                "ROS2 Not Available",
                "ROS2 is not available. Please install ROS2 first."
            )
            return
        
        try:
            # Try to launch field boundary service
            subprocess.Popen([
                'ros2', 'run', 'tractobots_mission_ui', 'field_boundary_service'
            ])
            
            QMessageBox.information(
                self,
                "ROS2 Components",
                "Launching ROS2 field boundary service in background"
            )
            
        except Exception as e:
            QMessageBox.critical(
                self,
                "Launch Error",
                f"Failed to launch ROS2 components: {str(e)}"
            )
    
    def run_system_check(self):
        """Run system diagnostics"""
        try:
            result = subprocess.run(['python3', 'diagnose_problems.py'], 
                                  capture_output=True, text=True, cwd='.')
            
            # Show results in a dialog
            dialog = QMessageBox()
            dialog.setWindowTitle("System Check Results")
            dialog.setText("System diagnostic completed")
            dialog.setDetailedText(result.stdout)
            dialog.exec_()
            
        except Exception as e:
            QMessageBox.critical(self, "System Check Error", f"Failed to run diagnostics: {str(e)}")
    
    def show_about(self):
        QMessageBox.about(
            self,
            "About Tractobots",
            """
            <h3>Tractobots Field Management System</h3>
            <p>Desktop application for autonomous tractor field operations</p>
            <p><b>Features:</b></p>
            <ul>
            <li>Shapefile import and processing</li>
            <li>Field boundary management</li>
            <li>Coverage path planning</li>
            <li>ROS2 integration</li>
            </ul>
            <p><b>Version:</b> 1.0.0</p>
            <p><b>Built with:</b> PyQt5, GeoPandas, ROS2</p>
            """
        )
    
    def show_documentation(self):
        """Open documentation in external browser or viewer"""
        try:
            import webbrowser
            docs_path = Path('SHAPEFILE_GUIDE.md').absolute()
            webbrowser.open(f'file://{docs_path}')
        except Exception:
            QMessageBox.information(
                self,
                "Documentation",
                "Documentation available in SHAPEFILE_GUIDE.md"
            )
    
    def save_settings(self):
        """Save application settings"""
        self.settings.setValue('geometry', self.saveGeometry())
        self.settings.setValue('windowState', self.saveState())
    
    def restore_settings(self):
        """Restore application settings"""
        geometry = self.settings.value('geometry')
        if geometry:
            self.restoreGeometry(geometry)
        
        state = self.settings.value('windowState')
        if state:
            self.restoreState(state)
    
    def closeEvent(self, event):
        """Handle application close"""
        self.save_settings()
        event.accept()

def main():
    """Main application entry point"""
    app = QApplication(sys.argv)
    app.setApplicationName("Tractobots")
    app.setApplicationVersion("1.0.0")
    
    # Set application icon if available
    try:
        app.setWindowIcon(QIcon('icon.png'))
    except:
        pass
    
    # Create and show main window
    window = TractorbotsDesktopApp()
    window.show()
    
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
