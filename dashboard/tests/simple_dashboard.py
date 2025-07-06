#!/usr/bin/env python3
"""
Simplified Tractobots Dashboard for testing
"""

import sys
import os

# Import required packages directly
try:
    from PyQt5.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QPushButton, QLabel, QGridLayout, QGroupBox, QTextEdit, QTabWidget,
        QFileDialog, QScrollArea, QSizePolicy, QSpacerItem, QShortcut
    )
    from PyQt5.QtGui import QFont, QIcon, QColor, QPalette
    from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject, pyqtSlot, QThread
    
    import matplotlib
    matplotlib.use('Qt5Agg')
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
    from matplotlib.figure import Figure
    
    import numpy as np
    import roslibpy
    
    DEPENDENCIES_MET = True
    print("[INFO] All dependencies loaded successfully")
    
except ImportError as e:
    print(f"[ERROR] Missing dependency: {e}")
    DEPENDENCIES_MET = False


class SimpleDashboard(QMainWindow):
    """Minimal dashboard to test basic functionality"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Tractobots Simple Dashboard")
        self.setGeometry(100, 100, 800, 600)
        self.setup_ui()
        
    def setup_ui(self):
        # Create main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        
        # Add a title
        title_label = QLabel("Tractobots Simple Dashboard")
        title_label.setFont(QFont("Arial", 18, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title_label)
        
        # Add a status label
        self.status_label = QLabel("Dashboard is running")
        self.status_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(self.status_label)
        
        # Add an interactive terminal
        terminal_group = QGroupBox("Interactive Terminal")
        terminal_layout = QVBoxLayout(terminal_group)
        
        self.log_text_edit = QTextEdit()
        self.log_text_edit.setReadOnly(True)
        self.log_text_edit.append("====================================")
        self.log_text_edit.append("  TRACTOBOTS INTERACTIVE TERMINAL")
        self.log_text_edit.append("====================================")
        self.log_text_edit.append("Terminal is working!")
        self.log_text_edit.append("====================================")
        terminal_layout.addWidget(self.log_text_edit)
        
        self.command_input = QTextEdit()
        self.command_input.setMaximumHeight(50)
        self.command_input.setPlaceholderText("Type command here...")
        terminal_layout.addWidget(self.command_input)
        
        self.execute_btn = QPushButton("Execute")
        self.execute_btn.clicked.connect(self.execute_command)
        terminal_layout.addWidget(self.execute_btn)
        
        main_layout.addWidget(terminal_group)
        
        # Add test buttons
        buttons_layout = QHBoxLayout()
        
        self.test_btn = QPushButton("Test Terminal")
        self.test_btn.clicked.connect(self.test_terminal)
        buttons_layout.addWidget(self.test_btn)
        
        self.quit_btn = QPushButton("Quit")
        self.quit_btn.clicked.connect(self.close)
        buttons_layout.addWidget(self.quit_btn)
        
        main_layout.addLayout(buttons_layout)
    
    def execute_command(self):
        """Execute a command typed in the terminal"""
        command = self.command_input.toPlainText().strip()
        if command:
            self.log_text_edit.append(f"\n> {command}")
            self.log_text_edit.append(f"Command received: {command}")
            self.command_input.clear()
            
    def test_terminal(self):
        """Test that the terminal is working"""
        self.log_text_edit.append("\n[TEST] Terminal is working correctly!")


def main():
    """Main function to run the application"""
    app = QApplication(sys.argv)
    window = SimpleDashboard()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    if DEPENDENCIES_MET:
        main()
    else:
        print("[ERROR] Cannot start dashboard - dependencies missing")
