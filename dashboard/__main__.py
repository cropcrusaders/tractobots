#!/usr/bin/env python3
"""
Tractobots Dashboard Main Entry Point
"""

import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import the main dashboard launcher
from dashboard.launcher import main

if __name__ == "__main__":
    main()
