import os
import subprocess
import sys

# =================================================================
# 1. COMPREHENSIVE DEPENDENCY MANIFEST
# =================================================================
# Every library required across your entire advanced autopilot system
REQUIRED_PACKAGES = [
    "dronekit",          # Main MAVLink drone connection layer
    "dronekit-sitl",     # Local simulator for safe virtual testing
    "pymavlink",         # Low-level flight controller messaging
    "pygame",            # Cockpit interface and keyboard control canvas
    "opencv-python",     # Computer vision engine for race hoop tracking
    "numpy"              # Matrix math engine for camera pixels
]

def auto_install_pipeline():
    """Automatically installs every package from the manifest with zero manual entry."""
    
    # Establish total count tracking
    total_packages = len(REQUIRED_PACKAGES)
    
    for index, package in enumerate(REQUIRED_PACKAGES, 1):
        # Build the pip execution string bound to your active Python runtime environment
        command_args = [sys.executable, "-m", "pip", "install", package]
        
        # --- AUTOMATED 2026 ENVIRONMENT BYPASS ---
        # Automatically injects break flags if running on modern Linux distributions
        # (like Ubuntu 24.04+ or Raspberry Pi OS Bookworm) that block raw global pip calls.
        if os.path.exists("/usr/lib/python3.11/EXTERNALLY-MANAGED") or \
           os.path.exists("/usr/lib/python3.12/EXTERNALLY-MANAGED") or \
           os.path.exists("/usr/lib/python3.13/EXTERNALLY-MANAGED"):
            command_args.append("--break-system-packages")
            
        # Execute background stream process silently to save CPU cycles
        process = subprocess.run(
            command_args, 
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE
        )
        
        # Immediate safety check: if installation fails, drop out to help debug
        if process.returncode != 0:
            sys.exit(1)

    # --- AUTOMATED INTEGRITY CHECKS ---
    # Instantly validates that your local system can open and mount the libraries
    try:
        import dronekit
        import pygame
        import cv2
        import numpy
    except ImportError:
        # Exit with a system error flag if any library fails to mount properly
        sys.exit(1)
        
    # Exit with a clean status code indicating complete unattended installation success
    sys.exit(0)

