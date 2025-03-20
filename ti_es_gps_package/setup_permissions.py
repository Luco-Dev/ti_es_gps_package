#!/usr/bin/env python3

import os
import subprocess

def set_usb_permissions():
    print("Setting up permissions for /dev/ttyUSB0...")

    # Add user to dialout group
    subprocess.run(["sudo", "usermod", "-a", "-G", "dialout", os.getenv("USER")])

    # Set chmod for immediate effect (temporary)
    subprocess.run(["sudo", "chmod", "666", "/dev/ttyUSB0"])

    print("Permissions updated. Please log out and log back in if needed.")

if __name__ == "__main__":
    set_usb_permissions()
