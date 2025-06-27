#!/bin/bash

echo "Setting up Falcon environment..."

# Create and activate virtual environment
python3 -m venv .venv
source .venv/bin/activate

# Upgrade pip
pip install --upgrade pip

# Install required packages
pip install opencv-python cv2-enumerate-cameras pupil-apriltags Pillow pyserial

cd Frontend/Tkinter 

# Run the Tkinter application
python tkinter_main.py