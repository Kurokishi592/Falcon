#!/bin/bash

echo "Setting up Falcon environment..."

# Create and activate virtual environment
python -m venv .venv
source .venv/Scripts/activate

# Upgrade pip
python.exe -m pip install --upgrade pip

# Install required packages
pip install opencv-python cv2-enumerate-cameras pupil-apriltags Pillow pyserial matplotlib python-telegram-bot

# Run the main script
python main.py