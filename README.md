# CP2106-Orbital-Falcon
A vision-based dynamic landing system for drones (UAVs) on moving platforms autonomously, using real-time AprilTag detection and comes with a GUI. Ideal for Raspberry Pi and compatible with Linux, macOS, and Windows.

# Setup Instructions 
1. Download the Project
```bash
cd ~/Desktop
git clone https//github.com/Kurokishi592/Falcon.git
cd Falcon
```
Or alternatively, [download the ZIP file](https://github.com/Kurokishi592/Falcon/archive/refs/heads/main.zip), extract it, and move into under your local Desktop folder.

## For 🐧 Linux/macOS/Raspberry Pi
If you don't have git, install it using your system's package manager (e.g., sudo apt install git on Raspberry Pi/Linux).

2. Run the following commands:
``` bash
chmod +x setup_and_run.sh
./setup_and_run.sh
```

## For 💻 Windows
Download Git Bash if you do not have it installed: https://git-scm.com/downloads, click on "Windows", run the Installer `.exe` file and leave settings at their default values.

2. Open Git Bash from the Start Menu

3. Go to the "Falcon" directory by running:
```bash
cd ~/desktop/Falcon
```

4. Run the following command to run Falcon:
```bash
./setup_and_run.sh
```

This will:
- Create a Python virtual environment
- Install all required packages
- Launch the GUI application

Done!
The AprilTag detection window should open with your webcam feed and a GUI.


---------------------------------------------------


# Installation Instructions
The [GUI](https://github.com/Kurokishi592/Falcon/blob/main/Frontend/Tkinter/tkinter_main.py) relies on Python's Tkinter dependency. Tkinter comes preinstalled with any [Python 3](https://www.python.org/downloads/) version.

Other dependencies include:
1. [OpenCV](https://opencv.org/) 

OpenCV is used for interfacing with camera. It can be installed with the following bash command:
```bash
pip install opencv-python
```

2. [cv2-enumerate-cameras](https://pypi.org/project/cv2-enumerate-cameras/)

cv2-enumerate-cameras is used for listing all the cameras present on the system. It can be installed with the following bash command:
```bash
pip install cv2-enumerate-cameras
```

3. [pupil-apriltags](https://pypi.org/project/pupil-apriltags/)
pupil-apriltags is used for the AprilTag detection.
```bash
pip install pupil-apriltags
```

# Runtime Instructions
To run the GUI, run the frontend Python file. This can be done through the terminal, after navigating to the folder that the file is located in:
```bash
python tkinter_main.py
```

Otherwise, ```tkinter_main.exe``` [(link (Deprecated for now))]() can be run. However, as the application has not been signed by any antivirus companies, the active antivirus (Windows Defender or otherwise) is likely to produce a [malware warning](https://stackoverflow.com/questions/62095008/i-am-not-allowed-to-run-a-python-executable-on-other-pcs). If using Windows Defender, press on the ```More info``` button, and then the ```Run Anyway``` button. For other antiviruses, add the ```.exe``` to the exclusion or white list for the executable to lauch properly.

If the user is comfortable building the ```.exe``` themselves, it can be done with ```pyinstaller```. [It](https://pypi.org/project/pyinstaller/) can be installed with the following bash command:
```bash
pip install pyinstaller
```

After navigating to the directory with ```tkinter_main.py```, run the following bash command to build the ```.exe```:
```bash
pyinstaller -F tkinter_main.py
```