# CP2106-Orbital-Falcon
Falcon

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