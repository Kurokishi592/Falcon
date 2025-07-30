# CP2106-Orbital-Falcon
A vision-based dynamic landing system for drones (UAVs) on moving platforms autonomously, using real-time AprilTag detection and comes with a GUI. Ideal for Raspberry Pi and compatible with Linux, macOS, and Windows.

# Setting up of Telegram bot
The code for running a Telegram bot that sends warning messages when parameters exceed a certain threshold is included with FALCON. This requires the user to have a Telegram account, and the computer running FALCON must have internet access.

To begin the Telegram setup, search for the @BotFather account on Telegram, which is Telegram's built-in bot manager. To begin, send the command `/newbot` to begin the setup process.
1. Choosing a name for the bot

This name will be the display name on Telegram.

2. Choosing a username for the bot

This is the "@username_bot" Telegram handle for the new bot, and can be found by anybody with a Telegram account.

3. API Key

After the name and username are chosen, BotFather will automatically generate a HTTP API key. 

Duplicate `creds_sample.json` and rename it to `creds.json`, and paste the generated API key inside.

# FALCON Setup Instructions 
1. Download the Project
```bash
cd ~/Desktop
git clone https//github.com/Kurokishi592/Falcon.git
cd Falcon
```
Or alternatively, [download the ZIP file](https://github.com/Kurokishi592/Falcon/archive/refs/heads/main.zip), extract it, and move into under your local Desktop folder.

#### For 🐧 Linux/macOS/Raspberry Pi
If you don't have git, install it using your system's package manager (e.g., sudo apt install git on Raspberry Pi/Linux).

2. To use the Telegram bot, check the steps above for the setup process after the download is complete. 

3. Run the following commands:
``` bash
chmod +x setup_and_run.sh
./setup_and_run.sh
```

#### For 💻 Windows
Download Git Bash if you do not have it installed: https://git-scm.com/downloads, click on "Windows", run the Installer `.exe` file and leave settings at their default values.

4. Open Git Bash from the Start Menu

5. Go to the "Falcon" directory by running:
```bash
cd ~/desktop/Falcon
```

6. Run the following command to run Falcon:
```bash
./setup_and_run.sh
```

This will:
- Create a Python virtual environment
- Install all required packages
- Launch the GUI application


# FALCON Simple User Test
With the FALCON GUI opened, start the camera feed:
1. Under "Select Camera" dropdown options, select the camera that starts with '700', this should start your webcam. (Or any other usb camera you can use)

2. Open the pdf "recursive tag id 0 1 2" to test the apriltag detection using another device. As long as the tag is displayed on a separate screen any method will do. You can just open using your phone.

3. Display the tag in front of your webcam/camera and FALCON will reflect an overlay to show detection. 

- Ensure that the tag is fully displayed on your separate screen
- Zoom in on the pdf. The recursive tag, as the name suggests, has tags placed within tags
- It is robust to rotation of up to 80 degrees, and far or near, try it yourself

Done!
FALCON is now opened with a GUI and your selected camera feed. Feel free to play around with the other features too!



# Required Dependencies
The [GUI](https://github.com/Kurokishi592/Falcon/blob/main/Frontend/Tkinter/tkinter_main.py) relies on Python's Tkinter dependency. Tkinter comes preinstalled with any [Python 3](https://www.python.org/downloads/) version.

Other required dependencies include:
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

4. [Pillow](https://pypi.org/project/pillow/)
Pillow is used for displaying images on the Tkinter GUI
```bash
pip install Pillow
```

5. [pyserial](https://pypi.org/project/pyserial/)
pyserial is used for communicating with the MCU (Teensy 4.1) through UART
```bash
pip install pyserial
```

6. [matplotlib](https://pypi.org/project/matplotlib/)
Matplotlib is used to generate the live graph on the Tkinter GUI
```bash
pip install matplotlib
```

7. [python-telegram-bot](https://pypi.org/project/python-telegram-bot/)
Python interface to control the backend of a Telegram bot
```bash
pip install python-telegram-bot
```
