# mercury_sequence_ui
## Description
The Mercury-A1 robotic arm by Elephant Robotics comes with a raspberry pi computer (M5Stack CM4Stack) which is Linux based and has a touch screen for user operation.
The attached python code uses the preinstalled pymycobot.py library to display a Tkinter gui to control the robot.
The library is in active development and can be found at https://github.com/elephantrobotics/pymycobot/tree/main
To update the library without connecting to the internet, replace the pymycobot folder at /home/er/,local/lib/python3.8/site-packages
## Autostart
One of the reasons for creating this software was to allow autostart of a sequence. This has not been implemented in this version yet, but to do so, add a .desktop file to /home/er/.config/autostart
An example is attached: autoruyn.desktop


