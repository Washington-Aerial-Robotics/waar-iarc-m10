# KAF Drone Flight Control Software
This is the repositorry for an ESP32 based quadcopter flight controller written in C-like C++ designed for the Washington Aerial Robotics RSO at University of Washington. This flight software is capable of semi-autonomous flight, swarm coordination with other quadcopters, and custom input control. It is able to execute 4-th order polynomial trajectories, position waypoints, direct acceleration control, and manual mode inputs. Some additional features include a web-based ground station and manual controller as well as both indoor and GPS position tracking.

## Setup Instructions

1. Download the contents of the ```waar-iarc-m10/ESP32/KAF_Drone``` directory from the WAAR GitHub [repository](https://github.com/Washington-Aerial-Robotics/waar-iarc-m10/tree/main/ESP32/KAF_Drone), and extract its contents if necessary.
2. From the download, identify the  ```KAF_Drone``` folder, and move this folder to the file location of ```C:\Users\[USERNAME]\Documents\Arduino\libraries\ ``` on your local computer.
3. Check that the ArduinoIDE on your computer has been properly installed and configured. If not, follow the instructions listed in the new member onboarding instructions document provided by Francisco.
4. Open ```C:\Users\[USERNAME]\Documents\Arduino\libraries\KAF_Drone\flashing_examples\firmware_full\firmware_full.ino``` in ArduinoIDE.
5. Connect your flight controller or ESP32 devboard to your computer, then flash the firmware_full Arduino sketch.
6. Verify that the flight controller is operational by checking for the appearance of a WiFi network named "KAF_Quadcopter_Drone" and by a proper serial port print-out at a baudrate of 115200.
