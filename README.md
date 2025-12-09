# LiCAS_ECI_UDP
LiCAS External Control Interface (ECI) through UDP sockets. This package developed in C++ is used to send control references to and receive state feedback from the control program of the LiCAS dual arm.

The LiCAS Robotic Arms (Lightweight and Compliant Anthropomorphic Dual Arm Systems) are intended to operate on high altitude workspaces or in remote areas, executing the control program in an onboard computer accessed through SSH session.The LiCAS ECI allows to send motion commands and receive state feedback from the arms through UDP sockets, defining the data packets as C-style structures.

# Installation
Create a LiCAS workspace folder in your preferred location (mkdir licas_ws) and clone the repository there. Then execute the following commands:

cd LiCAS_ECI_UDP

mkdir build

cd build

cmake ..

make

The executable is located within the Main folder

# Customization

Modify the Main.cpp program, particularly the content of the while loop, according to the requirements of the control task. Make sure to recompile with make every time the source code is modified.


# Data logs
The LiCAS_ECI program generates a log file called LiCAS_DataLog.txt that can be plotted with the DataViewer_LiCAS_ECI.m script.

# Run the program
Open a terminal within tbe build/Main folder and run the program providing the IP address of the computer board in which the LiCAS Control Program is executed along with the sending port and receiving port (use preferably the ports indicated below):

./LiCAS_ECI IP_Address 23000 24003



